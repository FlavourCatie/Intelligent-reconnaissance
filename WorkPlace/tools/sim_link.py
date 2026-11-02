# -*- coding: utf-8 -*-
"""
sim_link.py —— 给课程环境做一个"无空格"入口（飞控根因的一键修复）
================================================================================

问题
----
飞控驱动 `driver.exe` 拼 SITL 命令行时不加引号，工作区路径一带空格
（`D:\\Intelligent Unmanned Systems`），`--defaults <路径>` 就被劈成两半，
`--serial0 udpclient:...` 整段丢失，SITL 退回默认 TCP 5760，飞控再也收不到心跳。
证据（可复现）：`tools/sitl_probe.py` 里"拼串"方式 0 个心跳，
"参数列表"方式 13 个心跳。

为什么不能靠改代码解决
----------------------
`driver.exe` 是别人打包好的二进制（PyInstaller + dronekit），源码不在我们手里，
改不了它的拼串方式。能改的只有**它脚下的路径**。

做法：建一个目录联接（junction）
--------------------------------
    mklink /J C:\\kyxz  "D:\\Intelligent Unmanned Systems\\environment_forclass"

联接（junction）不复制数据、不占空间、不需要管理员权限、可以随时删，
删掉也不影响真实目录。之后**从联接路径启动飞控**：

    C:\\kyxz\\environment\\simu_driver_windows\\start.bat

`driver.exe` 的 `sys.executable` 就变成 `C:\\kyxz\\...`，一路无空格，
SITL 参数就完整了。CarlaUE4 / subject4.exe 不受影响，仍可用原路径。

（本项目还提供另一种等价做法：把整个课程环境搬到 C:\\kyxz。
两者效果相同，联接更省事、可回退，推荐这个。）

用法
----
    python WorkPlace\\tools\\sim_link.py                 # 看现状 + 建议
    python WorkPlace\\tools\\sim_link.py --create         # 建 C:\\kyxz 联接
    python WorkPlace\\tools\\sim_link.py --create --link D:\\kyxz
    python WorkPlace\\tools\\sim_link.py --remove         # 删掉联接（不删数据）

判读
----
    本脚本只建"入口"，不改课程包的任何一个字节。
    建完请按它打印的顺序启动，再用 tools\\sitl_probe.py --path <联接路径> 复核。
"""

from __future__ import print_function

import argparse
import os
import subprocess
import sys

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
WORKPLACE_DIR = os.path.dirname(THIS_DIR)
sys.path.insert(0, WORKPLACE_DIR)

from core import env_paths  # noqa: E402

#: 默认联接位置（无空格、短、好记）
DEFAULT_LINK = "C:\\kyxz"

FILE_ATTRIBUTE_REPARSE_POINT = 0x400


def is_reparse_point(path):
    """判断路径是不是 junction / 符号链接（不跟进去）。"""
    if os.name != "nt":
        return os.path.islink(path)
    try:
        import ctypes
        attrs = ctypes.windll.kernel32.GetFileAttributesW(str(path))
        if attrs in (-1, 0xFFFFFFFF):
            return False
        return bool(attrs & FILE_ATTRIBUTE_REPARSE_POINT)
    except Exception:
        return False


def link_target(path):
    """读 junction 指向的真实目录；读不到返回 None。"""
    try:
        import ctypes
        buf = ctypes.create_unicode_buffer(1024)
        # GetFinalPathNameByHandle 需要句柄，这里用更省事的办法：
        # 直接打开目录再查询，失败就返回 None。
        handle = ctypes.windll.kernel32.CreateFileW(
            str(path), 0, 0x00000007, None, 3, 0x02000000, None)
        if handle in (None, -1, 0xFFFFFFFFFFFFFFFF):
            return None
        try:
            size = ctypes.windll.kernel32.GetFinalPathNameByHandleW(
                handle, buf, 1024, 0)
            if size <= 0:
                return None
            text = buf.value
            if text.startswith("\\\\?\\"):
                text = text[4:]
            return text
        finally:
            ctypes.windll.kernel32.CloseHandle(handle)
    except Exception:
        return None


def create_junction(link, target):
    """用 mklink /J 建联接（junction 不需要管理员）。返回 (ok, 输出)。"""
    cmd = 'mklink /J "%s" "%s"' % (link, target)
    proc = subprocess.Popen(cmd, shell=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    out = proc.communicate()[0]
    text = out.decode("mbcs", "replace") if out else ""
    return proc.returncode == 0, text.strip()


def remove_junction(link):
    """删联接（rmdir 对 junction 只删链接，不删真实内容）。"""
    if not is_reparse_point(link):
        return False, "%s 不是联接，拒绝删除（防止误删真实目录）" % link
    proc = subprocess.Popen('rmdir "%s"' % link, shell=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    out = proc.communicate()[0]
    text = out.decode("mbcs", "replace") if out else ""
    return proc.returncode == 0, text.strip()


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="给课程环境建一个不含空格的联接入口，修掉飞控 SITL 被劈开的问题")
    parser.add_argument("--link", default=DEFAULT_LINK,
                        help="联接路径（默认 %s）" % DEFAULT_LINK)
    parser.add_argument("--create", action="store_true", help="创建联接")
    parser.add_argument("--remove", action="store_true", help="删除联接")
    parser.add_argument("--json", action="store_true", help="输出 JSON")
    args = parser.parse_args(argv)

    link = os.path.abspath(args.link)
    env = env_paths.find_course_env()

    if args.remove:
        ok, text = remove_junction(link)
        print(text or ("已删除 %s" % link))
        if ok:
            print("真实目录未被改动。")
        return 0 if ok else 1

    print("课程环境目录 : %s" % (env or "（没找到）"))
    if env:
        base = os.path.dirname(env)          # environment_forclass
        print("要联接的目录 : %s" % base)
        for code, msg in env_paths.path_problems(base):
            print("  !! [%s] %s" % (code, msg))
    print("联接路径     : %s" % link)
    if is_reparse_point(link):
        print("联接现状     : 已存在 -> %s" % (link_target(link) or "（读不到目标）"))
    elif os.path.exists(link):
        print("联接现状     : 该路径已存在且不是联接，请换 --link")
    else:
        print("联接现状     : 不存在")

    if not env:
        print()
        print("找不到课程环境，先确认 environment_forclass 放在哪，")
        print("或用环境变量 SWARMAE_ENV_ROOT 指过去。")
        return 2

    base = os.path.dirname(env)
    patch_ok = not env_paths.path_problems(base)

    if not args.create:
        print()
        if patch_ok:
            print("路径本身不含空格，不需要联接。")
        else:
            print("路径含空格 → 飞控 SITL 会被劈开。建议执行：")
            print("    python WorkPlace\\tools\\sim_link.py --create")
        return 0

    if is_reparse_point(link):
        print()
        print("联接已存在，直接用它即可。")
    else:
        ok, text = create_junction(link, base)
        print()
        print(text or ("建立联接: %s -> %s" % (link, base)))
        if not ok:
            print("建立失败。可手动执行（普通权限即可）：")
            print('    mklink /J "%s" "%s"' % (link, base))
            return 1

    linked_driver = os.path.join(link, "environment", "simu_driver_windows")
    batt = os.path.join(linked_driver, "start.bat")
    sitl = os.path.join(linked_driver, "bin", "driver", "bin",
                        "quadrotor_windows")
    print()
    print("=" * 78)
    print("以后按这个顺序启动（飞控一定要从联接路径起）：")
    print("=" * 78)
    print("  1) CarlaUE4（原路径就行）")
    print("  2) %s   → 选 2（四旋翼）" % batt)
    print("  3) subject4.exe（原路径）")
    print("  4) 算法脚本")
    print()
    print("复核（应看到两种方式都 PASS）：")
    print("  python WorkPlace\\tools\\sitl_probe.py --path \"%s\"" % sitl)
    print()
    print("不想要了随时删：")
    print("  python WorkPlace\\tools\\sim_link.py --remove")
    return 0


if __name__ == "__main__":
    sys.exit(main())
