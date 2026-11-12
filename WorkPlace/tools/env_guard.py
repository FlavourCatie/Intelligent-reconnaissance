# -*- coding: utf-8 -*-
"""
env_guard.py —— 一条命令体检"环境有没有被污染 / 飞控起没起来"
================================================================================

为什么需要它
------------
这套课的运行环境是**共享且脆弱**的：

* 只有 Python **3.6** 能用（SDK 的 `swarmae.cp36-win_amd64.pyd`、
  `libcarla.cp36-win_amd64.pyd` 的名字里写死了 ABI），一旦有人顺手
  `pip install` 升级了 numpy / setuptools，整个环境就悄悄变样；
* SDK 是用 `easy_install` 装进 conda 环境 `kyxz` 的（不是 venv），
  装的东西全局可见，**谁装谁污染**；
* 课程包（10 GB）不在仓库里，每个人放的位置还不一样，路径带空格就直接
  把飞控打瘸（见 `docs/飞控调通_根因与修复.md`）。

所以把"体检"做成一条命令，**只读、不装任何东西**，输出一张 PASS/WARN/FAIL 表，
出问题直接给可执行的下一步。

检查项
------
    1. Python 版本 / 位数            （必须 3.6 / 64 位）
    2. 路径卫生                     （空格、中文；空格 = 飞控致命）
    3. 关键依赖版本                 （对不上 = 环境被污染的证据）
    4. SDK 可导入 + 关键接口在位     （含"launch_kinetic 是空壳"这一事实）
    5. 课程包文件                  （main-release / driver / SITL model / config.yaml）
    6. 端口                        （2000 CARLA、4443/4444 主控、3301/3302 驱动）
    7. 进程与启动顺序               （CarlaUE4 / main-release / driver / subject4）

工具
----
    --snapshot PATH   把当前环境指纹存成 JSON（建议提交到 env/env_snapshot.json）
    --diff PATH       跟已存的指纹比对，**列出谁动了什么**（抓环境漂移最快的手段）

用法
----
    conda activate kyxz
    python WorkPlace\\tools\\env_guard.py
    python WorkPlace\\tools\\env_guard.py --snapshot WorkPlace\\env\\env_snapshot.json
    python WorkPlace\\tools\\env_guard.py --diff     WorkPlace\\env\\env_snapshot.json

退出码
------
    0 = 没有 FAIL（可以有 WARN）
    1 = 有 FAIL
    2 = 工具自己跑不起来（缺路径等）
"""

from __future__ import print_function

import argparse
import hashlib
import json
import os
import socket
import sys

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
WORKPLACE_DIR = os.path.dirname(THIS_DIR)
sys.path.insert(0, WORKPLACE_DIR)

from core import env_paths  # noqa: E402

#: 课程包 + 踩坑记录共同确定的版本基线（改这里之前先想清楚）
EXPECTED_PYTHON = (3, 6)
EXPECTED_PACKAGES = [
    ("numpy", "1.19.5"),
    ("Pillow", "7.0.0"),
    ("requests", "2.22.0"),
    ("matplotlib", "3.3.4"),
    ("scipy", "1.5.4"),
    ("opencv-python", "4.4.0.40"),
    ("setuptools", "44.1.0"),
    ("pygame", None),          # 版本不敏感，只要在
]

#: 端口 → 用途
PORTS = [
    (2000, "CARLA 场景服务（SDK 连这里）", "tcp"),
    (4443, "main-release: UE4 上报口", "tcp"),
    (4444, "main-release: 飞控驱动登录口", "tcp"),
    (3301, "driver: 消息口", "udp"),
    (3302, "driver: 指令口", "udp"),
]

#: 需要留指纹的课程包文件（相对课程 environment 目录）
FINGERPRINT_FILES = [
    ("simu_driver_windows/bin/main-release.exe", "main-release（飞控主控）"),
    ("simu_driver_windows/bin/driver/driver.exe", "driver（飞控驱动）"),
    ("simu_driver_windows/bin/driver/config.yaml", "driver 配置（Type 在这）"),
    ("simu_driver_windows/bin/config.yaml", "main-release 配置（端口表）"),
    ("sdk/windows/swarmae-1.1-py3.6-win-amd64.egg", "SDK 本体"),
    ("sdk/windows/swarmae_lib-0.1.1-py3.6-win-amd64.egg", "libcarla 绑定"),
]


# ---------------------------------------------------------------------------
#  小工具
# ---------------------------------------------------------------------------
OK, WARN, FAIL, SKIP = "PASS", "WARN", "FAIL", "SKIP"


class Report(object):
    """收集检查结果，最后统一渲染。"""

    def __init__(self):
        self.rows = []
        self.fixes = []

    def add(self, level, item, detail="", fix=None):
        self.rows.append((level, item, detail))
        if fix:
            self.fixes.append(fix)

    def count(self, level):
        return len([r for r in self.rows if r[0] == level])

    def render(self):
        width = max([len(r[1]) for r in self.rows] + [10])
        for level, item, detail in self.rows:
            print("  [%-4s] %-*s %s" % (level, width, item, detail))
        print()
        print("  小计：PASS %d / WARN %d / FAIL %d / SKIP %d"
              % (self.count(OK), self.count(WARN), self.count(FAIL), self.count(SKIP)))
        if self.fixes:
            print()
            print("  建议动作：")
            seen = []
            for fix in self.fixes:
                if fix not in seen:
                    seen.append(fix)
            for fix in seen:
                print("    · " + fix)


def _sha256(path, limit=None):
    """算文件指纹；返回 (大小, sha256 前 16 位)。

    默认**整文件**算，好跟 docs/当前环境指纹.md 里那张表直接对数字。
    """
    h = hashlib.sha256()
    size = os.path.getsize(path)
    with open(path, "rb") as fp:
        remain = size if limit is None else min(size, limit)
        while remain > 0:
            chunk = fp.read(min(65536, remain))
            if not chunk:
                break
            h.update(chunk)
            remain -= len(chunk)
    return size, h.hexdigest().upper()[:16]


def _pkg_version(name):
    """取已安装包版本，不 import 包本体（避免有副作用的初始化）。"""
    try:
        import pkg_resources  # noqa: F401
    except Exception:
        return None
    try:
        import pkg_resources
        return pkg_resources.get_distribution(name).version
    except Exception:
        return None


def _port_open(port, proto="tcp", host="127.0.0.1", timeout=0.35):
    """tcp: 能连上 = 有人在听；udp: bind 失败 = 有人在听。"""
    if proto == "tcp":
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        try:
            sock.connect((host, port))
            return True
        except OSError:
            return False
        finally:
            sock.close()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((host, port))
        return False          # 能 bind 说明没人在听
    except OSError:
        return True
    finally:
        sock.close()


# ---------------------------------------------------------------------------
#  各项检查
# ---------------------------------------------------------------------------
def check_python(rep):
    info = sys.version_info
    detail = "%d.%d.%d (%d 位)" % (info[0], info[1], info[2],
                                   64 if sys.maxsize > 2 ** 32 else 32)
    if (info[0], info[1]) != EXPECTED_PYTHON:
        rep.add(FAIL, "Python 版本", detail + " —— 必须是 3.6（SDK 是 cp36 扩展）",
                "conda activate kyxz（若没有该环境，见 env/README.md）")
    elif not sys.maxsize > 2 ** 32:
        rep.add(FAIL, "Python 位数", detail + " —— SDK 只有 win-amd64",
                "换 64 位 Python")
    else:
        rep.add(OK, "Python 版本", detail)


def find_space_free_entry():
    """返回一个"无空格且含飞控驱动"的入口目录；没有则 None（逻辑在 core/env_paths）。"""
    return env_paths.find_space_free_entry()


def check_paths(rep):
    alias = find_space_free_entry()

    # 仓库根：带空格只是"用起来别扭"（工具、脚本、部分二进制），不直接打死飞控
    problems = env_paths.path_problems(env_paths.REPO_ROOT)
    if any(c == "space" for c, _ in problems):
        rep.add(WARN, "仓库根 路径", env_paths.REPO_ROOT + " —— 含空格（能用，但坑多）",
                "想彻底干净就把仓库放到 C:\\work\\... 这类路径下")
    elif problems:
        rep.add(WARN, "仓库根 路径", "含非 ASCII 字符")
    else:
        rep.add(OK, "仓库根 路径", env_paths.REPO_ROOT)

    # 课程环境 / 飞控驱动：这里带空格就是飞控的致命伤，除非有无空格入口
    for label, path in (("课程环境 路径", env_paths.find_course_env()),
                        ("飞控驱动 路径", env_paths.find_simu_driver())):
        if not path:
            rep.add(WARN, label, "没找到（课程包不在常见位置）")
            continue
        spaces = [c for c, _ in env_paths.path_problems(path) if c == "space"]
        if spaces and alias:
            rep.add(WARN, label, path + " —— 含空格，但已有无空格入口")
        elif spaces:
            rep.add(FAIL, label, path + " —— 含空格（SITL 命令行会被劈开）",
                    "python WorkPlace\\tools\\sim_link.py --create   "
                    "然后从 C:\\kyxz\\... 启动 start.bat")
        elif env_paths.path_problems(path):
            rep.add(WARN, label, path + " —— 含非 ASCII 字符")
        else:
            rep.add(OK, label, path)

    # 飞控入口：决定"从哪启动 start.bat"
    if alias:
        rep.add(OK, "飞控无空格入口", alias)
    elif env_paths.find_simu_driver() and not [
            c for c, _ in env_paths.path_problems(env_paths.find_simu_driver())
            if c == "space"]:
        rep.add(OK, "飞控无空格入口", "原路径本身无空格，直接用")
    else:
        rep.add(FAIL, "飞控无空格入口", "没有（飞控一定会超时）",
                "python WorkPlace\\tools\\sim_link.py --create")


def check_packages(rep):
    drift = []
    for name, want in EXPECTED_PACKAGES:
        have = _pkg_version(name)
        if have is None:
            rep.add(FAIL, "包 " + name, "未安装",
                    "见 env/requirements.txt 用 pip 装回来（别装新版）")
            drift.append(name)
        elif want and have != want:
            rep.add(WARN, "包 " + name, "%s（基线 %s）" % (have, want),
                    "环境可能被升级过：%s 需要 %s" % (name, want))
            drift.append(name)
        else:
            rep.add(OK, "包 " + name, have or "已安装")
    return drift


def check_sdk(rep):
    try:
        import swarmae  # noqa: F401
        from swarmae.SwarmAENode import SwarmAENode
    except Exception as exc:
        rep.add(FAIL, "SDK 导入", "失败：%r" % (exc,),
                "确认在 kyxz 环境里跑；SDK 安装见 env/README.md")
        return
    rep.add(OK, "SDK 导入", "swarmae 可用")

    required = ["control_kinetic_simply", "control_kinetic_simply_global",
                "get_kinetic_data_simply", "get_location", "detect_situation",
                "apply_vehicle_control", "send_msg", "receive_msg"]
    missing = [n for n in required if not hasattr(SwarmAENode, n)]
    if missing:
        rep.add(FAIL, "SDK 接口", "缺 %s" % ", ".join(missing))
    else:
        rep.add(OK, "SDK 接口", "动力学 / 探测 / 车辆控制接口齐全")

    # launch_kinetic 在 V1.4 里被删除，SDK 里只剩一个 `return None` 的空壳。
    # 这不是"灵异事件"，把事实写进体检，省得组员再去枚举 18 组参数。
    fn = getattr(SwarmAENode, "launch_kinetic", None)
    stub = False
    if fn is not None and getattr(fn, "__code__", None) is not None:
        consts = [c for c in fn.__code__.co_consts if not hasattr(c, "co_code")]
        stub = consts and consts[-1] is None and len(fn.__code__.co_code) <= 8
    if stub:
        rep.add(WARN, "launch_kinetic", "空壳（return None）—— V1.4 已删除该接口",
                "别再用它启动动力学；飞机由 driver 自动实例化")
    elif fn is None:
        rep.add(OK, "launch_kinetic", "不存在（V1.4 已删除，正常）")


def check_course_files(rep):
    env = env_paths.find_course_env()
    if not env:
        rep.add(FAIL, "课程包", "没找到 environment 目录",
                "把 environment_forclass 放到工作区根，或用 SWARMAE_ENV_ROOT 指定")
        return
    required = ["simu_driver_windows/bin/main-release.exe",
                "simu_driver_windows/bin/driver/driver.exe",
                "simu_driver_windows/start.bat",
                "sdk/windows/swarmae-1.1-py3.6-win-amd64.egg"]
    missing = [p for p in required if not os.path.isfile(os.path.join(env, *p.split("/")))]
    if missing:
        rep.add(FAIL, "课程包文件", "缺 %d 个" % len(missing),
                "课程包不完整，重新解压 environment_forclass")
    else:
        rep.add(OK, "课程包文件", "main-release / driver / start.bat / egg 在位")

    sitl = env_paths.find_sitl_dir("quadrotor")
    if not sitl:
        rep.add(FAIL, "SITL 二进制", "找不到 quadrotor_windows/model",
                "课程包不完整（driver/bin 目录）")
    elif any(c == "space" for c, _ in env_paths.path_problems(sitl)):
        if find_space_free_entry():
            rep.add(WARN, "SITL 路径", "原路径含空格；请走无空格入口启动")
        else:
            rep.add(FAIL, "SITL 路径", "含空格 → 收不到 MAVLink 心跳",
                    "python WorkPlace\\tools\\sitl_probe.py   # 复现")
    else:
        rep.add(OK, "SITL 路径", "无空格")

    dtype = env_paths.read_driver_type()
    if dtype is None:
        rep.add(WARN, "driver Type", "读不到 config.yaml")
    elif dtype == "quadrotor":
        rep.add(OK, "driver Type", "quadrotor（四旋翼）")
    else:
        rep.add(WARN, "driver Type", "%s（不是 quadrotor）" % dtype,
                "start.bat 选 2，或改 bin/driver/config.yaml")


def check_ports(rep):
    for port, label, proto in PORTS:
        open_ = _port_open(port, proto)
        if open_:
            rep.add(OK, "端口 %d" % port, "%s 在听" % label)
        else:
            rep.add(WARN, "端口 %d" % port, "%s 没人在听" % label)
    return


def check_processes(rep):
    try:
        from core import preflight
    except Exception as exc:
        rep.add(SKIP, "进程检查", "导入 preflight 失败：%r" % (exc,))
        return
    table = preflight._process_table()
    if table is None:
        rep.add(SKIP, "进程检查", "拿不到进程表")
        return
    names = ["carlaue4.exe", "main-release.exe", "driver.exe", "subject4.exe"]
    for exe in names:
        pids = table.get(exe, [])
        if len(pids) > 1:
            rep.add(WARN, "进程 " + exe, "有 %d 个实例（会抢端口）" % len(pids),
                    "python WorkPlace\\tools\\kill_all.py")
        elif pids:
            rep.add(OK, "进程 " + exe, "运行中 pid=%d" % pids[0])
        else:
            rep.add(WARN, "进程 " + exe, "未运行")
    res = preflight.check()
    if res.get("order_ok") is False:
        rep.add(FAIL, "启动顺序", "飞控早于 CarlaUE4（绑不上飞机）",
                "按 CarlaUE4 → start.bat → subject4.exe 重启")


# ---------------------------------------------------------------------------
#  指纹 / 漂移比对
# ---------------------------------------------------------------------------
def collect_fingerprint():
    """收集一份可比对的环境指纹（纯数据，可提交进仓库）。"""
    data = {
        "python": "%d.%d.%d" % sys.version_info[:3],
        "platform": sys.platform,
        "packages": {},
        "files": {},
        "paths": {
            "repo_root": env_paths.REPO_ROOT,
            "course_env": env_paths.find_course_env(),
            "simu_driver": env_paths.find_simu_driver(),
        },
        "driver_type": env_paths.read_driver_type(),
    }
    for name, _want in EXPECTED_PACKAGES:
        data["packages"][name] = _pkg_version(name)
    env = env_paths.find_course_env()
    if env:
        for rel, label in FINGERPRINT_FILES:
            path = os.path.join(env, *rel.split("/"))
            if os.path.isfile(path):
                size, digest = _sha256(path)
                data["files"][rel] = {"size": size, "sha256_16": digest,
                                      "label": label}
    return data


def diff_fingerprint(old, new):
    """返回 [(项, 旧值, 新值), ...]。"""
    changes = []

    def walk(prefix, a, b):
        keys = sorted(set(list(a.keys()) + list(b.keys())))
        for key in keys:
            va = a.get(key, "<缺>")
            vb = b.get(key, "<缺>")
            if isinstance(va, dict) and isinstance(vb, dict):
                walk(prefix + key + ".", va, vb)
            elif va != vb:
                changes.append((prefix + key, va, vb))

    walk("", {"python": old.get("python"), "driver_type": old.get("driver_type"),
              "packages": old.get("packages", {}), "files": old.get("files", {}),
              "paths": old.get("paths", {})},
         {"python": new.get("python"), "driver_type": new.get("driver_type"),
          "packages": new.get("packages", {}), "files": new.get("files", {}),
          "paths": new.get("paths", {})})
    return changes


# ---------------------------------------------------------------------------
def main(argv=None):
    parser = argparse.ArgumentParser(
        description="环境体检 / 指纹快照 / 漂移比对（只读，不装任何东西）")
    parser.add_argument("--snapshot", metavar="PATH",
                        help="把当前指纹写进 JSON")
    parser.add_argument("--diff", metavar="PATH", nargs="?", const="",
                        help="跟指纹文件比对（不给路径则用 WorkPlace/env/env_snapshot.json）")
    parser.add_argument("--json", action="store_true", help="以 JSON 输出体检结果")
    parser.add_argument("--quiet", action="store_true", help="只输出 FAIL/WARN")
    args = parser.parse_args(argv)

    if args.diff is not None:
        path = args.diff or os.path.join(WORKPLACE_DIR, "env", "env_snapshot.json")
        if not os.path.isfile(path):
            print("没有指纹文件：%s" % path)
            print("先生成一份：python WorkPlace\\tools\\env_guard.py --snapshot \"%s\"" % path)
            return 2
        with open(path, "rb") as fp:
            old = json.loads(fp.read().decode("utf-8"))
        changes = diff_fingerprint(old, collect_fingerprint())
        print("比对基线：%s" % path)
        if not changes:
            print("  没有任何变化 —— 环境干净。")
            return 0
        print("  发现 %d 处变化：" % len(changes))
        for item, before, after in changes:
            print("    %-40s %s  ->  %s" % (item, before, after))
        print()
        print("  提示：只要不是你自己改的，一律按\"被污染\"处理并回滚到基线版本。")
        return 1

    rep = Report()
    print("=" * 78)
    print("环境体检  (%s)" % __file__)
    print("=" * 78)

    check_python(rep)
    check_paths(rep)
    check_packages(rep)
    check_sdk(rep)
    check_course_files(rep)
    check_ports(rep)
    check_processes(rep)

    print()
    report_obj = rep
    if args.quiet:
        slim = Report()
        slim.rows = [r for r in report_obj.rows if r[0] in (WARN, FAIL)]
        slim.fixes = report_obj.fixes
        report_obj = slim
    report_obj.render()

    if args.json:
        print()
        print(json.dumps({"rows": report_obj.rows}, ensure_ascii=False, indent=2))

    if args.snapshot:
        data = collect_fingerprint()
        folder = os.path.dirname(os.path.abspath(args.snapshot))
        if folder and not os.path.isdir(folder):
            os.makedirs(folder)
        import io
        # newline="\n"：仓库 .gitattributes 规定 json 走 LF，别让 Windows 写出 CRLF
        with io.open(args.snapshot, "w", encoding="utf-8", newline="\n") as fp:
            fp.write(json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True))
        print()
        print("指纹已写入：%s" % args.snapshot)

    return 1 if report_obj.count(FAIL) else 0


if __name__ == "__main__":
    sys.exit(main())
