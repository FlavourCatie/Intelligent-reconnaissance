# -*- coding: utf-8 -*-
"""
kill_all.py —— 一键清场：结束仿真相关的所有进程
================================================================================

为什么要这个
------------
CarlaUE4 允许端口复用，很容易在没关旧实例的情况下又开一个：

```
2000  TCP  43872  carlaue4.exe
2000  TCP  45252  carlaue4.exe     ← 两个都在听 2000
```

这时 `subject4.exe` 连 `127.0.0.1:2000` 连到哪一个是不确定的，
经常会连到"空的那个"，于是**没有任何场景、也没有报错就退出**。

本工具把下面这些进程全部结束，回到干净状态：

    CarlaUE4.exe               仿真器
    subject4.exe               场景脚本
    main-release.exe / driver  飞控驱动（start.bat 起的）

用法
----
    python WorkPlace\\tools\\kill_all.py            # 直接结束
    python WorkPlace\\tools\\kill_all.py --dry-run  # 只看会结束谁
    python WorkPlace\\tools\\kill_all.py --check    # 只看当前状态

建议以【管理员身份】运行，否则可能"拒绝访问"。
"""

from __future__ import print_function

import argparse
import os
import subprocess
import sys
import time

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
WORKPLACE_DIR = os.path.dirname(THIS_DIR)
sys.path.insert(0, WORKPLACE_DIR)

TARGETS = [
    ("CarlaUE4.exe", "仿真器"),
    ("CarlaUE4-Win64-Shipping.exe", "仿真器（Shipping）"),
    ("subject4.exe", "场景脚本"),
    ("subject3.exe", "场景脚本（D3）"),
    ("main-release.exe", "飞控主控"),
    ("driver.exe", "飞控驱动"),
    # 【新增】SITL 本体。driver.exe 每次启动会拉起 3 个 model.exe，但结束 driver 时
    # **不会**带走它们；反复重启就会攒下几十个孤儿 SITL（实测一次攒到 24 个），
    # 它们抢 UDP/端口、干扰新场景，表现为"飞机莫名其妙没动力学/速度飙到几十 m/s"。
    ("model.exe", "ArduPilot SITL（孤儿常客）"),
]


def pids_of():
    try:
        from core import preflight
        table = preflight._process_table()
    except Exception:
        table = None
    return table or {}


def main():
    ap = argparse.ArgumentParser(description="结束仿真相关进程")
    ap.add_argument("--dry-run", action="store_true", help="只打印，不结束")
    ap.add_argument("--check", action="store_true", help="只看状态")
    args = ap.parse_args()

    table = pids_of()
    if not table:
        print("读不到进程列表，改用 tasklist 直接结束")

    print("=" * 66)
    print("当前状态")
    print("=" * 66)
    total = 0
    for exe, desc in TARGETS:
        pids = table.get(exe.lower(), [])
        total += len(pids)
        print("  %-30s %-14s %s" % (exe, desc,
                                    ("运行中 PID=%s" % pids) if pids else "未运行"))

    # 端口告警
    try:
        sys.path.insert(0, THIS_DIR)
        import net_check
        rows = net_check._listening_tcp()
        if rows:
            on2000 = [r for r in rows if r[1] == 2000 and r[3] == 2]
            if len(on2000) > 1:
                print("")
                print("  !! 端口 2000 有 %d 个监听者 —— 开了多个仿真器实例！"
                      % len(on2000))
                print("     subject4 连到哪个是随机的，经常会连到空的那个而直接退出。")
    except Exception:
        pass

    if args.check:
        return 0
    if total == 0:
        print("\n没有需要结束的进程。")
        return 0

    print("")
    print("=" * 66)
    if args.dry_run:
        print("--dry-run：不实际结束")
        return 0

    print("开始结束 …")
    failed = []
    for exe, desc in TARGETS:
        pids = table.get(exe.lower(), [])
        for pid in pids:
            try:
                out = subprocess.run(["taskkill", "/F", "/T", "/PID", str(pid)],
                                     stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT)
                ok = out.returncode == 0
                print("  %-30s PID=%-6d %s" % (exe, pid, "已结束" if ok else "失败"))
                if not ok:
                    failed.append(exe)
            except Exception as e:
                print("  %-30s PID=%-6d 异常 %r" % (exe, pid, e))
                failed.append(exe)

    time.sleep(1.5)
    left = pids_of()
    remain = [e for e, _ in TARGETS if left.get(e.lower())]
    print("")
    if remain:
        print("!" * 66)
        print("!! 还有进程没结束：%s" % ", ".join(remain))
        print("!! 多半是权限问题 —— 请用【管理员身份】重开 PowerShell 再执行。")
        print("!" * 66)
        return 2

    print("已全部结束，现在是干净状态。")
    print("")
    print("按这个顺序重新开始（顺序不能错）：")
    print("   1) CarlaUE4")
    print("   2) Materials\\simu_driver_windows\\start.bat  → 选 2（旋翼）")
    print("   3) Materials\\practice\\practice1_Scene\\subject4.exe")
    print("   4) 确认 subject4 窗口里车和机都出来了，再跑算法")
    return 0


if __name__ == "__main__":
    sys.exit(main())
