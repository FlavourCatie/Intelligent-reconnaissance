# -*- coding: utf-8 -*-
"""
demo_midterm.py —— 中期答辩演示脚本（无人机纯模拟 + 实物坦克出击）
================================================================================

一句话：**无人机只在图上飞（不调真机），发现敌人后派真坦克去打。**

为什么这样设计
--------------
课程包这套 SDK 里，四旋翼的飞行动力学通道是空壳（`control_kinetic_*` /
`launch_kinetic` / `get_kinetic_data` 全部无效，且 M10 的物理关不掉、速度清不掉），
所以 3D 画面里的无人机无法稳定飞行。答辩阶段先按"无人机运动学模拟 + 实物无人车"
的组合演示，链路是真跑的、数据是真产生的。

演示时观众会看到
----------------
    1. 小地图锁定在任务区域（不再是整张 4.8 km 的全城图）
    2. 三架无人机图标沿规划航线飞出，留下航迹
    3. 进入 300 m 探测范围后，敌人的空心圈逐个变成实心红点
    4. 攒够目标数 → 三台**真实无人车**自动分配目标、A* 出发
    5. 无人车沿道路行驶、遇障重规划，到达或上报

用法
----
    # 第一次：撒一批近处敌人（之后必须手动重启 subject4.exe 才生效）
    python WorkPlace\\demo_midterm.py --spawn

    # 之后每次演示：
    python WorkPlace\\demo_midterm.py

    # 想更近 / 更多敌人：
    python WorkPlace\\demo_midterm.py --spawn --radius 280 --count 4

    # 只看要执行什么（不真跑）
    python WorkPlace\\demo_midterm.py --dry-run
"""

from __future__ import print_function

import argparse
import os
import subprocess
import sys

THIS_DIR = os.path.dirname(os.path.abspath(__file__))       # WorkPlace
PROJECT_DIR = os.path.dirname(THIS_DIR)
TOOLS_DIR = os.path.join(THIS_DIR, "tools")
MISSIONS_DIR = os.path.join(THIS_DIR, "missions")
SCENE = os.path.join(PROJECT_DIR, "Materials", "practice", "practice1_Scene")

PY = sys.executable
DRY = False

# 演示默认参数（都用最小的"能出效果"配置，答辩时省事）
DEFAULT_COUNT = 3                # 3 个敌人 ↔ 3 台车，一一对应
DEFAULT_RADIUS = 450.0           # 环带外半径
DEFAULT_RADIUS_MIN = 200.0       # 环带内半径（太近的敌人一开始就贴脸）
DEFAULT_MIN_SEP = 110.0          # 敌人之间最小间距
DEFAULT_NEAR = "-1875,-518"      # 撒点中心 = 无人机/无人车出生点
DEFAULT_ROI_MAX = 500.0          # 局部地图固定 ±500 m，正好罩住 450 m 环带


def banner(text):
    print("")
    print("=" * 74)
    print(text)
    print("=" * 74)
    sys.stdout.flush()


def call(cmd, what):
    print("     " + " ".join(cmd))
    sys.stdout.flush()
    if DRY:
        print("     （--dry-run：不实际执行）")
        return 0
    rc = subprocess.call(cmd, cwd=PROJECT_DIR)
    if rc != 0:
        raise SystemExit("\n[失败] %s（退出码 %d）" % (what, rc))
    return rc


def main():
    global DRY
    ap = argparse.ArgumentParser(
        description="中期答辩演示：无人机模拟侦查 + 实物无人车出击")
    ap.add_argument("--spawn", action="store_true",
                    help="先撒一批近距离敌人（之后要手动重启 subject4.exe 才生效）")
    ap.add_argument("--count", type=int, default=DEFAULT_COUNT,
                    help="敌人数（默认%d）" % DEFAULT_COUNT)
    ap.add_argument("--radius", type=float, default=DEFAULT_RADIUS,
                    help="撒点半径 m（默认%.0f；越大越分散）" % DEFAULT_RADIUS)
    ap.add_argument("--radius-min", type=float, default=DEFAULT_RADIUS_MIN,
                    help="环带内半径 m（默认%.0f）" % DEFAULT_RADIUS_MIN)
    ap.add_argument("--min-sep", type=float, default=DEFAULT_MIN_SEP,
                    help="敌人之间最小间距 m（默认%.0f）" % DEFAULT_MIN_SEP)
    ap.add_argument("--roi-max", type=float, default=DEFAULT_ROI_MAX,
                    help="局部地图半径 m（默认%.0f）" % DEFAULT_ROI_MAX)
    ap.add_argument("--sweep-half", type=float, default=None,
                    help="扫描区域半边长 m；默认自动 = 撒点半径 + 150，"
                         "保证敌人一定落在搜索区域内")
    ap.add_argument("--off-road", type=float, default=14.0,
                    help="敌人从路中心横向挪开多少米（默认14）。"
                         "路正中间会被无人车当成挡路障碍，越挪越好走")
    ap.add_argument("--dispatch-gap", type=float, default=4.0,
                    help="三台车错峰发车间隔秒（默认4），减少路口拥堵")
    ap.add_argument("--near", default=DEFAULT_NEAR,
                    help="撒点中心 x,y（默认 %s）" % DEFAULT_NEAR)
    ap.add_argument("--min-targets", type=int, default=3,
                    help="发现几个目标就派车（默认3）")
    ap.add_argument("--duration", type=float, default=600.0,
                    help="整轮总时长上限秒（默认600，给足运动时间）")
    ap.add_argument("--recon-timeout", type=float, default=200.0,
                    help="侦查超时秒（默认200）")
    ap.add_argument("--ugv-deadline", type=float, default=360.0,
                    help="派车后最长跑多少秒（默认360）")
    ap.add_argument("--uav-speed", type=float, default=None,
                    help="无人机模拟速度 m/s（默认用程序的15；演示可调快，如 25）")
    ap.add_argument("--dry-run", action="store_true", help="只打印命令")
    ap.add_argument("--extra", default=None, help="额外传给 full_mission 的参数")
    args = ap.parse_args()
    DRY = args.dry_run

    # ---------------- 1) 可选：撒敌人 ----------------
    if args.spawn:
        banner("第 1 步：生成敌人（%d 个，环带 %.0f~%.0f m，中心 %s）"
               % (args.count, args.radius_min, args.radius, args.near))
        call([PY, os.path.join(TOOLS_DIR, "spawn_targets.py"),
              "--scene", SCENE,
              "--count", str(args.count),
              "--near", args.near,
              "--radius", str(args.radius),
              "--radius-min", str(args.radius_min),
              "--min-sep", str(args.min_sep),
              "--off-road", str(args.off_road),
              "--types", "dynamic.prop.soldier"], "生成敌人")
        print("")
        print("!" * 74)
        print("!! 现在必须手动重启 subject4.exe，新敌人才能出现在场景里：")
        print("!!   1) 关掉 subject4.exe 的黑窗口")
        print("!!   2) 重新打开  %s\\subject4.exe" % SCENE)
        print("!!   3) 等它加载完（窗口里滚出 Actor 列表）")
        print("!!   4) 再执行： python WorkPlace\\demo_midterm.py")
        print("!" * 74)
        return 0

    # ---------------- 2) 跑演示 ----------------
    banner("演示开始：无人机模拟侦查 → 发现敌人 → 实物无人车出击")
    print("图例：三角形=无人机（模拟）  方形=无人车（实物）  实心红点=已发现敌人")
    print("      空心圈=未发现敌人  星标=目标点  虚线=目标连线")
    print("")

    cmd = [PY, os.path.join(MISSIONS_DIR, "full_mission.py"),
           "--scene", SCENE,
           "--uav-mode", "virtual",          # 关键：无人机只在图上飞
           "--view-roi", "auto",             # 小地图锁定任务区域
           "--uav-route", "sweep",           # 牛耕式覆盖扫描（敌人位置未知）
           "--sweep-region", "auto",         # 只扫出生点周边，演示更快

           "--sweep-partition", "rects",     # 三块矩形，一机一块
           "--sweep-radius", "300",
           "--min-targets", str(args.min_targets),
           "--duration", str(args.duration),
           "--recon-timeout", str(args.recon_timeout),
           "--ugv-deadline", str(args.ugv_deadline),
           "--view-roi-range", str(max(args.radius * 2.0, 1200.0)),
           "--dispatch-gap", str(args.dispatch_gap),
           "--view-interval", "1.5"]
    if args.uav_speed:
        cmd += ["--uav-speed", str(args.uav_speed)]

    # ---- 搜索区域：默认直接采用「局部地图任务区域」----
    # 敌人撒在出生点 ±radius 内，包围盒天然覆盖它们，不用再手工推半径。
    if args.sweep_half is not None:
        cmd += ["--sweep-half", str(args.sweep_half)]
        print("    扫描区域：手动指定半边长 %.0f m" % args.sweep_half)
    else:
        print("    扫描区域：自动采用「局部地图任务区域」（覆盖出生点与近处敌人）")
    if args.extra:
        cmd += args.extra.split()
    call(cmd, "演示")

    banner("演示结束")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n演示中断")
        sys.exit(130)
