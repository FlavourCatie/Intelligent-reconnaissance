# -*- coding: utf-8 -*-
"""一键演示：把「敌人生成范围 / 无人机巡逻范围 / 小地图局部区域」统一成同一块地。

默认取**半张地图**（西半边），敌人只在这块地里随机生成，无人机只在这块地里牛耕扫描，
小地图也只显示这块地。

用法：
    python WorkPlace\\tools\\demo_run.py                 # 半图，7 个敌人
    python WorkPlace\\tools\\demo_run.py --count 5
    python WorkPlace\\tools\\demo_run.py --region full   # 整张图
    python WorkPlace\\tools\\demo_run.py --print-only    # 只打印命令，不撒点

流程：它只负责「撒点 + 打印命令」，撒完必须重启 CarlaUE4 + subject4.exe 再跑任务。
"""
from __future__ import print_function

import argparse
import json
import os
import subprocess
import sys

THIS = os.path.dirname(os.path.abspath(__file__))
WORKPLACE = os.path.dirname(THIS)
PROJECT = os.path.dirname(WORKPLACE)
SCENE = r"C:\Users\llq-5\Desktop\IS\material\practice\practice1_Scene"


def arena_of(scene):
    """从 game.json 读出场景边界，返回 (x0, y0, x1, y1)。"""
    path = os.path.join(scene, "game.json")
    with open(path, "r", encoding="utf-8-sig") as fp:
        data = json.load(fp)
    pts = data["SUBJECT_ARENA"]["arena"]
    xs = [p["x"] for p in pts]
    ys = [p["y"] for p in pts]
    return min(xs), min(ys), max(xs), max(ys)


def main():
    ap = argparse.ArgumentParser(description="统一区域的演示入口")
    ap.add_argument("--scene", default=SCENE)
    ap.add_argument("--count", type=int, default=7)
    ap.add_argument("--region", choices=("half", "full"), default="half")
    ap.add_argument("--side", choices=("west", "east"), default="west",
                    help="half 时选哪半边")
    ap.add_argument("--radius-min", type=float, default=150.0,
                    help="敌人离出生点至少多远（默认150）")
    ap.add_argument("--print-only", action="store_true")
    args = ap.parse_args()

    x0, y0, x1, y1 = arena_of(args.scene)
    if args.region == "half":
        mid = (x0 + x1) / 2.0
        if args.side == "west":
            x1 = mid
        else:
            x0 = mid
    area = "%.0f,%.0f,%.0f,%.0f" % (x0, y0, x1, y1)
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    half_w, half_h = (x1 - x0) / 2.0, (y1 - y0) / 2.0
    # 撒点半径取短边的一半再收一点，保证都落在区域里
    radius = min(half_w, half_h) * 0.85

    print("=" * 66)
    print("任务区域（敌人撒点 = 无人机巡查 = 小地图）: %s" % area)
    print("  中心 (%.0f, %.0f)  尺寸 %.0f x %.0f m"
          % (cx, cy, x1 - x0, y1 - y0))
    print("=" * 66)

    if not args.print_only:
        cmd = [sys.executable, os.path.join(THIS, "spawn_targets.py"),
               "--scene", args.scene,
               "--count", str(args.count),
               "--rect", area,
               "--off-road", "14",
               "--types", "dynamic.prop.soldier"]
        print("撒 %d 个敌人..." % args.count)
        if subprocess.call(cmd) != 0:
            raise SystemExit("撒点失败")

    print("")
    print("接下来（顺序不能错）：")
    print("  1) 重启 CarlaUE4  →  start.bat(选2)  →  subject4.exe")
    print("  2) 跑任务：")
    print("")
    print("     python WorkPlace\\missions\\full_mission.py \\")
    print("       --scene %s \\" % args.scene)
    print("       --uav-mode kinetic --uav-route sweep \\")
    print("       --area \"%s\" \\" % area)
    print("       --sweep-radius 300 --min-targets %d \\" % min(3, args.count))
    print("       --duration 1500 --ugv-deadline 1200 --dispatch-gap 5 \\")
    print("       --shot-at 600")
    print("")
    print("说明：无人机只在自己那块区域牛耕扫描（默认不再奔向敌人坐标），")
    print("      无人车按发现顺序分批出击、到点开火，打完一批自动派下一批。")


if __name__ == "__main__":
    main()
