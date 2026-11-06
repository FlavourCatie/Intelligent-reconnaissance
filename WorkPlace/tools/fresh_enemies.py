# -*- coding: utf-8 -*-
"""每次演示前重新随机撒敌人（3~5 个，离出生点 180~350 m）。

用法：
    python WorkPlace\\tools\\fresh_enemies.py            # 默认 practice1
    python WorkPlace\\tools\\fresh_enemies.py --count 4

注意：撒完必须**重启 CarlaUE4 + subject4.exe**，新敌人才会出现在场景里。
"""
from __future__ import print_function

import argparse
import os
import random
import subprocess
import sys

THIS = os.path.dirname(os.path.abspath(__file__))
WORKPLACE = os.path.dirname(THIS)
PROJECT = os.path.dirname(WORKPLACE)
DEFAULT_SCENE = r"C:\Users\llq-5\Desktop\IS\material\practice\practice1_Scene"   # 必须与 subject4.exe 所在目录一致
NEAR = "-1875,-518"          # 出生点（无人机/无人车都在这一带）


def main():
    ap = argparse.ArgumentParser(description="随机撒 3~5 个敌人")
    ap.add_argument("--scene", default=DEFAULT_SCENE)
    ap.add_argument("--count", type=int, default=0, help="0=随机 3~5")
    ap.add_argument("--radius-min", type=float, default=180.0,
                    help="离出生点最近距离（默认 180，别贴脸）")
    ap.add_argument("--radius", type=float, default=350.0)
    args = ap.parse_args()

    count = args.count or random.randint(3, 5)
    seed = random.randint(1, 10 ** 6)
    cmd = [sys.executable, os.path.join(THIS, "spawn_targets.py"),
           "--scene", args.scene,
           "--count", str(count),
           "--near", NEAR,
           "--radius", str(args.radius),
           "--radius-min", str(args.radius_min),
           "--off-road", "14",
           "--types", "dynamic.prop.soldier",
           "--seed", str(seed)]
    print("撒敌人: count=%d seed=%d" % (count, seed))
    rc = subprocess.call(cmd)
    if rc != 0:
        raise SystemExit("spawn_targets 失败（退出码 %d）" % rc)

    # 【重要】场景由 material 目录的 subject4.exe 启动，而 full_mission 的 --scene
    # 常传仓库内那份；两份 targets.json 不一致就会出现"地图上的敌人和场景里的敌人对不上"。
    # 这里顺手同步到仓库内那份（反过来也成立），保证两边永远一致。
    repo_scene = os.path.join(PROJECT, "Materials", "practice", "practice1_Scene")
    src = os.path.join(args.scene, "targets.json")
    dst = os.path.join(repo_scene, "targets.json")
    if (os.path.abspath(repo_scene) != os.path.abspath(args.scene)
            and os.path.isfile(src) and os.path.isdir(repo_scene)):
        try:
            import shutil
            shutil.copyfile(src, dst)
            print("已同步 targets.json 到: %s" % repo_scene)
        except Exception as e:
            print("同步 targets.json 失败（不影响本次演示）: %r" % (e,))
    print("")
    print("=" * 62)
    print("下一步（必须按顺序）：")
    print("  1) 关掉 CarlaUE4 和 subject4.exe（或直接跑一键脚本重启）")
    print("  2) 重新启动 CarlaUE4 → start.bat(选2) → subject4.exe")
    print("  3) 跑任务：")
    print("     python WorkPlace\\missions\\full_mission.py \\")
    print("       --scene %s \\" % args.scene)
    print("       --uav-mode kinetic --uav-route sweep \\")
    print("       --area \"-1790,-720,-1440,-320\" \\")
    print("       --sweep-radius 150 --min-targets %d \\" % count)
    print("       --duration 600 --ugv-deadline 400 --dispatch-gap 3")
    print("=" * 62)


if __name__ == "__main__":
    main()
