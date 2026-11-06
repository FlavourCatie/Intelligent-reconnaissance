# -*- coding: utf-8 -*-
"""把无人车/无人机的出生点整体平移到任务区附近（保持相对位置不变）。

背景：practice1 的出生点在场地西侧（x≈-1860），而半图/战术层的任务区常在
x[-772,128]，每次测试要白飞 300 s 才进入工作面。

做法：所有车辆按同一个 (dx, dy) 平移，相对队形完全不变。
平移后务必检查无人车是否还压在道路上（不在路上会导致 A* 直接失败）。

用法：
    python WorkPlace\\tools\\shift_spawns.py --dry-run
    python WorkPlace\\tools\\shift_spawns.py --center "-1050,-650"    # 把队形质心移到该点
    python WorkPlace\\tools\\shift_spawns.py --delta "780,-80"        # 或直接给平移量
    python WorkPlace\\tools\\shift_spawns.py --restore               # 从 .bak 还原
"""

from __future__ import print_function

import argparse
import json
import os
import shutil
import sys

DEFAULT_SCENE = r"C:\Users\llq-5\Desktop\IS\material\practice\practice1_Scene"
REPO_SCENE = r"C:\Users\llq-5\Desktop\IS\Intelligent-reconnaissance\Materials\practice\practice1_Scene"


def load(path):
    with open(path, "r", encoding="utf-8-sig") as fp:
        return json.load(fp)


def save(path, data):
    with open(path, "w", encoding="utf-8") as fp:
        json.dump(data, fp, ensure_ascii=False, indent=4)
        fp.write("\n")


def shift(data, dx, dy):
    n = 0
    for group in data:
        for v in group.get("vehicles", []):
            pos = v.get("position")
            if not pos:
                continue
            parts = [float(x) for x in pos.split(",")]
            parts[0] += dx
            parts[1] += dy
            v["position"] = "%.1f, %.1f, %s" % (parts[0], parts[1], pos.split(",")[2].strip())
            n += 1
    return n


def centroid(data):
    xs, ys = [], []
    for group in data:
        for v in group.get("vehicles", []):
            parts = [float(x) for x in v["position"].split(",")]
            xs.append(parts[0])
            ys.append(parts[1])
    return (sum(xs) / len(xs), sum(ys) / len(ys))


def process(scene, args):
    path = os.path.join(scene, "vehicles.json")
    if not os.path.isfile(path):
        print("  跳过（没有 vehicles.json）: %s" % scene)
        return
    if args.restore:
        bak = path + ".bak"
        if os.path.isfile(bak):
            shutil.copyfile(bak, path)
            print("  已从备份还原: %s" % path)
        else:
            print("  没有备份可还原: %s" % path)
        return

    data = load(path)
    cx, cy = centroid(data)
    if args.delta:
        dx, dy = [float(v) for v in args.delta.split(",")[:2]]
    elif args.center:
        tx, ty = [float(v) for v in args.center.split(",")[:2]]
        dx, dy = tx - cx, ty - cy
    else:
        dx, dy = 0.0, 0.0
    print("  质心 (%.0f, %.0f)  平移 (%+.0f, %+.0f)" % (cx, cy, dx, dy))
    if abs(dx) < 1e-6 and abs(dy) < 1e-6:
        print("  平移量为 0，未改动")
        return
    if args.dry_run:
        print("  [dry-run] 不会写文件")
        return
    if not os.path.isfile(path + ".bak"):
        shutil.copyfile(path, path + ".bak")
    n = shift(data, dx, dy)
    save(path, data)
    print("  已平移 %d 台车，备份在 %s.bak" % (n, os.path.basename(path)))


def main():
    ap = argparse.ArgumentParser(description="整体平移出生点")
    ap.add_argument("--scene", default=None, help="场景目录（默认两份都处理）")
    ap.add_argument("--delta", default=None, help="平移量 dx,dy")
    ap.add_argument("--center", default=None, help="把队形质心移到 x,y")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--restore", action="store_true", help="从 .bak 还原")
    argv = sys.argv[1:]
    for opt in ("--center", "--delta"):
        if opt in argv:
            i = argv.index(opt)
            if i + 1 < len(argv) and argv[i + 1].startswith("-"):
                argv[i:i + 2] = ["%s=%s" % (opt, argv[i + 1])]
    args = ap.parse_args(argv)

    scenes = [args.scene] if args.scene else [DEFAULT_SCENE, REPO_SCENE]
    for s in scenes:
        if not os.path.isdir(s):
            continue
        print("== %s ==" % s)
        process(s, args)
    if not args.dry_run and not args.restore:
        print("")
        print("提醒：平移后重启 subject4.exe 生效；并确认无人车仍压在道路上（否则 A* 会失败）。")


if __name__ == "__main__":
    main()
