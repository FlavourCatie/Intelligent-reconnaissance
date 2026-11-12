# -*- coding: utf-8 -*-
"""清除/移动场景里的路障（沙袋、硬路障）。

背景：`rands.json` 里写的是**固定位置**的路障（实测 practice1 有两处
`dynamic.prop.hardbarricade`：(-1245,-870.9) 和 (-1245,-520)），它们正好横在
无人车必经的路上。无人车只能探测到很近的障碍，来不及绕行 → 倒车 → 和后面的车对撞。

用法：
    python WorkPlace\\tools\\clear_obstacles.py            # 直接清空（会备份 .bak）
    python WorkPlace\\tools\\clear_obstacles.py --dry-run   # 只看不改
    python WorkPlace\\tools\\clear_obstacles.py --move-to "-1245,-1350"  # 挪到别处
    python WorkPlace\\tools\\clear_obstacles.py --scene <另一份场景目录>

清完必须重启 subject4.exe（场景是启动时读 json 生成的）。
"""
from __future__ import print_function

import argparse
import json
import os
import shutil
import sys

DEFAULT_SCENE = r"C:\Users\llq-5\Desktop\IS\material\practice\practice1_Scene"
REPO_SCENE = r"C:\Users\llq-5\Desktop\IS\Intelligent-reconnaissance\Materials\practice\practice1_Scene"


def clear_one(scene, dry_run=False, move_to=None):
    path = os.path.join(scene, "rands.json")
    if not os.path.isfile(path):
        print("  跳过（没有 rands.json）: %s" % scene)
        return
    with open(path, "r", encoding="utf-8-sig") as fp:
        data = json.load(fp)
    pts = data.get("points") or []
    print("  %s: 现有 %d 个 %s" % (path, len(pts), data.get("type")))
    for p in pts:
        print("     - %s" % p.get("position"))
    if dry_run:
        return
    shutil.copyfile(path, path + ".bak")
    if move_to:
        ox, oy = [float(v) for v in move_to.split(",")[:2]]
        for i, p in enumerate(pts):
            p["position"] = "%.1f, %.1f, %.2f" % (ox + i * 40.0, oy, 0.60)
        data["points"] = pts
        data["count"] = len(pts)
        print("  已移动到 (%s 起，每 40 m 一个)" % move_to)
    else:
        data["points"] = []
        data["count"] = 0
        print("  已清空路障")
    with open(path, "w", encoding="utf-8") as fp:
        json.dump(data, fp, ensure_ascii=False, indent=4)
        fp.write("\n")


def main():
    ap = argparse.ArgumentParser(description="清除/移动场景路障")
    ap.add_argument("--scene", default=None, help="场景目录（默认两份都处理）")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--move-to", default=None, help="把路障挪到 x,y（例如 -1245,-1350）")
    args = ap.parse_args()

    scenes = [args.scene] if args.scene else [DEFAULT_SCENE, REPO_SCENE]
    for s in scenes:
        if not os.path.isdir(s):
            continue
        print("== %s ==" % s)
        clear_one(s, args.dry_run, args.move_to)
    if not args.dry_run:
        print("")
        print("完成。记得重启 subject4.exe（场景是启动时读 json 的）。")


if __name__ == "__main__":
    main()
