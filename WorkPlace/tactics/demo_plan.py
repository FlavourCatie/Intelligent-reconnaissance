# -*- coding: utf-8 -*-
"""离线演示：给定一组车辆/敌人/覆盖情况，生成 strike_plan.json 并打印。

不连仿真，用来验证「战术层总装」这条链路：
    车辆 + 敌人 + 覆盖 → assign → 计划文件 → 可读的结果表

跑法：
    python WorkPlace\\tactics\\demo_plan.py
    python WorkPlace\\tactics\\demo_plan.py --plan WorkPlace\\config\\strike_plan.json
"""

from __future__ import print_function

import argparse
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
WORKPLACE = os.path.dirname(HERE)
for p in (HERE, WORKPLACE):
    if p not in sys.path:
        sys.path.insert(0, p)

from tactics.zones import Zone          # noqa: E402
from tactics.planner import Tactics     # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="战术层离线演示")
    ap.add_argument("--plan", default=os.path.join(WORKPLACE, "config", "strike_plan.json"))
    ap.add_argument("--zone", default="-772,-1006,128,-306")
    args = ap.parse_args()

    zone = Zone.parse(args.zone, name="junction_core")
    out = args.plan
    tmp = tempfile.mkdtemp(prefix="tactics_demo_")
    path = os.path.join(tmp, "strike_plan.json")

    t = Tactics(zone, path)
    # 模拟：无人机扫过西侧（覆盖栅格），路上有三个敌人
    for x in range(-750, -300, 60):
        for y in range(-950, -350, 60):
            t.mark_coverage(x, y, r=250)

    vehicles = [
        {"vid": "UGV1", "pos": (-600.0, -650.0), "commit": "L0"},
        {"vid": "UGV2", "pos": (-400.0, -700.0), "commit": "L0"},
        {"vid": "UGV3", "pos": (50.0, -500.0), "commit": "L0"},
    ]
    enemies = [
        ("E1", (-200.0, -600.0), "junction", 4, 10.0),
        ("E2", (60.0, -400.0), "road", 2, 12.0),
    ]
    road = [(x, y) for x in range(-760, 120, 40) for y in range(-1000, -320, 40)]

    plan = t.update(now=40.0, vehicles=vehicles, enemies=enemies, road_points=road)
    if plan is None:
        print("没有改进，未下发新计划（防抖生效）")
        return 0

    print("计划已写入：%s" % path)
    import json
    with open(path, "r", encoding="utf-8") as fp:
        data = json.load(fp)
    print("plan_id=%s  t=%s" % (data["plan_id"], data["t_plan"]))
    for g in data["groups"]:
        print("  目标 %s (%s, 分支数=%s)  t_star=%ss" %
              (g["target_id"], g["kind"], g["junction_degree"], g.get("t_star")))
        for s in g["slots"]:
            print("     %s -> %s  eta=%ss" % (s["vid"], s["branch"], s.get("eta")))
    for i in data["idle"]:
        print("  空闲 %s -> 前沿 %s" % (i["vid"], i["frontier"]))
    line = data.get("line") or {}
    if line.get("leader"):
        print("  编队：%s 领航，成员 %s，队形 %s"
              % (line["leader"], line["members"], line["formation"]))
    if os.path.isdir(os.path.dirname(out)):
        try:
            import shutil
            shutil.copyfile(path, out)
            print("已另存到 %s" % out)
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
