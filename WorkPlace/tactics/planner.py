# -*- coding: utf-8 -*-
"""战术层总装：把「车辆状态 + 敌人 + 覆盖栅格」变成一份 strike_plan.json。

这一层不控车、不连仿真，只做：
    1. 把运行态整理成 assign.solve() 需要的输入；
    2. 调用 allocate 得到「谁去哪个阵位」；
    3. 给空闲车挑前沿点；
    4. 组一张编队描述；
    5. 原子写计划文件。

路口的「合围阵位」（m 个分支）由 P2 的 slots.py 提供；在它落地前，
junction 目标先用单阵位占位（fire_rule 仍然是 single），这样整条链路可以先跑通。

用法：
    t = Tactics(zone, plan_path, eta_fn=lambda p, q: dist(p,q)/8.0)
    plan = t.update(now=12.3,
                    vehicles=[{"vid": "UGV1", "pos": (0, 0), "commit": "L0"}],
                    enemies=[("E1", (100, 50), "junction", 4, 0.0)],
                    road_points=[...])
"""

from __future__ import print_function

import math

from . import assign as assign_mod
from .coverage import CoverageGrid
from .plan_io import PlanStore, make_plan
from .zones import Zone


class Tactics(object):
    def __init__(self, zone, plan_path, eta_fn=None, cell=50.0,
                 edge_clearance=100.0, formation_spacing=10.0):
        if not isinstance(zone, Zone):
            raise TypeError("zone 必须是 tactics.zones.Zone")
        self.zone = zone
        self.store = PlanStore(plan_path)
        self.eta_fn = eta_fn or (lambda p, q: math.hypot(p[0] - q[0], p[1] - q[1]) / 8.0)
        self.cov = CoverageGrid(zone, cell=cell, edge_clearance=edge_clearance)
        self.spacing = formation_spacing
        self.prev = None
        self._seen = {}

    # ---------------- 覆盖 ----------------
    def mark_coverage(self, x, y, r):
        self.cov.mark(x, y, r)

    # ---------------- 主入口 ----------------
    def update(self, now, vehicles, enemies, road_points=None, force=False):
        """enemies: [(tid, (x,y), kind, junction_degree, t_first_seen), ...]"""
        if not vehicles:
            return None

        targets = []
        for item in enemies:
            tid, pos, kind, deg, t_first = item[:5]
            given = item[5] if len(item) > 5 else None
            self._seen.setdefault(tid, t_first)
            wait = max(0.0, float(now) - float(self._seen[tid]))
            if given:
                slots = [{"sid": s.get("sid", "S%d" % i), "pos": tuple(s["pos"])}
                         for i, s in enumerate(given)]
            else:
                slots = [{"sid": "center", "pos": (float(pos[0]), float(pos[1]))}]
            targets.append({
                "tid": tid, "kind": kind, "junction_degree": int(deg or 0),
                "wait": wait, "slots": slots,
            })

        plan = assign_mod.solve(vehicles, targets, self.eta_fn,
                                prev=self.prev, improvement=0.20, max_preempt=2)
        if plan is None and not force:
            return None

        if plan is None:
            plan = {"groups": [], "commitment": {}}

        # 空闲车 → 前沿点
        idle = []
        busy = set()
        for g in plan.get("groups", []):
            for s in g.get("slots", []):
                busy.add(s["vid"])
        free = [v for v in vehicles if v["vid"] not in busy]
        if free and road_points:
            pts = self.cov.frontier_points(road_points, max_pts=len(free))
            for v, pt in zip(free, pts):
                idle.append({"vid": v["vid"], "frontier": [pt[0], pt[1]],
                             "reason": "uncovered_frontier"})
        plan["idle"] = idle

        # 编队描述（谁领航、谁跟随）
        line = {"leader": None, "members": [], "formation": "none", "anchor": None}
        if len(free) >= 2:
            line.update({"leader": free[0]["vid"],
                         "members": [v["vid"] for v in free[1:]],
                         "formation": "column",
                         "anchor": list(free[0]["pos"])})
        plan["line"] = line
        plan["t_plan"] = float(now)

        written = self.store.write(plan)
        self.prev = written
        return written

    # ---------------- 查询 ----------------
    def previous(self):
        return self.prev
