# -*- coding: utf-8 -*-
"""覆盖栅格与「前沿点」：告诉空闲车该往哪儿靠。

前沿点 = 已扫区域与未扫区域的交界，但**排除任务区边界附近**——
否则车会往地图边缘跑（那里没有路，也没有敌人）。

用法：
    cov = CoverageGrid(zone, cell=50.0, edge_clearance=100.0)
    cov.mark(x, y, r=300)                       # 无人机每帧上报自己的位置
    pts = cov.frontier_points(road_points, max_pts=3)   # 路网点里挑前沿
"""

from __future__ import print_function

import math


class CoverageGrid(object):
    """按固定网格记账的覆盖图（只记 0/1，不重复叠加）。"""

    def __init__(self, zone, cell=50.0, edge_clearance=100.0):
        self.zone = zone
        self.cell = float(cell)
        self.edge_clearance = float(edge_clearance)
        self.covered = set()

    # ---------------- 记账 ----------------
    def _key(self, x, y):
        return (int(math.floor(x / self.cell)), int(math.floor(y / self.cell)))

    def mark(self, x, y, r):
        """把以 (x,y) 为圆心、r 为半径的格子标为已覆盖。"""
        r = float(r)
        n = int(math.ceil(r / self.cell))
        kx, ky = self._key(x, y)
        cells = set()
        for dx in range(-n, n + 1):
            for dy in range(-n, n + 1):
                cx = (kx + dx + 0.5) * self.cell
                cy = (ky + dy + 0.5) * self.cell
                if (cx - x) ** 2 + (cy - y) ** 2 <= r * r:
                    cells.add((kx + dx, ky + dy))
        self.covered |= cells
        return len(cells)

    def is_covered(self, x, y):
        return self._key(x, y) in self.covered

    # ---------------- 前沿 ----------------
    def inner_ok(self, x, y):
        """是否离任务区边界足够远（默认 100 m）。"""
        return self.zone.contains(x, y, margin=self.edge_clearance)

    def on_frontier(self, x, y, band=1):
        """已覆盖格子的邻域里存在未覆盖格子 → 认为该点在交界带上。"""
        kx, ky = self._key(x, y)
        for dx in range(-band, band + 1):
            for dy in range(-band, band + 1):
                k = (kx + dx, ky + dy)
                if k not in self.covered:
                    return True
        return False

    def frontier_points(self, candidates, max_pts=3, min_gap=120.0):
        """从候选路网点里挑前沿点。

        candidates: [(x, y), ...] —— 由调用方给（路网采样点/节点）
        返回 [(x, y), ...]，按「先前沿、再离区域边界远」排序。
        """
        picked = []
        for (x, y) in candidates:
            if not self.inner_ok(x, y):
                continue                 # 不许贴着任务区边界
            if not self.on_frontier(x, y):
                continue                 # 不在交界带上没意义
            if any((x - px) ** 2 + (y - py) ** 2 < min_gap ** 2 for px, py in picked):
                continue
            picked.append((float(x), float(y)))
            if len(picked) >= max_pts:
                break
        return picked

    def stats(self):
        return {"covered_cells": len(self.covered), "cell": self.cell,
                "area_m2": len(self.covered) * self.cell * self.cell}
