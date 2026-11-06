# -*- coding: utf-8 -*-
"""编队：领航-跟随 + 队形自适应（纵队 / 楔形），切换不停车。

设计要点：
  * 不重新规划每台车，只在领航车路径上取「后方 offset」作为僚车目标点；
  * 队形按可通行宽度自适应：宽路楔形、窄路纵队；
  * 换队形时偏移量按一阶滤波平滑过渡（τ≈0.6 s），视觉上是收拢而不是急打方向；
  * 所有偏移点都过一遍 project()（由调用方给的路网吸附），偏不进路里就退回纵队。

用法：
    f = Formation(kind="column", spacing=10.0, lateral=4.0, tau=0.6)
    targets = f.targets(leader_path, n_members=3, dt=0.2)
"""

from __future__ import print_function

import math


def _norm(dx, dy):
    L = math.hypot(dx, dy)
    return (dx / L, dy / L) if L > 1e-6 else (1.0, 0.0)


class Formation(object):
    """纵队 / 楔形 / 解散 三种状态，带平滑过渡。"""

    COLUMN = "column"
    WEDGE = "wedge"
    NONE = "none"

    def __init__(self, kind=COLUMN, spacing=10.0, lateral=4.0, tau=0.6):
        self.kind = kind
        self.spacing = float(spacing)
        self.lateral = float(lateral)
        self.tau = float(tau)
        self._cur = {}          # member_index -> (dx, dy) 当前偏移（相对领航车）
        self._target = {}

    # ---------------- 队形选择 ----------------
    @staticmethod
    def choose_kind(road_width, car_width=2.6, margin=1.6):
        """可通行宽度够两车并行且留余量 → 楔形，否则纵队。"""
        need = 2.0 * car_width + margin
        return Formation.WEDGE if road_width >= need else Formation.COLUMN

    def set_kind(self, kind):
        if kind in (self.COLUMN, self.WEDGE, self.NONE):
            self.kind = kind

    # ---------------- 偏移目标 ----------------
    def _target_offset(self, i):
        """第 i 台僚车（i=1..n）相对领航车的目标偏移（局部坐标：x 前、y 左）。"""
        if self.kind == self.NONE:
            return None
        if self.kind == self.COLUMN:
            return (-self.spacing * i, 0.0)
        # 楔形：左右交替，越靠后横向越大
        side = 1.0 if i % 2 == 1 else -1.0
        rank = (i + 1) // 2
        return (-self.spacing * i, side * self.lateral * rank)

    def _smooth(self, i, target, dt):
        cur = self._cur.get(i)
        if cur is None or target is None:
            self._cur[i] = target
            return target
        a = min(1.0, dt / max(1e-3, self.tau))
        nx = cur[0] + (target[0] - cur[0]) * a
        ny = cur[1] + (target[1] - cur[1]) * a
        self._cur[i] = (nx, ny)
        return (nx, ny)

    # ---------------- 生成僚车目标 ----------------
    def targets(self, leader_pos, leader_heading, n_members, dt=0.2,
                project=None, max_offset=60.0):
        """给 n_members 台僚车算出目标点（世界坐标）。

        leader_pos     : (x, y)
        leader_heading : 弧度（0 = +x）
        project(pt)    : 可选，路网吸附；返回 None 表示该偏移不可行 → 退回纵队
        """
        out = []
        hx, hy = math.cos(leader_heading), math.sin(leader_heading)
        for i in range(1, n_members + 1):
            tgt = self._target_offset(i)
            if tgt is None:
                out.append(None)
                continue
            # 偏移距离限幅，避免车被甩太远
            d = math.hypot(*tgt)
            if d > max_offset:
                tgt = (tgt[0] * max_offset / d, tgt[1] * max_offset / d)
            off = self._smooth(i, tgt, dt)
            px = leader_pos[0] + hx * off[0] - hy * off[1]
            py = leader_pos[1] + hy * off[0] + hx * off[1]
            if project is not None:
                proj = project(px, py)
                if proj is None and self.kind == self.WEDGE:
                    # 楔形偏到路外 → 该车改走纵队位置
                    off = self._smooth(i, (-self.spacing * i, 0.0), dt)
                    px = leader_pos[0] + hx * off[0] - hy * off[1]
                    py = leader_pos[1] + hy * off[0] + hx * off[1]
                    proj = project(px, py)
                if proj is not None:
                    px, py = proj
            out.append((px, py))
        return out

    def reset(self):
        self._cur = {}
