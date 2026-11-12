# -*- coding: utf-8 -*-
"""编队：领航-跟随 + 队形自适应（纵队 / 楔形），切换不停车。

两套接口：
  * ``targets()``：经典「相对领航车朝向」队形，适合空旷地；
  * ``follow_targets()``：沿领航车**实际路径**取僚车目标（城市道路用这个），
    拐弯时目标仍在路中间，且可设 `stop_before_goal` 让僚车在距目标 N 米处停下。

设计要点：
  * 不重新规划每台车，只在领航车路径上取「后方 offset」作为僚车目标点；
  * 队形按可通行宽度自适应：宽路楔形、窄路纵队；
  * 换队形时偏移量按一阶滤波平滑过渡（τ≈0.6 s），视觉上是收拢而不是急打方向；
  * 所有楔形偏移点都过一遍 project()（由调用方给的路网吸附），偏不进路里就退回纵队。

用法：
    f = Formation(kind="column", spacing=10.0, lateral=4.0, tau=0.6)
    targets = f.targets(leader_path, n_members=3, dt=0.2)
    follow  = f.follow_targets(leader_path, leader_pos, n_members=3,
                               stop_before_goal=200.0, project=snap)
"""

from __future__ import print_function

import math


def _norm(dx, dy):
    L = math.hypot(dx, dy)
    return (dx / L, dy / L) if L > 1e-6 else (1.0, 0.0)


# ---------------------------------------------------------------- 路径几何
def path_arc(path):
    """路径累计弧长，返回与 path 等长的列表（米）。"""
    cum = [0.0]
    for a, b in zip(path, path[1:]):
        cum.append(cum[-1] + math.hypot(float(b[0]) - float(a[0]),
                                        float(b[1]) - float(a[1])))
    return cum


def project_on_path(path, cum, p):
    """把点投影到折线上，返回 (弧长 s, 到折线距离)。O(n)，n 通常 < 2000。"""
    px, py = float(p[0]), float(p[1])
    best = (0.0, float("inf"))
    for i in range(len(path) - 1):
        ax, ay = float(path[i][0]), float(path[i][1])
        bx, by = float(path[i + 1][0]), float(path[i + 1][1])
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        if L2 <= 1e-12:
            continue
        tt = ((px - ax) * dx + (py - ay) * dy) / L2
        tt = max(0.0, min(1.0, tt))
        qx, qy = ax + tt * dx, ay + tt * dy
        d = math.hypot(px - qx, py - qy)
        if d < best[1]:
            best = (cum[i] + tt * math.sqrt(L2), d)
    if len(path) == 1:
        return 0.0, math.hypot(px - float(path[0][0]), py - float(path[0][1]))
    return best


def point_at(path, cum, s):
    """折线上弧长 s 处的点 (x, y)。"""
    total = cum[-1]
    s = max(0.0, min(float(s), total))
    lo, hi = 0, len(cum) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if cum[mid] <= s:
            lo = mid
        else:
            hi = mid
    seg = cum[hi] - cum[lo]
    if seg <= 1e-12:
        return (float(path[lo][0]), float(path[lo][1]))
    tt = (s - cum[lo]) / seg
    return (float(path[lo][0]) + tt * (float(path[hi][0]) - float(path[lo][0])),
            float(path[lo][1]) + tt * (float(path[hi][1]) - float(path[lo][1])))


def tangent_at(path, cum, s, eps=3.0):
    """折线上弧长 s 处的单位切向（指向路径前进方向）。"""
    a = point_at(path, cum, s - eps)
    b = point_at(path, cum, s + eps)
    if math.hypot(b[0] - a[0], b[1] - a[1]) < 1e-9:
        j = min(len(path) - 1, 1)
        a = (float(path[0][0]), float(path[0][1]))
        b = (float(path[j][0]), float(path[j][1]))
    return _norm(b[0] - a[0], b[1] - a[1])


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

        这是「相对领航车朝向」的经典队形，适合空旷地；城市道路请用
        follow_targets()——它沿领航车实际路径取点，拐弯时不会把僚车甩到路外。

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

    # ---------------- 沿路径的编队跟随 ----------------
    def follow_targets(self, path, leader_pos, n_members, stop_before_goal=0.0,
                       project=None, dt=0.2):
        """沿领航车**实际路径**给僚车取目标点（城市道路的正确做法）。

        僚车目标 = 领航车当前位置沿路径后退 spacing*i 米的位置；
        所有僚车都不超过 `路径总长 - stop_before_goal`（集结点），
        即「跟到距目标 N 米就停」，不会跟着领航车冲进交火区。

        返回 [ (x, y) 或 None, ... ]，None 表示领航车还没走出足够距离，原地等。
        """
        n_members = int(n_members)
        if self.kind == self.NONE or n_members <= 0:
            return [None] * max(0, n_members)
        if not path or len(path) < 2:
            return [None] * n_members
        cum = path_arc(path)
        total = cum[-1]
        if total < 1.0:
            return [None] * n_members
        s_leader, _d = project_on_path(path, cum, leader_pos)
        s_cap = max(0.0, total - float(stop_before_goal))
        if s_cap <= 0.5:
            # 领航车的目标本身就在 rally 距离内：僚车没有可跟随的区段，原地待命
            return [None] * n_members
        out = []
        for i in range(1, n_members + 1):
            # 两个上限取小：不能超过领航车后方 spacing*i，也不能越过集结点；
            # 集结点之后仍要保持车间距（否则所有僚车会叠在集结点上）。
            s = min(s_leader - self.spacing * i,
                    s_cap - self.spacing * (i - 1))
            if s <= 0.5:
                # 领航车还没走出编队长度：等下一拍，避免僚车往身后倒车
                out.append(None)
                continue
            px, py = point_at(path, cum, s)
            if self.kind == self.WEDGE:
                tx, ty = tangent_at(path, cum, s)
                side = 1.0 if i % 2 == 1 else -1.0
                rank = (i + 1) // 2
                lat = self._smooth(i, side * self.lateral * rank, dt)
                px -= ty * lat
                py += tx * lat
                if project is not None:
                    proj = project(px, py)
                    if proj is None:
                        # 偏出路面 → 回到路径中心线（纵队位置）
                        px, py = point_at(path, cum, s)
                        self._cur[i] = (0.0, 0.0)
                    else:
                        px, py = proj
            out.append((px, py))
        return out

    def reset(self):
        self._cur = {}
