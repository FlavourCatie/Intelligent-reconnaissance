# -*- coding: utf-8 -*-
"""编队巡航控制器：把「领航车实际路径」变成「僚车这一拍该去哪儿」。

只做纯计算，不碰 SDK / 仿真，便于离线单测。主循环每帧把当前状态喂进来：

    convoy = ConvoyController(kind="column", spacing=12.0, rally=200.0)
    targets = convoy.update(
        now=12.3,
        leader={"name": "UGV1", "pos": (x, y), "path": [(x, y), ...],
                "kind": "strike"},              # strike=去打敌人, staging=前沿待机
        members=[{"name": "UGV2", "pos": (x, y)}, ...])

    # -> {"UGV2": (tx, ty), ...}   空 dict 表示这一拍不用改目标

设计要点
--------
* **沿路径**而不是沿朝向：领航车拐弯后，僚车目标仍在路中间，不会偏进绿化带；
* **集结点**：领航车去打敌人时，僚车最多跟到「距目标 rally 米」就停（默认 200 m），
  新目标下达时直接 split，不需要回队集合；
* **滞回**：目标移动不足 move_tol（默认 6 m）或距上次下发不足 min_interval，
  不下发新目标，避免每帧重规划造成抽搐；
* 队形可 `auto`：由调用方给领航车处路宽，宽度够就走楔形，否则纵队。
"""

from __future__ import print_function

import math

from .formation import Formation, project_on_path, path_arc


class ConvoyController(object):
    COLUMN = Formation.COLUMN
    WEDGE = Formation.WEDGE
    NONE = Formation.NONE

    def __init__(self, kind=Formation.COLUMN, spacing=12.0, lateral=4.0,
                 tau=0.6, rally=200.0, move_tol=6.0, min_interval=1.0):
        kind = kind or Formation.COLUMN
        self.requested_kind = kind
        self.formation = Formation(kind=(Formation.COLUMN if kind == "auto" else kind),
                                   spacing=spacing, lateral=lateral, tau=tau)
        self.rally = float(rally)
        self.move_tol = float(move_tol)
        self.min_interval = float(min_interval)
        self._last = {}          # name -> (x, y) 上次下发的跟随点
        self._last_t = {}        # name -> 上次下发时刻
        self._leader = None
        self.kind = self.formation.kind
        self.order = []          # 上一拍的成员顺序（离领航车近的在前）
        self.reason = ""         # 上一拍为什么没下发（调试用）

    # ---------------- 状态维护 ----------------
    def set_kind(self, kind):
        """切换队形；auto 需要调用方在 update() 里给 road_width。"""
        self.requested_kind = kind or Formation.COLUMN
        if self.requested_kind != "auto":
            self.formation.set_kind(self.requested_kind)

    def reset(self, name=None):
        if name is None:
            self._last.clear()
            self._last_t.clear()
            self.formation.reset()
            self._leader = None
        else:
            self._last.pop(name, None)
            self._last_t.pop(name, None)

    def describe(self):
        if not self.order:
            return "%s（无僚车）" % self.kind
        return "%s 领航，%s 跟随，间距 %.0f m，队形 %s" % (
            self._leader or "?", "/".join(self.order),
            self.formation.spacing, self.kind)

    # ---------------- 主入口 ----------------
    def update(self, now, leader, members, project=None, road_width=None):
        """返回 {成员名: (x, y)}；空 dict 表示保持现状。

        leader  : {"name","pos","path","kind"}；kind ∈ {"strike","staging"}
        members : [{"name","pos"}, ...]；调用方保证这里只放「空闲车」
        project : 可选 project(x, y) -> (x, y) 或 None，楔形偏出路面时回退用
        road_width : auto 队形用，领航车处可通行宽度（米）
        """
        self.reason = ""
        if not leader or not members:
            self.reason = "no_leader_or_members"
            return {}
        name = leader.get("name")
        path = leader.get("path") or []
        if len(path) < 2:
            self.reason = "leader_no_path"
            return {}
        if self.requested_kind == "auto":
            if road_width is not None:
                self.formation.set_kind(Formation.choose_kind(float(road_width)))
            else:
                self.formation.set_kind(Formation.COLUMN)
        self.kind = self.formation.kind

        if self._leader != name:
            # 换领航车：相对偏移全部清零，避免沿用上一任的横偏
            self.reset()
            self._leader = name
            self.reason = "leader_changed"

        # 成员排序：沿领航车路径走得最远的排前面（编号越小，越靠近领航车）
        cum = path_arc(path)
        scored = []
        for m in members:
            s, d = project_on_path(path, cum, m.get("pos") or (0.0, 0.0))
            scored.append((s, d, m))
        scored.sort(key=lambda it: (-it[0], it[1]))
        self.order = [m["name"] for (_s, _d, m) in scored]

        stop = self.rally if (leader.get("kind") == "strike") else 0.0
        pts = self.formation.follow_targets(
            path, leader.get("pos") or path[0], len(scored),
            stop_before_goal=stop, project=project)
        out = {}
        for (_s, _d, m), pt in zip(scored, pts):
            if pt is None:
                continue
            mn = m["name"]
            last = self._last.get(mn)
            if last is not None:
                if (now - self._last_t.get(mn, -1e9)) < self.min_interval:
                    continue
                if math.hypot(pt[0] - last[0], pt[1] - last[1]) < self.move_tol:
                    continue
            out[mn] = (float(pt[0]), float(pt[1]))
            self._last[mn] = (float(pt[0]), float(pt[1]))
            self._last_t[mn] = float(now)
        if not out and not self.reason:
            self.reason = "hold"
        return out
