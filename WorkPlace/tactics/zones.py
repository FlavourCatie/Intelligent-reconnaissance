# -*- coding: utf-8 -*-
"""任务区域（Zone）定义：战术层唯一的区域来源。

设计约束（针对此前「测试图/半图逻辑混在一起导致敌人撒错位置」的事故）：
  1. 本模块**不提供任何默认值**，也不提供 auto 兜底 —— 缺 --zone 就直接报错；
  2. 撒点、巡逻、小地图都必须用同一个 Zone 对象；
  3. 撒点结果必须落在 Zone 内，越界抛异常，绝不静默修正。

用法：
    from tactics.zones import Zone
    z = Zone.parse("-1250,-950,-350,-250", name="junction_core")
    z.contains(x, y)          # -> bool
    z.assert_inside(point)    # 越界 raise ZoneError
"""

from __future__ import print_function


class ZoneError(ValueError):
    """区域参数缺失或非法。"""


class Zone(object):
    """矩形任务区（世界坐标，米）。"""

    __slots__ = ("x0", "y0", "x1", "y1", "name")

    def __init__(self, x0, y0, x1, y1, name="zone"):
        x0, y0, x1, y1 = float(x0), float(y0), float(x1), float(y1)
        if x1 <= x0 or y1 <= y0:
            raise ZoneError("区域必须满足 x1>x0 且 y1>y0，收到 (%g,%g,%g,%g)"
                            % (x0, y0, x1, y1))
        self.x0, self.y0, self.x1, self.y1 = x0, y0, x1, y1
        self.name = name

    # ---------------- 构造 ----------------
    @classmethod
    def parse(cls, text, name="zone"):
        """解析 "x0,y0,x1,y1"；text 为空则报错（不猜默认值）。"""
        if not text:
            raise ZoneError("缺少 --zone。必须显式给出 \"x0,y0,x1,y1\"，本模块不提供默认区域。")
        parts = [p for p in str(text).replace("，", ",").split(",") if p.strip()]
        if len(parts) != 4:
            raise ZoneError("--zone 需要 4 个数字 x0,y0,x1,y1，收到：%r" % (text,))
        try:
            v = [float(p) for p in parts]
        except ValueError:
            raise ZoneError("--zone 含非数字：%r" % (text,))
        return cls(v[0], v[1], v[2], v[3], name=name)

    # ---------------- 查询 ----------------
    def contains(self, x, y, margin=0.0):
        return (self.x0 + margin <= x <= self.x1 - margin
                and self.y0 + margin <= y <= self.y1 - margin)

    def center(self):
        return ((self.x0 + self.x1) / 2.0, (self.y0 + self.y1) / 2.0)

    def size(self):
        return (self.x1 - self.x0, self.y1 - self.y0)

    def rect(self):
        return (self.x0, self.y0, self.x1, self.y1)

    def to_list(self):
        return [self.x0, self.y0, self.x1, self.y1]

    def as_arg(self):
        return "%.1f,%.1f,%.1f,%.1f" % (self.x0, self.y0, self.x1, self.y1)

    # ---------------- 校验 ----------------
    def assert_inside(self, point, what="point", margin=0.0):
        x, y = float(point[0]), float(point[1])
        if not self.contains(x, y, margin):
            raise ZoneError("%s (%.1f, %.1f) 超出区域 %s [%s]"
                            % (what, x, y, self.name, self.as_arg()))
        return True

    def assert_all_inside(self, points, what="target"):
        for i, p in enumerate(points):
            self.assert_inside(p, what="%s#%d" % (what, i))
        return True

    def clamp(self, x, y):
        return (min(max(x, self.x0), self.x1), min(max(y, self.y0), self.y1))

    def __repr__(self):
        return "Zone(%s, %.0f,%.0f,%.0f,%.0f)" % ((self.name,) + self.rect())


def zone_from_args(args, name="zone"):
    """从 argparse 结果里取 --zone；缺失即抛 ZoneError（不会悄悄用默认值）。"""
    text = getattr(args, "zone", None)
    return Zone.parse(text, name=name)
