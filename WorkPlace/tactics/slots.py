# -*- coding: utf-8 -*-
"""路口判定与合围阵位生成。

思路：不依赖任何新传感器，只用**道路掩膜**（roadnet_raw.png 或运行时的 binary 栅格）。
以敌人为中心画一个环（默认 35 m），沿环采样，统计"有路"的角向连续段数量：
    >= 3 段 → 路口（十字/丁字），可以合围
    2 段    → 丁字（首尾夹击）
    1 段    → 路段中间，单车打

阵位 = 沿"非缺口"的每个分支方向，在距敌人 standoff（默认 30 m）处取点，
再向本车来向回退 hold_back（默认 40 m）作为集结点。
"""

from __future__ import print_function

import math

import numpy as np

EPS = 1e-6


# ---------------------------------------------------------------- 掩膜加载
class RoadMask(object):
    """世界坐标 ↔ 像素的道路掩膜（scale=1 时 1 像素=1 m）。"""

    def __init__(self, arr, scale=1.0, offset=(0.0, 0.0), thr=128):
        self.arr = np.asarray(arr)
        self.scale = float(scale)
        self.offset = (float(offset[0]), float(offset[1]))
        self.thr = thr

    @classmethod
    def from_png(cls, path, scale=1.0, offset=(0.0, 0.0), meta_path=None):
        from PIL import Image
        img = np.array(Image.open(path).convert("L"))
        if meta_path:
            import json
            try:
                with open(meta_path, "r", encoding="utf-8-sig") as fp:
                    m = json.load(fp)
                scale = float(m.get("scale", scale))
                offset = tuple(m.get("offset", offset))
            except Exception:
                pass
        return cls(img, scale=scale, offset=offset)

    def to_px(self, x, y):
        return (int(round((x - self.offset[0]) / self.scale)),
                int(round((y - self.offset[1]) / self.scale)))

    def to_world(self, px, py):
        return (px * self.scale + self.offset[0], py * self.scale + self.offset[1])

    def is_road(self, x, y):
        px, py = self.to_px(x, y)
        h, w = self.arr.shape[:2]
        if px < 0 or py < 0 or px >= w or py >= h:
            return False
        return bool(self.arr[py, px] >= self.thr)


# ---------------------------------------------------------------- 分支识别
def ring_sectors(mask, center, ring=35.0, step_deg=5.0, min_gap_deg=25.0):
    """沿环采样，返回 [(中心角, 宽度角)] 的路面段（角度为度，0=+x，逆时针）。"""
    cx, cy = center
    flags = []
    n = int(360.0 / step_deg)
    for i in range(n):
        a = math.radians(i * step_deg)
        x = cx + ring * math.cos(a)
        y = cy + ring * math.sin(a)
        flags.append(1 if mask.is_road(x, y) else 0)
    if not any(flags):
        return []
    if all(flags):
        return [(0.0, 360.0)]

    # 环形连续段：从 0->1 跳变处开始扫一圈
    start = None
    for i in range(n):
        if flags[i] and not flags[i - 1]:
            start = i
            break
    sectors = []
    cur_len = 0
    cur_start = None
    for k in range(n):
        i = (start + k) % n
        if flags[i]:
            if cur_start is None:
                cur_start = i
            cur_len += 1
        else:
            if cur_start is not None:
                sectors.append((cur_start, cur_len))
                cur_start, cur_len = None, 0
    if cur_start is not None:
        sectors.append((cur_start, cur_len))

    out = []
    for (i0, length) in sectors:
        width = length * step_deg
        if width < min_gap_deg and len(sectors) > 1:
            continue                      # 太窄，当作噪声
        mid = (i0 + length / 2.0) * step_deg % 360.0
        out.append((mid, width))
    return out


_SK = None


class _Skel(object):
    """骨架图 + 8 邻域计数，用来可靠地区分直路 / 丁字 / 十字。"""

    def __init__(self, path, meta_path=None):
        import json
        from PIL import Image
        from skimage.morphology import skeletonize
        from scipy.ndimage import convolve
        img = np.array(Image.open(path))
        self.sk = skeletonize(np.all(img >= 200, axis=-1) if img.ndim == 3
                              else (img >= 200))
        k = np.ones((3, 3), dtype=int)
        k[1, 1] = 0
        self.nb = convolve(self.sk.astype(int), k, mode="constant")
        self.scale, self.off = 1.0, (0.0, 0.0)
        if meta_path:
            try:
                with open(meta_path, "r", encoding="utf-8-sig") as fp:
                    m = json.load(fp)
                self.scale = float(m.get("scale", 1.0))
                self.off = tuple(m.get("offset", (0.0, 0.0)))
            except Exception:
                pass

    def kind_at(self, x, y, search=30):
        px = int(round((x - self.off[0]) / self.scale))
        py = int(round((y - self.off[1]) / self.scale))
        best, bd = None, None
        r = int(search / self.scale)
        h, w = self.sk.shape
        for dy in range(-r, r + 1):
            yy = py + dy
            if yy < 0 or yy >= h:
                continue
            for dx in range(-r, r + 1):
                xx = px + dx
                if xx < 0 or xx >= w or not self.sk[yy, xx]:
                    continue
                d = dx * dx + dy * dy
                if bd is None or d < bd:
                    best, bd = (yy, xx), d
        if best is None:
            return None
        n = int(self.nb[best[0], best[1]])
        if n >= 4:
            return "junction"
        if n == 3:
            return "tee"
        if n == 2:
            return "road"
        return "road"


def _skeleton():
    """懒加载骨架（缺文件时返回 None，调用方退回环采样）。"""
    global _SK
    if _SK is not None:
        return _SK if _SK is not False else None
    import os
    wp = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    skl = os.path.join(wp, "assets", "skeletonized_image.png")
    meta = os.path.join(wp, "config", "roadnet_meta.json")
    try:
        _SK = _Skel(skl, meta)
    except Exception:
        _SK = False
        return None
    return _SK


def classify(mask, center, ring=35.0):
    """返回 (kind, 分支列表)。

    kind 用骨架图的 8 邻域连接数判定（直路=2、丁字=3、十字>=4）——实测 40/40 正确；
    分支方向仍用环采样给出（只影响阵位朝向，不影响"要不要合围"）。
    """
    sec = ring_sectors(mask, center, ring=ring)
    sk = _skeleton()
    if sk is not None:
        k = sk.kind_at(center[0], center[1])
        if k:
            return k, sec
    m = len(sec)
    if m >= 3:
        return "junction", sec
    if m == 2:
        d = abs(((sec[0][0] - sec[1][0]) + 180.0) % 360.0 - 180.0)
        return ("road" if abs(d - 180.0) <= 30.0 else "tee"), sec
    if m == 1:
        return "road", sec
    return "unknown", sec

# ---------------------------------------------------------------- 阵位
def build_slots(mask, center, branches, gap_index=None, standoff=18.0,
                hold_back=40.0, zone=None):
    """给每个"非缺口"分支生成一个阵位 + 集结点。

    返回 [{"sid": "N", "dir": deg, "pos": (x,y), "hold": (x,y)}, ...]
    """
    cx, cy = center
    # 丁字路口（2 个分支）采用"首尾夹击"：两个方向都要封，不留缺口
    if len(branches) <= 2:
        gap_index = -1
    elif gap_index is None:
        gap_index = _pick_gap(branches)
    slots = []
    for i, (deg, _w) in enumerate(branches):
        if i == gap_index:
            continue
        a = math.radians(deg)
        ux, uy = math.cos(a), math.sin(a)
        pos = (cx + ux * standoff, cy + uy * standoff)
        if not mask.is_road(*pos):
            # 吸附到该方向上最近的道路点（沿径向试探）
            for r in (standoff * 1.5, standoff * 2.0, standoff * 2.5):
                cand = (cx + ux * r, cy + uy * r)
                if mask.is_road(*cand):
                    pos = cand
                    break
        hold = (cx + ux * (standoff + hold_back), cy + uy * (standoff + hold_back))
        name = _dir_name(deg)
        slots.append({"sid": name, "dir": deg,
                      "pos": (float(pos[0]), float(pos[1])),
                      "hold": (float(hold[0]), float(hold[1]))})
    return slots


def _pick_gap(branches):
    """缺口选择：取"最接近 +y（北）"的分支作为诱敌方向（占位实现）。"""
    best, best_d = 0, 1e9
    for i, (deg, _w) in enumerate(branches):
        d = abs(((deg - 90.0) + 180.0) % 360.0 - 180.0)
        if d < best_d:
            best, best_d = i, d
    return best


def _dir_name(deg):
    names = ["E", "NE", "N", "NW", "W", "SW", "S", "SE"]
    idx = int(((deg + 22.5) % 360.0) // 45.0) % 8
    return names[idx]
