# -*- coding: utf-8 -*-
"""Step 3: 离线验证"封路重规划"——把路障位置从路网里封掉, 看 A* 能否绕开。

纯离线: 只读路网 PNG, 不连仿真器。scale/offset 是已确认的常量。
"""
import os
import time
import math
import heapq
import itertools
import numpy as np
from PIL import Image

_HERE = os.path.dirname(os.path.abspath(__file__))
SKL_PNG = os.path.join(_HERE, "skeletonized_image.png")

SCALE = 1.0
WORLD_OFFSET = (-2094.5277099609375, -2637.31982421875)

BARRIER_WORLD = (-1245.0, -520.0)   # 已知路障
BLOCK_RADIUS  = 8                    # 路障半径+车半宽+余量, 约 8m
START_WORLD   = (-1829.6, -517.7)   # 出生点
GOAL_WORLD    = (-700.0, -520.0)    # 路障以东的目标点


def load_binary():
    skl = np.array(Image.open(SKL_PNG))
    binary = np.all(skl >= 200, axis=-1) if skl.ndim == 3 else (skl >= 200)
    return binary


def w2p(x, y):
    return int(SCALE * (x - WORLD_OFFSET[0])), int(SCALE * (y - WORLD_OFFSET[1]))


def p2w(px, py):
    return px / SCALE + WORLD_OFFSET[0], py / SCALE + WORLD_OFFSET[1]


def nearest_road(binary, px, py, search=60):
    H, W = binary.shape
    best, best_pt = None, None
    for dy in range(-search, search + 1):
        for dx in range(-search, search + 1):
            nx, ny = px + dx, py + dy
            if 0 <= nx < W and 0 <= ny < H and binary[ny, nx]:
                d = dx * dx + dy * dy
                if best is None or d < best:
                    best, best_pt = d, (nx, ny)
    return best_pt


def astar(binary, start, goal):
    H, W = binary.shape

    def h(p):
        return math.hypot(p[0] - goal[0], p[1] - goal[1])

    g = {start: 0.0}
    parent = {start: None}
    cnt = itertools.count(1)
    heap = [(h(start), 0, start)]
    closed = set()
    moves = [(1, 0), (-1, 0), (0, 1), (0, -1),
             (1, 1), (1, -1), (-1, 1), (-1, -1)]
    while heap:
        _, _, cur = heapq.heappop(heap)
        if cur in closed:
            continue
        closed.add(cur)
        if cur == goal:
            path = []
            while cur is not None:
                path.append(cur)
                cur = parent[cur]
            return path[::-1]
        cx, cy = cur
        for dx, dy in moves:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < W and 0 <= ny < H):
                continue
            if not binary[ny, nx]:
                continue
            nxt = (nx, ny)
            if nxt in closed:
                continue
            ng = g[cur] + (1.414 if dx and dy else 1.0)
            if ng < g.get(nxt, 1e18):
                g[nxt] = ng
                parent[nxt] = cur
                heapq.heappush(heap, (ng + h(nxt), next(cnt), nxt))
    return None


def block_disk(binary, cx, cy, r):
    blocked = binary.copy()
    H, W = binary.shape
    yy, xx = np.ogrid[:H, :W]
    mask = (xx - cx) ** 2 + (yy - cy) ** 2 <= r * r
    blocked[mask] = False
    return blocked


def path_len(path):
    L = 0.0
    for a, b in zip(path, path[1:]):
        L += (1.414 if (a[0] != b[0] and a[1] != b[1]) else 1.0)
    return L


def corners(path, span=6, min_angle_deg=25):
    n = len(path)
    out = []
    for i in range(span, n - span):
        xa, ya = path[i - span]
        xb, yb = path[i]
        xc, yc = path[i + span]
        th1 = math.atan2(yb - ya, xb - xa)
        th2 = math.atan2(yc - yb, xc - xb)
        d = abs(th2 - th1)
        d = min(d, 2 * math.pi - d)
        if d > math.radians(min_angle_deg):
            out.append(i)
    return out


def draw(binary, blocked, orig, detour, start, goal, out_png):
    H, W = binary.shape
    rgb = np.zeros((H, W, 3), np.uint8)
    rgb[binary] = [60, 60, 60]
    rgb[binary & ~blocked] = [255, 60, 60]   # 封掉的路段: 红
    for (x, y) in orig:                       # 原路径: 蓝
        rgb[max(0, y - 1):min(H, y + 2), max(0, x - 1):min(W, x + 2)] = [60, 120, 255]
    for (x, y) in detour:                     # 绕行路径: 黄
        rgb[max(0, y - 1):min(H, y + 2), max(0, x - 1):min(W, x + 2)] = [255, 220, 0]
    sx, sy = start
    gx, gy = goal
    rgb[sy, sx] = [0, 255, 0]                 # 起点: 绿
    rgb[gy, gx] = [255, 0, 255]               # 终点: 品红
    Image.fromarray(rgb).save(out_png)


def main():
    binary = load_binary()

    sp = nearest_road(binary, *w2p(*START_WORLD))
    gp = nearest_road(binary, *w2p(*GOAL_WORLD))
    bp = nearest_road(binary, *w2p(*BARRIER_WORLD))
    print("start px=%s world=%s" % (sp, tuple(round(v, 1) for v in p2w(*sp))))
    print("goal  px=%s world=%s" % (gp, tuple(round(v, 1) for v in p2w(*gp))))
    print("barrier px=%s world=%s" % (bp, tuple(round(v, 1) for v in p2w(*bp))))

    # 1) 无路障原始路径
    t0 = time.perf_counter()
    orig = astar(binary, sp, gp)
    dt0 = time.perf_counter() - t0
    print("\n[original] length=%.1f m  time=%.3f s" % (path_len(orig), dt0))

    # 2) 封掉路障后重规划 (从起点)
    blocked = block_disk(binary, bp[0], bp[1], BLOCK_RADIUS)
    t0 = time.perf_counter()
    detour = astar(blocked, sp, gp)
    dt1 = time.perf_counter() - t0
    print("[detour from start] length=%.1f m  time=%.3f s  (+%.1f m)"
          % (path_len(detour), dt1, path_len(detour) - path_len(orig)))
    for ci in corners(detour):
        wx, wy = p2w(*detour[ci])
        print("   turn px=%s world=(%.1f, %.1f)" % (detour[ci], wx, wy))

    # 3) 模拟"车开到离路障 100m 时才探测到, 从当前位置重规划"
    mid = (bp[0] - 100, bp[1])
    mid = nearest_road(blocked, mid[0], mid[1]) or mid
    t0 = time.perf_counter()
    detour2 = astar(blocked, mid, gp)
    dt2 = time.perf_counter() - t0
    print("[replan from 100m-before-barrier px=%s] length=%.1f m  time=%.3f s"
          % (mid, path_len(detour2), dt2))

    out_png = os.path.join(_HERE, "step3_replan.png")
    draw(binary, blocked, orig, detour, sp, gp, out_png)
    print("saved %s" % out_png)


if __name__ == "__main__":
    main()
