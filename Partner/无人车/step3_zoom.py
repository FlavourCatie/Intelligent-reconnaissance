# -*- coding: utf-8 -*-
"""Step3 放大图: 只切出"出生点→路障→绕行"区域, 放大 3x, 让红盘和两条路径看清。"""
import os
import numpy as np
from PIL import Image

_HERE = os.path.dirname(os.path.abspath(__file__))
SKL_PNG = os.path.join(_HERE, "skeletonized_image.png")
OUT_PNG = os.path.join(_HERE, "step3_replan_zoom.png")

SCALE = 1.0
WORLD_OFFSET = (-2094.5277099609375, -2637.31982421875)

START = (264, 2119)
BARRIER = (849, 2117)
GOAL = (1394, 2082)
BLOCK_R = 8

# 裁切范围 (覆盖整个绕行: 北到 y~1760, 东到 goal~1400)
X0, X1 = 200, 1460
Y0, Y1 = 1725, 2145
ZOOM = 3


def block_disk(binary, cx, cy, r):
    blocked = binary.copy()
    H, W = binary.shape
    yy, xx = np.ogrid[:H, :W]
    blocked[(xx - cx) ** 2 + (yy - cy) ** 2 <= r * r] = False
    return blocked


def main():
    skl = np.array(Image.open(SKL_PNG))
    binary = np.all(skl >= 200, axis=-1) if skl.ndim == 3 else (skl >= 200)
    blocked = block_disk(binary, BARRIER[0], BARRIER[1], BLOCK_R)

    H, W = binary.shape
    rgb = np.zeros((H, W, 3), np.uint8)
    rgb[binary] = [70, 70, 70]

    # 红盘: 实心红 + 白描边
    disk = binary & ~blocked
    rgb[disk] = [255, 40, 40]
    yy, xx = np.ogrid[:H, :W]
    ring = ((xx - BARRIER[0]) ** 2 + (yy - BARRIER[1]) ** 2 <= (BLOCK_R + 1) ** 2) & \
           ((xx - BARRIER[0]) ** 2 + (yy - BARRIER[1]) ** 2 >= (BLOCK_R - 1) ** 2)
    rgb[ring] = [255, 255, 255]

    # 原路径: 蓝 (用 step3 的结果路径, 这里重算只画直线段示意, 但为准确重跑一次 A*)
    import heapq, itertools, math
    def astar(binm, s, g):
        Hh, Ww = binm.shape
        def h(p): return math.hypot(p[0] - g[0], p[1] - g[1])
        gs = {s: 0.0}; par = {s: None}; cnt = itertools.count(1)
        heap = [(h(s), 0, s)]; closed = set()
        mv = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]
        while heap:
            _, _, cur = heapq.heappop(heap)
            if cur in closed: continue
            closed.add(cur)
            if cur == g:
                p = []
                while cur is not None: p.append(cur); cur = par[cur]
                return p[::-1]
            cx, cy = cur
            for dx, dy in mv:
                nx, ny = cx+dx, cy+dy
                if not (0 <= nx < Ww and 0 <= ny < Hh): continue
                if not binm[ny, nx]: continue
                n = (nx, ny)
                if n in closed: continue
                ng = gs[cur] + (1.414 if dx and dy else 1.0)
                if ng < gs.get(n, 1e18):
                    gs[n] = ng; par[n] = cur
                    heapq.heappush(heap, (ng + h(n), next(cnt), n))
        return None

    orig = astar(binary, START, GOAL)
    detour = astar(blocked, START, GOAL)

    for (x, y) in orig:
        rgb[max(0,y-2):y+3, max(0,x-2):x+3] = [40, 120, 255]
    for (x, y) in detour:
        rgb[max(0,y-2):y+3, max(0,x-2):x+3] = [255, 220, 0]

    # 起点绿 / 终点品红 / 路障中心十字
    for (px, py, col) in [(START[0], START[1], [0,255,0]),
                          (GOAL[0], GOAL[1], [255,0,255])]:
        rgb[py-6:py+7, px-6:px+7] = col
    rgb[BARRIER[1]-1:BARRIER[1]+2, BARRIER[0]-8:BARRIER[0]+8] = [0, 0, 0]
    rgb[BARRIER[1]-8:BARRIER[1]+8, BARRIER[0]-1:BARRIER[0]+2] = [0, 0, 0]

    crop = rgb[Y0:Y1, X0:X1]
    crop = crop.repeat(ZOOM, axis=0).repeat(ZOOM, axis=1)
    Image.fromarray(crop).save(OUT_PNG)
    print("saved %s  (crop %dx%d -> %dx%d)"
          % (OUT_PNG, X1-X0, Y1-Y0, (X1-X0)*ZOOM, (Y1-Y0)*ZOOM))


if __name__ == "__main__":
    main()
