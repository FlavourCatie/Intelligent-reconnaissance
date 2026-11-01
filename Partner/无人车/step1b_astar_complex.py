# -*- coding: utf-8 -*-
"""Step 1b: 选一个最远的路口当目标, 测 A* 耗时, 并可视化"搜索足迹 + 最终路径"。"""
import os
import time
import math
import heapq
import itertools
import numpy as np
from PIL import Image

from swarmae.SwarmAEClient import SwarmAEClient

_HERE = os.path.dirname(os.path.abspath(__file__))
SKL_PNG = os.path.join(_HERE, "skeletonized_image.png")


def load_binary(game):
    _, scale, world_offset, _, _ = game.get_road_network()
    skl = np.array(Image.open(SKL_PNG))
    binary = np.all(skl >= 200, axis=-1) if skl.ndim == 3 else (skl >= 200)
    return binary, scale, world_offset


def w2p(x, y, scale, off):
    return int(scale * (x - off[0])), int(scale * (y - off[1]))


def p2w(px, py, scale, off):
    return px / scale + off[0], py / scale + off[1]


def count_neighbors(binary):
    """每个路点像素的 8 邻域路点计数 (路口 >=3)。"""
    H, W = binary.shape
    pad = np.pad(binary.astype(np.int8), 1, mode='constant')
    cnt = np.zeros((H, W), dtype=np.int16)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            cnt += pad[1 + dy:1 + dy + H, 1 + dx:1 + dx + W]
    return cnt


def astar(binary, start, goal):
    """返回 (path, closed) —— closed 是探索过的节点集合(用于可视化搜索足迹)。"""
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
            return path[::-1], closed
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
    return None, closed


def corners(path):
    """把路径压缩成拐点序列 (方向发生变化的点)。"""
    if len(path) < 2:
        return path[:]
    pts = [path[0]]
    prev = (path[1][0] - path[0][0], path[1][1] - path[0][1])
    for i in range(1, len(path) - 1):
        d = (path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
        if d != prev:
            pts.append(path[i])
            prev = d
    pts.append(path[-1])
    return pts


def draw_footprint(binary, closed, path, start, goal, out_png):
    H, W = binary.shape
    rgb = np.zeros((H, W, 3), np.uint8)
    rgb[binary] = [45, 45, 45]              # 路网: 暗灰
    for (x, y) in closed:                   # 搜索足迹: 浅蓝
        for dy in (0, 1):
            for dx in (0, 1):
                yy, xx = y + dy, x + dx
                if 0 <= xx < W and 0 <= yy < H:
                    rgb[yy, xx] = [90, 150, 255]
    for (x, y) in path:                     # 最终路径: 黄
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                yy, xx = y + dy, x + dx
                if 0 <= xx < W and 0 <= yy < H:
                    rgb[yy, xx] = [255, 220, 0]
    sx, sy = start
    gx, gy = goal
    rgb[sy, sx] = [0, 255, 0]               # 起点: 绿
    rgb[gy, gx] = [255, 0, 0]               # 终点: 红
    Image.fromarray(rgb).save(out_png)


def main():
    AeClient = SwarmAEClient(ue_ip="127.0.0.1", ue_port=2000)
    _, game, _ = AeClient.get_game()
    binary, scale, off = load_binary(game)
    H, W = binary.shape

    # 找路口 (8 邻域路点 >=3)
    cnt = count_neighbors(binary)
    jy, jx = np.where((binary) & (cnt >= 3))
    junctions = list(zip(jx.tolist(), jy.tolist()))
    print("路口总数: %d" % len(junctions))

    start = (264, 2119)  # 出生点
    # 选离出生点最远的路口
    goal = max(junctions, key=lambda p: (p[0] - start[0]) ** 2 + (p[1] - start[1]) ** 2)
    print("start px=%s world=(%.1f,%.1f)" % (start, *p2w(*start, scale, off)))
    print("goal  px=%s world=(%.1f,%.1f)" % (goal, *p2w(*goal, scale, off)))

    t0 = time.perf_counter()
    path, closed = astar(binary, start, goal)
    dt = time.perf_counter() - t0

    if path is None:
        print("NO PATH FOUND")
        return

    length = 0.0
    for a, b in zip(path, path[1:]):
        length += (1.414 if (a[0] != b[0] and a[1] != b[1]) else 1.0)
    print("path points=%d  length=%.1f m  time=%.3f s" % (len(path), length, dt))
    print("nodes_expanded(closed)=%d" % len(closed))

    # 拐点(路线怎么拐的)
    cs = corners(path)
    print("拐点数=%d, 路线:" % len(cs))
    for px, py in cs:
        wx, wy = p2w(px, py, scale, off)
        print("   px=(%d,%d) world=(%.1f,%.1f)" % (px, py, wx, wy))

    out_png = os.path.join(_HERE, "step1b_footprint.png")
    draw_footprint(binary, closed, path, start, goal, out_png)
    print("saved %s" % out_png)


if __name__ == "__main__":
    main()
