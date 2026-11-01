# -*- coding: utf-8 -*-
"""Step 1: 离线验证 A* 最短路径 (不控制车, 只读路网 + 画图 + 计时)。"""
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
    assert os.path.exists(SKL_PNG), "missing %s" % SKL_PNG
    skl = np.array(Image.open(SKL_PNG))
    if skl.ndim == 3:
        binary = np.all(skl >= 200, axis=-1)
    else:
        binary = skl >= 200
    return binary, scale, world_offset


def w2p(x, y, scale, off):
    return int(scale * (x - off[0])), int(scale * (y - off[1]))


def p2w(px, py, scale, off):
    return px / scale + off[0], py / scale + off[1]


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
            return path[::-1], len(closed)
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
    return None, len(closed)


def draw(binary, path, start, goal, out_png):
    H, W = binary.shape
    rgb = np.zeros((H, W, 3), np.uint8)
    rgb[binary] = [70, 70, 70]
    for (x, y) in path:
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                yy, xx = y + dy, x + dx
                if 0 <= xx < W and 0 <= yy < H:
                    rgb[yy, xx] = [255, 220, 0]
    sx, sy = start
    gx, gy = goal
    rgb[sy, sx] = [0, 255, 0]
    rgb[gy, gx] = [255, 0, 0]
    Image.fromarray(rgb).save(out_png)


def main():
    AeClient = SwarmAEClient(ue_ip="127.0.0.1", ue_port=2000)
    _, game, _ = AeClient.get_game()
    binary, scale, off = load_binary(game)
    print("road grid: %dx%d  scale=%s  world_offset=%s"
          % (binary.shape[1], binary.shape[0], scale, off))

    start_world = (-1829.6, -517.7)   # UGV1 出生点
    tests = [
        ("straight_barricade", (-1245.0, -520.0)),   # 同一条路, 长直线 (计时)
        ("vertical_turn",      (-1488.5, -600.0)),   # 需要拐上南北向路 (拐弯)
    ]

    for name, goal_world in tests:
        sp = nearest_road(binary, *w2p(*start_world, scale, off))
        gp = nearest_road(binary, *w2p(*goal_world, scale, off))
        t0 = time.perf_counter()
        path, nclosed = astar(binary, sp, gp)
        dt = time.perf_counter() - t0

        print("\n=== %s ===" % name)
        print("start world %s -> px %s" % (start_world, sp))
        print("goal  world %s -> px %s" % (goal_world, gp))
        if path is None:
            print("NO PATH FOUND")
            continue

        length = 0.0
        for a, b in zip(path, path[1:]):
            length += (1.414 if (a[0] != b[0] and a[1] != b[1]) else 1.0)
        print("path points=%d  length=%.1f m  time=%.3f s  nodes_expanded=%d"
              % (len(path), length, dt, nclosed))

        print("sampled waypoints:")
        for i in range(0, len(path), 100):
            px, py = path[i]
            wx, wy = p2w(px, py, scale, off)
            print("   px=(%d,%d) world=(%.1f,%.1f)" % (px, py, wx, wy))
        px, py = path[-1]
        wx, wy = p2w(px, py, scale, off)
        print("   end px=(%d,%d) world=(%.1f,%.1f)" % (px, py, wx, wy))

        out_png = os.path.join(_HERE, "step1_path_%s.png" % name)
        draw(binary, path, sp, gp, out_png)
        print("saved %s" % out_png)


if __name__ == "__main__":
    main()
