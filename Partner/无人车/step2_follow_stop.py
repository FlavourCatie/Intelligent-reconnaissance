# -*- coding: utf-8 -*-
"""Step 2: 沿 A* 路径行驶到目标, 边开边探测, 路障挡路就刹停(先不重规划)。

路径跟随: 对 A* 路径做"沿路径 lookahead 预瞄"取目标点(避免 B 样条把直角拐弯磨圆),
         临近拐弯自动减速, Stanley + PID 控制。
"""
import os
import time
import math
import heapq
import itertools
import numpy as np
from PIL import Image

from swarmae import SwarmAENode as SN
from swarmae.SwarmAEClient import SwarmAEClient

import lib.swarm_api as api
import lib.controller as control
import lib.data_struct as struct
import lib.utils as utils

_HERE = os.path.dirname(os.path.abspath(__file__))
SKL_PNG = os.path.join(_HERE, "skeletonized_image.png")

# ---- 参数 ----
TARGET_V     = 5.0    # 直道巡航车速 (m/s)
SLOW_V       = 2.5    # 临近拐弯的车速 (m/s)
SLOW_RANGE   = 20.0   # 距拐弯多少米开始减速 (m)
LOOKAHEAD    = 8.0    # 预瞄距离 (m)
ARRIVE_DIST  = 20.0   # 距目标多近算到达 (m)
CLEARANCE    = 10.0   # 路障中心离路径多近算"挡路" (m)
MAX_RUNTIME  = 150.0  # 兜底最长行驶时间 (s)
SPAWN        = (-1829.6, -517.7, 2.1)
GOAL_WORLD   = (-1488.5, -600.0)   # Step1 里的拐弯点


def load_binary(game):
    _, scale, world_offset, _, _ = game.get_road_network()
    skl = np.array(Image.open(SKL_PNG))
    binary = np.all(skl >= 200, axis=-1) if skl.ndim == 3 else (skl >= 200)
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


def nearest_idx(path, x, y):
    best_i, best_d = 0, 1e18
    for i, (px, py) in enumerate(path):
        d = (px - x) ** 2 + (py - y) ** 2
        if d < best_d:
            best_d, best_i = d, i
    return best_i


def lookahead_pose(path, x, y, L, span=5):
    """沿路径找 L 米外的预瞄点, 航向用 span 点跨度算(平滑锯齿)。"""
    n = len(path)
    i = nearest_idx(path, x, y)
    acc = 0.0
    for k in range(i, n - 1):
        x1, y1 = path[k]
        x2, y2 = path[k + 1]
        seg = math.hypot(x2 - x1, y2 - y1)
        if acc + seg >= L:
            frac = (L - acc) / seg if seg > 0 else 0.0
            tx = x1 + frac * (x2 - x1)
            ty = y1 + frac * (y2 - y1)
            j = min(k + span, n - 1)
            theta = math.atan2(path[j][1] - path[k][1], path[j][0] - path[k][0])
            return struct.Transform(tx, ty, theta)
        acc += seg
    j = n - 1
    i0 = max(0, j - span)
    theta = math.atan2(path[j][1] - path[i0][1], path[j][0] - path[i0][0])
    return struct.Transform(path[j][0], path[j][1], theta)


def detect_corners(path, span=6, min_angle_deg=30):
    """找路径上的拐弯点 (返回索引列表), 用于临近减速。"""
    corners = []
    n = len(path)
    for i in range(span, n - span):
        xa, ya = path[i - span]
        xb, yb = path[i]
        xc, yc = path[i + span]
        th1 = math.atan2(yb - ya, xb - xa)
        th2 = math.atan2(yc - yb, xc - xb)
        if abs(utils.normalize_angle(th2 - th1)) > math.radians(min_angle_deg):
            corners.append(i)
    return corners


def is_barricade(t):
    t = (t or "").lower()
    return t == "other" or "barricade" in t


def main():
    AeClient = SwarmAEClient(ue_ip="127.0.0.1", ue_port=2000)
    _, game, _ = AeClient.get_game()
    game.stage_start('plan_start')
    game.stage_start('reconnaissance_start')
    game.stage_start('vau_reconnaissance_start')

    ugv = api.Vehicle(AeClient, "UGV1", 1, "ugv")

    # 复位到出生点
    carla = SN.carla
    actor = ugv.Vehicle_._SwarmAENode__actor
    actor.set_transform(carla.Transform(
        carla.Location(SPAWN[0], SPAWN[1], SPAWN[2]),
        carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)))
    time.sleep(1.5)
    ugv.apply_control(0, 0, 0, True)
    time.sleep(0.3)
    ugv.apply_control(0, 0, 0, False)

    # 规划 A* 路径
    binary, scale, off = load_binary(game)
    sp = nearest_road(binary, *w2p(*SPAWN[:2], scale, off))
    gp = nearest_road(binary, *w2p(*GOAL_WORLD, scale, off))
    path_px = astar(binary, sp, gp)
    if path_px is None:
        print("NO PATH")
        return
    path = [p2w(px, py, scale, off) for (px, py) in path_px]
    corners = detect_corners(path)
    print("path points=%d  corners=%d  goal_world=%s" % (len(path), len(corners), GOAL_WORLD))
    for ci in corners:
        print("  corner idx=%d world=(%.1f,%.1f)" % (ci, path[ci][0], path[ci][1]))

    stanley = control.Stanley(k=1)
    pid = control.PID(Kp=0.1, Ki=0.0, Kd=0.0)
    initial = time.time()
    prev = time.time()
    dt = 0.05

    print("start following ...\n")
    while True:
        pose = ugv.get_transform()
        imu = ugv.get_imu_data()
        state = struct.State(pose, abs(imu.v), imu.a)

        # 到达判断
        if utils.distance(pose.x_, pose.y_, GOAL_WORLD[0], GOAL_WORLD[1]) < ARRIVE_DIST:
            ugv.apply_control(0, 0, 0.8, False)
            time.sleep(1)
            ugv.apply_control(0, 0, 0, True)
            print("\n>>> 到达目标附近, 停车. 位置 (%.1f, %.1f)" % (pose.x_, pose.y_))
            break

        # 探测路障, 判断是否挡在剩余路径上
        ob = ugv.detect_fun()
        blocked = None
        if len(ob.Type):
            imin = nearest_idx(path, pose.x_, pose.y_)
            for i in range(len(ob.Type)):
                if is_barricade(ob.Type[i]):
                    bx, by = ob.x_ue[i], ob.y_ue[i]
                    md = min(utils.distance(bx, by, px, py)
                             for (px, py) in path[imin:])
                    if md < CLEARANCE:
                        blocked = (ob.Type[i], bx, by, md)
                        break

        if blocked:
            ugv.apply_control(0, 0, 0.8, False)
            time.sleep(1)
            ugv.apply_control(0, 0, 0, True)
            t, bx, by, md = blocked
            print("\n>>> 路障挡路, 刹停(不重规划). 类型=%s 路障=(%.1f,%.1f) 距路径 %.1f m"
                  % (t, bx, by, md))
            print("    停车位置 (%.1f, %.1f)" % (pose.x_, pose.y_))
            break

        if (time.time() - initial) > MAX_RUNTIME:
            ugv.apply_control(0, 0, 0.8, False)
            time.sleep(1)
            ugv.apply_control(0, 0, 0, True)
            print("\n超时停车. 位置 (%.1f, %.1f)" % (pose.x_, pose.y_))
            break

        # 临近拐弯减速
        imin = nearest_idx(path, pose.x_, pose.y_)
        near_corner = False
        for ci in corners:
            if ci > imin and utils.distance(pose.x_, pose.y_, path[ci][0], path[ci][1]) < SLOW_RANGE:
                near_corner = True
                break
        target_v = SLOW_V if near_corner else TARGET_V

        target = lookahead_pose(path, pose.x_, pose.y_, LOOKAHEAD)
        delta = stanley.update_control(target, state)
        accel = pid.update_control(target_v, state.v_, dt)
        steer = delta / (math.pi / 6)
        if accel > 0:
            throttle, brake = min(accel, 1.0), 0.0
        else:
            throttle, brake = 0.0, min(abs(accel) * 0.5, 1.0)
        ugv.apply_control(steer, throttle, brake, False)

        if int(time.time() * 2) % 4 == 0:
            det_str = ",".join(ob.Type) if ob.Type else "-"
            print("  pos=(%.1f,%.1f) v=%.1f slow=%s detect=[%s]"
                  % (pose.x_, pose.y_, state.v_, near_corner, det_str))

        dt = time.time() - prev
        prev = time.time()
        time.sleep(0.05)


if __name__ == "__main__":
    main()
