# -*- coding: utf-8 -*-
"""Step 4: 真车在线重规划。

沿 A* 路径行驶; 边开边探测; 一旦路障挡在剩余路径上:
  1. 刹停
  2. 把探测到的路障在路网上"封格子"(半径 BLOCK_R)
  3. 从当前位置重跑 A* 得到绕行路径
  4. 若绕行路径要先往回走(死路倒车), 就直线倒车退出路口
  5. 继续沿新路径绕行; 没路可走则停车上报(Step 5 兜底)
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
TARGET_V     = 5.0     # 直道巡航车速 (m/s)
SLOW_V       = 2.5     # 临近拐弯车速 (m/s)
SLOW_RANGE   = 20.0    # 距拐弯多少米减速 (m)
LOOKAHEAD    = 8.0     # 预瞄距离 (m)
ARRIVE_DIST  = 20.0    # 距目标多近算到达 (m)
CLEARANCE    = 10.0    # 路障中心离路径多近算"挡路" (m)
MAX_RUNTIME  = 600.0   # 兜底最长行驶时间 (s)
MAX_REPLAN   = 5       # 最多重规划次数
BLOCK_R      = 10      # 封格子半径 (px≈m, 路障半径+车半宽+余量)
REVERSE_OVERSHOOT = 12.0  # 倒车退出路口时, 越过路口再倒多远 (m)
DEV_WARN     = 5.0     # 距路中心线超过多少米判定"偏离路面" (m)

SPAWN      = (-1829.6, -517.7, 2.1)
GOAL_WORLD = (-1500.0, -520.0)   # 快速测试: 出生点以东 330m, 路障(-1245)以西, 纯跟车+到达


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


def block_disk(grid, cx, cy, r):
    H, W = grid.shape
    yy, xx = np.ogrid[:H, :W]
    grid[(xx - cx) ** 2 + (yy - cy) ** 2 <= r * r] = False


def nearest_idx(path, x, y):
    best_i, best_d = 0, 1e18
    for i, (px, py) in enumerate(path):
        d = (px - x) ** 2 + (py - y) ** 2
        if d < best_d:
            best_d, best_i = d, i
    return best_i


def lookahead_pose(path, x, y, L, span=5):
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


def needs_reverse(path, pose, look=6):
    """绕行路径开头是不是要往回走 (与当前朝向相反)。"""
    if len(path) < look + 1:
        return False
    x0, y0 = path[0]
    x1, y1 = path[look]
    dx, dy = x1 - x0, y1 - y0
    if dx == 0 and dy == 0:
        return False
    fwd = (math.cos(pose.theta_), math.sin(pose.theta_))
    return (dx * fwd[0] + dy * fwd[1]) < 0


def reverse_distance(ugv, dist_m, throttle=-0.7, max_t=150.0):
    """直线倒车 dist_m 米 (用 imu.v 积分里程), 到点后刹停。"""
    t0 = time.time()
    prev = time.time()
    traveled = 0.0
    print("    倒车退出死路 %.1f m ..." % dist_m)
    while traveled < dist_m and (time.time() - t0) < max_t:
        imu = ugv.get_imu_data()
        dt = time.time() - prev
        prev = time.time()
        traveled += abs(imu.v) * dt
        ugv.apply_control(0, throttle, 0, False)
        time.sleep(0.05)
    ugv.apply_control(0, 0, 0.8, False)
    time.sleep(1.0)
    ugv.apply_control(0, 0, 0, True)
    time.sleep(0.3)
    return traveled


def main():
    AeClient = SwarmAEClient(ue_ip="127.0.0.1", ue_port=2000)
    _, game, _ = AeClient.get_game()
    game.stage_start('plan_start')
    game.stage_start('reconnaissance_start')
    game.stage_complete('reconnaissance_end')
    game.stage_start('vau_reconnaissance_start')
    game.stage_complete('vau_reconnaissance_end')

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

    binary, scale, off = load_binary(game)
    blocked_grid = binary.copy()          # 运行期间被路障封掉的路网
    blocked_obstacles = set()             # 已处理过的路障世界坐标(避免重复触发)

    def plan_from(pose):
        sp = nearest_road(blocked_grid, *w2p(pose.x_, pose.y_, scale, off))
        gp = nearest_road(blocked_grid, *w2p(*GOAL_WORLD, scale, off))
        if sp is None or gp is None:
            return None
        path_px = astar(blocked_grid, sp, gp)
        if path_px is None:
            return None
        return [p2w(px, py, scale, off) for (px, py) in path_px]

    pose = ugv.get_transform()
    path = plan_from(pose)
    if path is None:
        print("NO PATH")
        return
    corners = detect_corners(path)
    print("初始路径 points=%d  corners=%d  goal=%s" % (len(path), len(corners), GOAL_WORLD))
    for ci in corners:
        print("  corner world=(%.1f, %.1f)" % (path[ci][0], path[ci][1]))

    stanley = control.Stanley(k=1)
    pid = control.PID(Kp=0.1, Ki=0.0, Kd=0.0)
    initial = time.time()
    prev = time.time()
    dt = 0.05
    replan_count = 0
    max_dev = 0.0
    stuck_since = None

    print("start driving ...\n")
    while True:
        pose = ugv.get_transform()
        imu = ugv.get_imu_data()
        state = struct.State(pose, abs(imu.v), imu.a)

        # 到达判断: 比的是"吸附到路的最近点"(path[-1]), 不是原始 GOAL_WORLD。
        # 之前用 GOAL_WORLD 比对: 它落在路外 ~30m, 车永远到不了, 冲过终点后 Stanley
        # 对"身后的终点"算出 0 侧偏 → 一路直线向北冲出路面(冲进树丛)。
        gx, gy = path[-1]
        if utils.distance(pose.x_, pose.y_, gx, gy) < ARRIVE_DIST:
            ugv.apply_control(0, 0, 0.8, False)
            time.sleep(1)
            ugv.apply_control(0, 0, 0, True)
            print("\n>>> 到达目标附近, 停车. 位置 (%.1f, %.1f)  距路网终点 %.1f m  距原始目标 %.1f m  max_dev=%.1f m"
                  % (pose.x_, pose.y_,
                     utils.distance(pose.x_, pose.y_, gx, gy),
                     utils.distance(pose.x_, pose.y_, GOAL_WORLD[0], GOAL_WORLD[1]), max_dev))
            break

        # 探测, 找"挡在剩余路径上且未处理"的路障
        ob = ugv.detect_fun()
        blocking = None
        if len(ob.Type):
            imin = nearest_idx(path, pose.x_, pose.y_)
            for i in range(len(ob.Type)):
                if not is_barricade(ob.Type[i]):
                    continue
                bx, by = ob.x_ue[i], ob.y_ue[i]
                key = (round(bx), round(by))
                if key in blocked_obstacles:
                    continue
                md = min(utils.distance(bx, by, px, py) for (px, py) in path[imin:])
                if md < CLEARANCE:
                    blocking = (ob.Type[i], bx, by, md)
                    break

        if blocking:
            # 1) 刹停
            ugv.apply_control(0, 0, 0.8, False)
            time.sleep(1)
            ugv.apply_control(0, 0, 0, True)
            t, bx, by, md = blocking
            print("\n>>> 路障挡路: 类型=%s 路障=(%.1f, %.1f) 距路径 %.1f m"
                  % (t, bx, by, md))
            print("    停车位置 (%.1f, %.1f)" % (pose.x_, pose.y_))

            # 2) 把当前探测到的所有路障封进路网
            for i in range(len(ob.Type)):
                if not is_barricade(ob.Type[i]):
                    continue
                bpx, bpy = w2p(ob.x_ue[i], ob.y_ue[i], scale, off)
                block_disk(blocked_grid, bpx, bpy, BLOCK_R)
                blocked_obstacles.add((round(ob.x_ue[i]), round(ob.y_ue[i])))
            print("    已封 %d 个路障格子" % sum(1 for i in range(len(ob.Type)) if is_barricade(ob.Type[i])))

            # 3) 从当前位置重规划
            replan_count += 1
            if replan_count > MAX_REPLAN:
                print("    重规划次数超限, 停车上报。")
                break
            new_path = plan_from(pose)
            if new_path is None:
                print("    >>> 无路可走(Step5 兜底): 停车上报。位置 (%.1f, %.1f)"
                      % (pose.x_, pose.y_))
                break

            # 4) 若开头要往回走, 先倒车退出死路
            if needs_reverse(new_path, pose):
                c = detect_corners(new_path)
                if not c:
                    print("    绕行路径无拐点, 无法倒车退出, 停车上报。")
                    break
                corner = new_path[c[0]]
                d0 = utils.distance(pose.x_, pose.y_, corner[0], corner[1])
                print("    绕行需先回退到路口 (%.1f, %.1f), 距 %.1f m"
                      % (corner[0], corner[1], d0))
                reverse_distance(ugv, d0 + REVERSE_OVERSHOOT)
                pose = ugv.get_transform()
                replan_count += 1
                new_path = plan_from(pose)
                if new_path is None:
                    print("    >>> 倒车后无路可走, 停车上报。")
                    break

            path = new_path
            corners = detect_corners(path)
            print("    重规划完成: points=%d  corners=%d" % (len(path), len(corners)))
            prev = time.time()
            dt = 0.05
            continue

        if (time.time() - initial) > MAX_RUNTIME:
            ugv.apply_control(0, 0, 0.8, False)
            time.sleep(1)
            ugv.apply_control(0, 0, 0, True)
            print("\n超时停车. 位置 (%.1f, %.1f)  max_dev=%.1f m" % (pose.x_, pose.y_, max_dev))
            break

        # 临近拐弯减速
        imin = nearest_idx(path, pose.x_, pose.y_)
        near_corner = False
        for ci in corners:
            if ci > imin and utils.distance(pose.x_, pose.y_, path[ci][0], path[ci][1]) < SLOW_RANGE:
                near_corner = True
                break
        target_v = SLOW_V if near_corner else TARGET_V

        # 距路中心线横向偏差 (路中心线 = A* 路径)
        dev = math.hypot(pose.x_ - path[imin][0], pose.y_ - path[imin][1])
        max_dev = max(max_dev, dev)
        if dev > DEV_WARN:
            print("  [OFF-ROAD] dev=%.1f m pos=(%.1f,%.1f) v=%.1f heading=%.1fdeg near_corner=%s"
                  % (dev, pose.x_, pose.y_, state.v_, math.degrees(pose.theta_), near_corner))

        # 撞停检测: 想走但速度持续≈0 (撞树/卡住)
        if state.v_ < 0.3 and target_v > 0.5:
            if stuck_since is None:
                stuck_since = time.time()
            elif time.time() - stuck_since > 8.0:
                ugv.apply_control(0, 0, 0.8, False)
                time.sleep(1)
                ugv.apply_control(0, 0, 0, True)
                print("\n>>> 疑似撞停(速度<0.3 持续 8s). 位置 (%.1f,%.1f) dev=%.1f max_dev=%.1f"
                      % (pose.x_, pose.y_, dev, max_dev))
                break
        else:
            stuck_since = None

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
            print("  pos=(%.1f,%.1f) v=%.1f dev=%.1f slow=%s replan=%d detect=[%s]"
                  % (pose.x_, pose.y_, state.v_, dev, near_corner, replan_count, det_str))

        dt = time.time() - prev
        prev = time.time()
        time.sleep(0.05)


if __name__ == "__main__":
    main()
