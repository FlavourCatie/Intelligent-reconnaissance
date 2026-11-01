# -*- coding: utf-8 -*-
"""
无人车1 (UGV1) 随机选择一条路, 开到装甲车 (Stryker) 旁边。

流程:
    1. 连仿真器, 注册 UGV1 节点, 开启相关阶段 (plan_start / reconnaissance_start)。
    2. 获取道路网络 -> 骨架细化 -> 二值化路网。
    3. 探测装甲车, 取其世界坐标作为最终目标点。
    4. 把目标点吸附到最近路点; 起点与目标都在路网上用"随机 A*"找一条路径
       (平局时随机选, 即"随机选路")。
    5. 沿路路径 -> 三次 B 样条拟合, Stanley + PID 控制 UGV1 行驶。
    6. 到达离装甲车最近的路点后, 若还没到装甲车旁 (装甲车不在路上), 再做
       最后一段 off-road 直线逼近, 距装甲车 < STOP_DIST 时停车。
"""
import math
import time
import random
import heapq
import numpy as np

from swarmae.SwarmAEClient import SwarmAEClient

import lib.swarm_api as api
import lib.image_process as imp
import lib.frenet_optimal_planner as fop
import lib.controller as control
import lib.data_struct as struct
import lib.utils as utils

# 控制参数
TARGET_V     = 5.0    # 目标车速 (m/s)
STOP_DIST    = 2.0    # 距装甲车多近算"到达" (m)
MAX_RUNTIME  = 60.0   # 每段最多跑多久 (s)
ARMY_VEHICLE = 'Stryker'  # 装甲车类型名


def nearest_road(binary, x, y, search=30):
    """在二值路网上找 (x,y) 附近最近的路点 (图像坐标, x=列, y=行)。"""
    H, W = binary.shape
    best, best_pt = None, None
    for dy in range(-search, search + 1):
        for dx in range(-search, search + 1):
            nx, ny = x + dx, y + dy
            if 0 <= nx < W and 0 <= ny < H and binary[ny][nx]:
                d = dx * dx + dy * dy
                if best is None or d < best:
                    best, best_pt = d, (nx, ny)
    return best_pt


def randomized_astar(binary, start, goal):
    """随机 A*: 在 f 值接近的节点里随机选择, 实现"随机选一条路"。"""
    H, W = binary.shape
    start = nearest_road(binary, *start) or start
    goal = nearest_road(binary, *goal) or goal
    if start == goal:
        return [start]

    def h(p):
        return abs(p[0] - goal[0]) + abs(p[1] - goal[1])

    g_cost = {start: 0.0}
    parent = {start: None}
    counter = [0]
    heap = []
    heapq.heappush(heap, (h(start) + random.random(), counter[0], start))
    counter[0] += 1
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
            if not binary[ny][nx]:
                continue
            nxt = (nx, ny)
            ng = g_cost[cur] + (1.414 if dx and dy else 1.0)
            if nxt in closed:
                continue
            if ng < g_cost.get(nxt, 1e9):
                g_cost[nxt] = ng
                parent[nxt] = cur
                heapq.heappush(heap, (ng + h(nxt) + random.random(),
                                      counter[0], nxt))
                counter[0] += 1
    return None


def calculate_target_pose(vehicle_pose, ref_path):
    """计算参考路径上的预瞄点 (照搬 plan_control.py 的思路)。"""
    front_x = vehicle_pose.x_ + 10 * math.cos(vehicle_pose.theta_)
    front_y = vehicle_pose.y_ + 10 * math.sin(vehicle_pose.theta_)

    dx = np.ravel(ref_path.interp_x_) - front_x
    dy = np.ravel(ref_path.interp_y_) - front_y
    dist = np.abs(dx) + np.abs(dy)
    idx = int(np.argmin(dist))

    return struct.Transform(
        ref_path.interp_x_[idx, 0],
        ref_path.interp_y_[idx, 0],
        ref_path.interp_theta_[idx, 0])


def build_ref_path(world_pts, resolution=2.0):
    """把世界坐标点列表拟合为三次 B 样条参考路径。"""
    waypoint_x = np.mat([p[0] for p in world_pts]).T
    waypoint_y = np.mat([p[1] for p in world_pts]).T
    return fop.ReferencePath(waypoint_x, waypoint_y, resolution=resolution)


def drive_to(ugv, ref_path, goal_xy, target_v, stop_dist, max_runtime, label):
    """沿 ref_path 行驶, 直到距 goal_xy < stop_dist (或超时), 然后停车。

    返回结束时的实际距离。"""
    stanley = control.Stanley(k=1)
    pid = control.PID(Kp=0.1, Ki=0.0, Kd=0.0)

    initial_time = time.time()
    prev_time = time.time()
    dt = 0.05

    while True:
        pose = ugv.get_transform()
        imu = ugv.get_imu_data()
        state = struct.State(pose, abs(imu.v), imu.a)

        dist2goal = utils.distance(pose.x_, pose.y_, goal_xy[0], goal_xy[1])
        if dist2goal < stop_dist or (time.time() - initial_time) > max_runtime:
            ugv.apply_control(0, 0, 0.8, False)
            time.sleep(1)
            ugv.apply_control(0, 0, 0, True)
            print("  [%s] 停车, 距目标 %.1f m" % (label, dist2goal))
            return dist2goal

        target_pose = calculate_target_pose(pose, ref_path)
        delta = stanley.update_control(target_pose, state)
        accel = pid.update_control(target_v, state.v_, dt)

        steer = delta / (math.pi / 6)
        if accel > 0:
            throttle, brake = min(accel, 1.0), 0.0
        else:
            throttle, brake = 0.0, min(abs(accel) * 0.5, 1.0)

        ugv.apply_control(steer, throttle, brake, False)

        if int(time.time() * 2) % 4 == 0:
            print("  [%s] pos=(%.1f, %.1f) v=%.1f 距目标 %.1f m"
                  % (label, pose.x_, pose.y_, state.v_, dist2goal))

        dt = time.time() - prev_time
        prev_time = time.time()
        time.sleep(0.05)


def main():
    AeClient = SwarmAEClient(ue_ip="127.0.0.1", ue_port=2000)
    _, game, _ = AeClient.get_game()

    # 开启阶段 (plan_start 是无人车控制生效的关键)
    game.stage_start('plan_start')
    game.stage_start('reconnaissance_start')
    game.stage_start('vau_reconnaissance_start')

    ugv = api.Vehicle(AeClient, "UGV1", 1, "ugv")

    # 探测装甲车, 作为最终目标
    ob = ugv.detect_fun()
    goal_world = None
    for i, t in enumerate(ob.Type):
        if t == ARMY_VEHICLE:
            goal_world = (ob.x_ue[i], ob.y_ue[i])
            print("找到装甲车 %s @ (%.1f, %.1f)" % (t, goal_world[0], goal_world[1]))
    if goal_world is None:
        print("未探测到装甲车, 使用已知坐标 (-1829.6, -500.7)")
        goal_world = (-1829.6, -500.7)

    # 路网 + 骨架
    img, scale, world_offset, _, _ = game.get_road_network()
    print("scale =", scale, "world_offset =", world_offset)
    img_skl = imp.skeleton_refinement(img)
    skl = np.array(img_skl)
    binary = np.all(skl >= 200, axis=-1)  # True = 道路

    # 世界坐标 <-> 图像坐标
    def w2p(x, y):
        return int(scale * (x - world_offset[0])), int(scale * (y - world_offset[1]))

    def p2w(px, py):
        return px + world_offset[0], py + world_offset[1]

    pose = ugv.get_transform()
    start_px = w2p(pose.x_, pose.y_)
    goal_px = w2p(goal_world[0], goal_world[1])
    print("起点像素:", start_px, " 目标像素:", goal_px)

    # 第 1 段: 沿路网开到离装甲车最近的路点
    goal_road_px = nearest_road(binary, *goal_px)
    road_path = randomized_astar(binary, start_px, goal_road_px)

    if road_path is not None and len(road_path) >= 2:
        road_goal_xy = p2w(*goal_road_px)
        world_pts = [p2w(*p) for p in road_path]
        print("沿路行驶: 随机路径 %d 个点 -> 最近路点 (%.1f, %.1f)"
              % (len(road_path), road_goal_xy[0], road_goal_xy[1]))
        drive_to(ugv, build_ref_path(world_pts), road_goal_xy,
                 TARGET_V, STOP_DIST, MAX_RUNTIME, "road")
    else:
        print("UGV1 已在离装甲车最近的路点上 (路网段跳过)。")

    # 第 2 段: off-road 直线逼近装甲车 (装甲车不在路上)
    pose = ugv.get_transform()
    if utils.distance(pose.x_, pose.y_, goal_world[0], goal_world[1]) < STOP_DIST:
        print("已在装甲车旁, 无需再行驶。")
    else:
        print("最后一段 off-road 逼近装甲车 ...")
        final_pts = [(pose.x_, pose.y_), goal_world]
        drive_to(ugv, build_ref_path(final_pts), goal_world,
                 TARGET_V, STOP_DIST, MAX_RUNTIME, "final")

    pose = ugv.get_transform()
    print("完成。最终位置 (%.1f, %.1f), 装甲车 (%.1f, %.1f), 距离 %.1f m"
          % (pose.x_, pose.y_, goal_world[0], goal_world[1],
             utils.distance(pose.x_, pose.y_, goal_world[0], goal_world[1])))


if __name__ == '__main__':
    main()
