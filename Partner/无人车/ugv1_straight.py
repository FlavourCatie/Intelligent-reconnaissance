# -*- coding: utf-8 -*-
"""
无人车1 (UGV1) 沿当前车道直走, 连过三个路口后停车 (供实时观察)。

流程:
    1. 连仿真器, 注册 UGV1, 开启 plan_start 等阶段。
    2. 从道路骨架里找出出生点所在车道上、东侧的南北向路口 (交叉点)。
    3. 取第 3 个路口, 在其东侧 CROSS_MARGIN 米处作为终点。
    4. 从出生点到终点拟合一条直线参考路径, Stanley + PID 控制 UGV1 直行。
    5. 实时打印位置/速度/已过路口数, 到终点后停车。
"""
import math
import os
import time
import numpy as np
from PIL import Image

from swarmae.SwarmAEClient import SwarmAEClient

import lib.swarm_api as api
import lib.image_process as imp
import lib.frenet_optimal_planner as fop
import lib.controller as control
import lib.data_struct as struct
import lib.utils as utils

TARGET_V      = 10.0   # 目标车速 (m/s)
GOAL_STOP_DIST = 5.0   # 距终点多近算"到达" (m)
INTERSECTIONS  = 1     # 直走通过几个路口
CROSS_MARGIN   = 12.0  # 过最后一个路口后再走多少米停下 (m)
MAX_RUNTIME    = 180.0 # 最多跑多久 (s)


def find_crossings_on_row(binary, row):
    """找某一行上被南北向路穿过的列 (路口 x 坐标, 已按分组取中心)。"""
    W = binary.shape[1]
    cols = [x for x in range(1, W - 1)
            if binary[row, x] and binary[row - 3, x] and binary[row + 3, x]]
    groups = []
    if cols:
        start = prev = cols[0]
        for x in cols[1:]:
            if x - prev > 3:
                groups.append((start + prev) // 2)
                start = x
            prev = x
        groups.append((start + prev) // 2)
    return groups


def build_straight_ref(p0, p1, n=20, resolution=2.0):
    """在 p0 -> p1 之间均匀取 n 个点, 拟合参考路径 (需 >=4 点)。"""
    xs = np.linspace(p0[0], p1[0], n)
    ys = np.linspace(p0[1], p1[1], n)
    return fop.ReferencePath(np.mat(xs).T, np.mat(ys).T, resolution)


def calculate_target_pose(vehicle_pose, ref_path):
    """计算参考路径上的预瞄点。"""
    front_x = vehicle_pose.x_ + 10 * math.cos(vehicle_pose.theta_)
    front_y = vehicle_pose.y_ + 10 * math.sin(vehicle_pose.theta_)
    dx = np.ravel(ref_path.interp_x_) - front_x
    dy = np.ravel(ref_path.interp_y_) - front_y
    dist = np.abs(dx) + np.abs(dy)
    idx = int(np.argmin(dist))
    return struct.Transform(ref_path.interp_x_[idx, 0],
                            ref_path.interp_y_[idx, 0],
                            ref_path.interp_theta_[idx, 0])


def main():
    AeClient = SwarmAEClient(ue_ip="127.0.0.1", ue_port=2000)
    _, game, _ = AeClient.get_game()

    game.stage_start('plan_start')
    game.stage_start('reconnaissance_start')
    game.stage_start('vau_reconnaissance_start')

    ugv = api.Vehicle(AeClient, "UGV1", 1, "ugv")

    # 路网骨架: 直接加载预生成的骨架图, 避免在线 skeletonize 占用过多内存。
    # (skeleton_refinement 会把结果存成 skeletonized_image.png, 与当前地图一致)
    img, scale, world_offset, _, _ = game.get_road_network()
    skl_png = "skeletonized_image.png"
    if os.path.exists(skl_png):
        img_skl = Image.open(skl_png)
    else:
        img_skl = imp.skeleton_refinement(img)
    binary = np.all(np.array(img_skl) >= 200, axis=-1)

    pose = ugv.get_transform()
    spawn_px = (int(scale * (pose.x_ - world_offset[0])),
                int(scale * (pose.y_ - world_offset[1])))
    print("出生点: 世界 (%.1f, %.1f)  像素 %s  朝向 %.3f rad"
          % (pose.x_, pose.y_, spawn_px, pose.theta_))

    # 出生点所在车道上的路口
    row = spawn_px[1]
    crossings = find_crossings_on_row(binary, row)
    east = [c for c in crossings if c > spawn_px[0]]
    print("该车道全部路口(像素x):", crossings)
    print("出生点东侧路口(像素x):", east)

    if len(east) < INTERSECTIONS:
        print("ERROR: 东侧路口不足 %d 个, 无法直走三个路口。" % INTERSECTIONS)
        return

    third = east[INTERSECTIONS - 1]
    goal_world = ((third + CROSS_MARGIN / scale) * scale + world_offset[0],
                  pose.y_)  # 保持在同一车道 (y 不变)
    print("目标: 直走通过 3 个路口后, 在第 %d 个路口(x=%.1f)东侧 %.0f m 处停车"
          % (INTERSECTIONS, third * scale + world_offset[0], CROSS_MARGIN))
    print("终点世界坐标: (%.1f, %.1f)" % goal_world)

    ref_path = build_straight_ref((pose.x_, pose.y_), goal_world)
    stanley = control.Stanley(k=1)
    pid = control.PID(Kp=0.1, Ki=0.0, Kd=0.0)

    initial_time = time.time()
    prev_time = time.time()
    dt = 0.05
    passed = 0  # 已通过的路口数

    print("\n开始直行 ...\n")
    while True:
        pose = ugv.get_transform()
        imu = ugv.get_imu_data()
        state = struct.State(pose, abs(imu.v), imu.a)

        # 统计已通过的路口
        while passed < len(east) and pose.x_ > east[passed] * scale + world_offset[0]:
            passed += 1
            print(">>> 已过第 %d 个路口 (x=%.1f)" % (passed, east[passed - 1] * scale + world_offset[0]))

        dist2goal = utils.distance(pose.x_, pose.y_, goal_world[0], goal_world[1])
        if dist2goal < GOAL_STOP_DIST or (time.time() - initial_time) > MAX_RUNTIME:
            ugv.apply_control(0, 0, 0.8, False)
            time.sleep(1)
            ugv.apply_control(0, 0, 0, True)
            print("\n停车。已通过路口 %d 个, 距终点 %.1f m, 位置 (%.1f, %.1f)"
                  % (passed, dist2goal, pose.x_, pose.y_))
            break

        target_pose = calculate_target_pose(pose, ref_path)
        delta = stanley.update_control(target_pose, state)
        accel = pid.update_control(TARGET_V, state.v_, dt)

        steer = delta / (math.pi / 6)
        if accel > 0:
            throttle, brake = min(accel, 1.0), 0.0
        else:
            throttle, brake = 0.0, min(abs(accel) * 0.5, 1.0)

        ugv.apply_control(steer, throttle, brake, False)

        if int(time.time() * 2) % 4 == 0:
            print("  pos=(%.1f, %.1f) v=%.1f 已过%d个路口 距终点 %.0f m"
                  % (pose.x_, pose.y_, state.v_, passed, dist2goal))

        dt = time.time() - prev_time
        prev_time = time.time()
        time.sleep(0.05)


if __name__ == '__main__':
    main()
