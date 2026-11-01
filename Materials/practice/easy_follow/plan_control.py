import math
import numpy as np
import matplotlib.pyplot as plt
import time

# 仿真软件官方的工具包
from swarmae.SwarmAEClient import SwarmAEClient

# 自己封装的函数和接口
import lib.utils as utils
import lib.swarm_api as api
import lib.frenet_optimal_planner as fop
import lib.controller as control
import lib.data_struct as struct
import lib.global_planner as gp
import lib.image_process as imp
import lib.init as init
from lib.formation_calculate import FormationToolkit
# UDP

# 导入字体管理器，用于显示中文
from matplotlib.font_manager import FontProperties
def plot_helper(vehicle_pose, ref_path, vehicle_geometry,vehicle_pose_2, ref_path_2, vehicle_geometry_2, vehicle_pose_3, ref_path_3, vehicle_geometry_3, actual_x_list, actual_y_list, actual_x_list_2, actual_y_list_2,actual_x_list_3, actual_y_list_3):
    area = 15
    #area = 1500
    vehicle_shape = vehicle_geometry.get_vehicle_shape(vehicle_pose)
    vehicle_shape_2 = vehicle_geometry_2.get_vehicle_shape(vehicle_pose_2)
    vehicle_shape_3 = vehicle_geometry_3.get_vehicle_shape(vehicle_pose_3)

    plt.cla()
    plt.plot(ref_path.interp_x_, ref_path.interp_y_, color='black', label='reference path')
    plt.plot(vehicle_shape[:,0], vehicle_shape[:,1], color='orange', label='ego_vehicle')
    plt.plot(actual_x_list, actual_y_list, color='red', label='traveld path')

    plt.plot(ref_path_2.interp_x_, ref_path_2.interp_y_, color='black', label='reference path 2')
    plt.plot(vehicle_shape_2[:,0], vehicle_shape_2[:,1], color='pink', label='ego_vehicle 2')
    plt.plot(actual_x_list_2, actual_y_list_2, color='blue', label='traveld path 2')

    plt.plot(ref_path_3.interp_x_, ref_path_3.interp_y_, color='black', label='reference path 3')
    plt.plot(vehicle_shape_3[:,0], vehicle_shape_3[:,1], color='orange', label='ego_vehicle 3')
    plt.plot(actual_x_list_3, actual_y_list_3, color='red', label='traveld path 3')

    plt.axis('equal')
    plt.xlim([vehicle_pose.x_ - area, vehicle_pose.x_ + area])
    plt.ylim([vehicle_pose.y_ - area, vehicle_pose.y_ + area])
    plt.legend()
    plt.grid(True)
    plt.pause(0.01)

#计算预瞄点
def calculate_target_pose(vehicle_pose, ref_path):
    # 计算前瞄点位置
    front_x = vehicle_pose.x_ + 10 * math.cos(vehicle_pose.theta_)
    front_y = vehicle_pose.y_ + 10 * math.sin(vehicle_pose.theta_)

    # 计算与参考路径的距离
    dx = np.ravel(ref_path.interp_x_) - front_x
    dy = np.ravel(ref_path.interp_y_) - front_y
    dist = np.abs(dx) + np.abs(dy)

    # 找到最小距离对应的索引
    min_dist_idx = np.argmin(dist)  # 预瞄点

    # 获取目标点坐标和朝向
    target_x = ref_path.interp_x_[min_dist_idx, 0]
    target_y = ref_path.interp_y_[min_dist_idx, 0]
    target_theta = ref_path.interp_theta_[min_dist_idx, 0]

    # 返回目标点的变换
    return struct.Transform(target_x, target_y, target_theta)


if __name__ == '__main__':


    # 初始化小群api 地图等
    AeClient = SwarmAEClient(ue_ip="127.0.0.1", ue_port=2000)
    _, AeGame, _ = AeClient.get_game()
    AeGame.stage_start('reconnaissance_start')
    AeGame.stage_complete('reconnaissance_end')
    AeGame.stage_start('vau_reconnaissance_start')
    AeGame.stage_complete('vau_reconnaissance_end')

    ##获取地图，变换比例，偏移量
    img, scale, world_offset, _, _ = AeGame.get_road_network()
    print("scale = ", scale, "world_offset = ", world_offset)

    #骨架细化算法，返回骨架图
    img_skl = imp.skeleton_refinement(img)

    # 维护注册节点
    ugvs = []
    uavs = []
    #注册节点
    ugv_1 = api.Vehicle(AeClient, vehicle_name="UGV1", vehicle_no= 1, vehicle_type="ugv")
    ugvs.append(ugv_1)

    ugv_2 = api.Vehicle(AeClient, vehicle_name="UGV2", vehicle_no= 2, vehicle_type="ugv")
    ugvs.append(ugv_2)

    ugv_3 = api.Vehicle(AeClient, vehicle_name="UGV3", vehicle_no= 3, vehicle_type="ugv")
    ugvs.append(ugv_3)

    uav_1 = api.Vehicle(AeClient, vehicle_name="UAV1" , vehicle_no= 4 ,vehicle_type="uav")
    uavs.append(uav_1)

    uav_2 = api.Vehicle(AeClient, vehicle_name="UAV2" , vehicle_no= 5 ,vehicle_type="uav")
    uavs.append(uav_2)

    uav_3 = api.Vehicle(AeClient, vehicle_name="UAV3" , vehicle_no= 6 ,vehicle_type="uav")
    uavs.append(uav_3)


    #提取头车的坐标位置
    start_x_world  = ugvs[0].get_state().pose_.x_
    start_y_world  = ugvs[0].get_state().pose_.y_

    #ue世界坐标转换为像素坐标
    start_x = int(scale * (start_x_world - world_offset[0]))
    start_y = int(scale * (start_y_world - world_offset[1]))

    #设置终点坐标
    end_x = int(-1500 - world_offset[0])
    end_y = int(-217 - world_offset[1])

    #寻找到最近的白点
    start_x_skl, start_y_skl, end_x_skl, end_y_skl = imp.skeleton_point(img_skl, start_x, start_y, end_x, end_y)
    print("start",start_x,start_y)
    print("end",end_x,end_y)
    print("start_skl",start_x_skl,start_y_skl)
    print("end_skl",end_x_skl,end_y_skl)

    #进行路径规划
    a_star_planner = gp.AStar(start_x_skl, start_y_skl, end_x_skl, end_y_skl, img_skl)
    path = a_star_planner.update_planning()

    #绘制路线
    gp.plot_map_and_path(img_skl, path)

    path_array = np.array(path)

    #转换为世界坐标路径
    waypoint_x = path_array[:, 0] + world_offset[0]
    waypoint_y = path_array[:, 1] + world_offset[1]

    #转换为numpy格式，并进行了转置
    waypoint_x = np.mat(waypoint_x).T
    waypoint_y = np.mat(waypoint_y).T

    # 三次B样条拟合
    ref_path_1 = fop.ReferencePath(waypoint_x, waypoint_y, resolution=2)


    #初始化控制器，stanley pid
    stanley = control.Stanley(k=1)
    pid = control.PID(Kp=0.1, Ki=0.0, Kd=0.0)
    wheel_base = 2

    # 创建任务目标 -> 行驶到最后一个参考点
    goal_x = ref_path_1.interp_x_[-1, 0]
    goal_y = ref_path_1.interp_y_[-1, 0]

    #设置车辆的显示矩形
    vehicle_geometry_1 = fop.VehicleGeometry(l=3, w=2)
    vehicle_geometry_2 = fop.VehicleGeometry(l=3, w=2)
    vehicle_geometry_3 = fop.VehicleGeometry(l=3, w=2)

    # 创建记录数据的空数组
    actual_x_list_1 = []
    actual_y_list_1 = []
    actual_x_list_2 = []
    actual_y_list_2 = []
    actual_x_list_3 = []
    actual_y_list_3 = []

    # 初始化采样时间
    initial_time = time.time()
    prev_time = time.time()
    dt = 0.01


    while True:

        #更新车辆自身状态
        vehicle_pose_1 = ugvs[0].get_transform()
        vehicle_imu_1  = ugvs[0].get_imu_data()
        vehicle_state_1 = struct.State(vehicle_pose_1, abs(vehicle_imu_1.v), vehicle_imu_1.a)

        vehicle_pose_2 = ugvs[1].get_transform()
        vehicle_imu_2  = ugvs[1].get_imu_data()
        vehicle_state_2 = struct.State(vehicle_pose_2, abs(vehicle_imu_2.v), vehicle_imu_2.a)

        vehicle_pose_3 = ugvs[2].get_transform()
        vehicle_imu_3  = ugvs[2].get_imu_data()
        vehicle_state_3 = struct.State(vehicle_pose_3, abs(vehicle_imu_3.v), vehicle_imu_3.a)

        #到达终点，即停止
        if utils.distance(vehicle_pose_1.x_, vehicle_pose_1.y_, goal_x, goal_y) < 1:
            # 先踩 10 秒钟刹车
            ugvs[0].apply_control(steer=0, throttle=0, brake=0.8, hand_brake=False)
            ugvs[1].apply_control(steer=0, throttle=0, brake=0.8, hand_brake=False)
            ugvs[2].apply_control(steer=0, throttle=0, brake=0.8, hand_brake=False)
            time.sleep(10)

            # 然后拉手刹制动
            ugvs[0].apply_control(steer=0, throttle=0, brake=0, hand_brake=True)
            ugvs[1].apply_control(steer=0, throttle=0, brake=0, hand_brake=True)
            ugvs[2].apply_control(steer=0, throttle=0, brake=0, hand_brake=True)

            # 随后跳出循环
            break

        #记录轨迹数据
        actual_x_list_1.append(vehicle_pose_1.x_)
        actual_y_list_1.append(vehicle_pose_1.y_)

        actual_x_list_2.append(vehicle_pose_2.x_)
        actual_y_list_2.append(vehicle_pose_2.y_)

        actual_x_list_3.append(vehicle_pose_3.x_)
        actual_y_list_3.append(vehicle_pose_3.y_)

        #绘制图形化页面
        ref_path_2 = ref_path_1 #纯跟车可以公用路径
        ref_path_3 = ref_path_2
        plot_helper(vehicle_pose_1, ref_path_1, vehicle_geometry_1,vehicle_pose_2, ref_path_2, vehicle_geometry_2, vehicle_pose_3, ref_path_3, vehicle_geometry_3, actual_x_list_1, actual_y_list_1, actual_x_list_2, actual_y_list_2,actual_x_list_3, actual_y_list_3)

        #计算预瞄点
        # 计算每个车辆的目标点
        target_pose_1 = calculate_target_pose(vehicle_pose_1, ref_path_1)
        target_pose_2 = calculate_target_pose(vehicle_pose_2, ref_path_2)
        target_pose_3 = calculate_target_pose(vehicle_pose_3, ref_path_3)

        target_v_1 = 10
        s0 = 30 #希望保持的车距

        #计算目标速度
        if target_v_1 > 0.5 and time.time()-initial_time < 4:
            target_v_2 = target_v_1 + 0.1 * (math.sqrt((vehicle_pose_1.x_ - vehicle_pose_2.x_) * (vehicle_pose_1.x_ - vehicle_pose_2.x_) + (vehicle_pose_1.y_ - vehicle_pose_2.y_) * (vehicle_pose_1.y_ - vehicle_pose_2.y_)) - s0)
            target_v_3 = target_v_2 + 0.1 * (math.sqrt((vehicle_pose_2.x_ - vehicle_pose_3.x_) * (vehicle_pose_2.x_ - vehicle_pose_3.x_) + (vehicle_pose_2.y_ - vehicle_pose_3.y_) * (vehicle_pose_2.y_ - vehicle_pose_3.y_)) - s0)
            if target_v_2 < 0 :
                target_v_2 = 1
            if target_v_3 < 0 :
                target_v_3 = 1

        if target_v_1 > 0.5 and time.time()-initial_time >= 4:
            target_v_2 = target_v_1 + 0.1 * (math.sqrt((vehicle_pose_1.x_ - vehicle_pose_2.x_) * (vehicle_pose_1.x_ - vehicle_pose_2.x_) + (vehicle_pose_1.y_ - vehicle_pose_2.y_) * (vehicle_pose_1.y_ - vehicle_pose_2.y_)) - s0)
            target_v_3 = target_v_2 + 0.1 * (math.sqrt((vehicle_pose_2.x_ - vehicle_pose_3.x_) * (vehicle_pose_2.x_ - vehicle_pose_3.x_) + (vehicle_pose_2.y_ - vehicle_pose_3.y_) * (vehicle_pose_2.y_ - vehicle_pose_3.y_)) - s0)
            if target_v_2 < 0 :
                target_v_2 = 1
            if target_v_3 < 0 :
                target_v_3 = 1

        #计算角度和加速度
        delta_1 = stanley.update_control(target_pose_1, vehicle_state_1)
        accel_1 = pid.update_control(target_v_1, vehicle_state_1.v_, dt)

        delta_2 = stanley.update_control(target_pose_2, vehicle_state_2)
        accel_2 = pid.update_control(target_v_2, vehicle_state_2.v_, dt)

        delta_3 = stanley.update_control(target_pose_3, vehicle_state_3)
        accel_3 = pid.update_control(target_v_3, vehicle_state_3.v_, dt)

        #完成角度和转向角的映射，数值可以按需求调整，没有实际的物理意义
        steer_1 = delta_1 / (math.pi / 6)
        steer_2 = delta_2 / (math.pi / 6)
        steer_3 = delta_3 / (math.pi / 6)

        # 将加速度转换为油门、刹车
        if accel_1 > 0:
            throttle_1 = accel_1
            brake_1 = 0
        else:
            throttle = 0
            brake = abs(accel_1) * 0.5

        if accel_2 > 0:
            throttle_2 = accel_2
            brake_2 = 0
        else:
            throttle_2 = 0
            brake_2 = abs(accel_2) * 0.5

        if accel_3 > 0:
            throttle_3 = accel_3
            brake_3 = 0
        else:
            throttle_3 = 0
            brake_3 = abs(accel_3) * 0.5

        # 控制车辆运动
        ugvs[0].apply_control(steer_1, throttle_1, brake_1, hand_brake=False)
        ugvs[1].apply_control(steer_2, throttle_2, brake_2, hand_brake=False)
        ugvs[2].apply_control(steer_3, throttle_3, brake_3, hand_brake=False)

        #计算控制间隔
        dt = time.time() - prev_time
        prev_time = time.time()