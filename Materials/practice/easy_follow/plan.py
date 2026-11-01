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

    ob_1 = ugvs[0].detect_fun()
    ugvs[0].set_weapon(ob_1,0)
    ugvs[0].set_weapon(ob_1,1)
    ugvs[0].set_weapon(ob_1,2)
    ugvs[0].set_weapon(ob_1,3)
    ugvs[0].set_weapon(ob_1,4)
    print(ob_1.x_ue)
    print(ob_1.Type)

