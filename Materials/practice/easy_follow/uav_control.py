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

    # 无人机待机
    if len(uavs) > 0:
        while True:
            all_ready = True  
            
            for uav in uavs:
                uav_state = uav.get_state()
                uav.target_point_control(uav_state.pose_.x_, uav_state.pose_.y_, 15, 100)
                
                if uav_state.pose_.z_ <= 14:
                    all_ready = False
                    print(f"{uav.name_} is not ready! Current height: {uav_state.pose_.z_}")

            if all_ready:
                print("All UAVs are ready!")
                break
            
            time.sleep(1)  
    
    # 无人机移动控制
    uav1_state = uavs[0].get_state()
    uav2_state = uavs[1].get_state()
    uav3_state = uavs[2].get_state()

    uavs[0].target_point_control(uav1_state.pose_.x_ + 25, uav1_state.pose_.y_, uav1_state.pose_.z_, 100)
    uavs[1].target_point_control(uav2_state.pose_.x_ , uav2_state.pose_.y_, uav2_state.pose_.z_ + 25, 100)
    uavs[2].target_point_control(uav3_state.pose_.x_ - 25, uav3_state.pose_.y_, uav3_state.pose_.z_, 100)