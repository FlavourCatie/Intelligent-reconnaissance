from . import data_struct as struct

import time
import math
import numpy as np
import json

"""
用 Vehicle 来统一管理无人车无人机 SwarmAENode 对象
Vehicle类封装了无人机和无人车节点 车和机的控制和位置获取是不同的 _check_type_函数会检查运用的接口是否合法
"""
class Vehicle:
    def __init__(self, Client, vehicle_name, vehicle_no, vehicle_type):
        # 设置 Client
        self.Client_ = Client
        
        # 定义车辆基本信息
        self.type_ = vehicle_type  # 节点类型
        self.name_ = vehicle_name  # 节点名称
        self.no_   = vehicle_no    # 节点在队伍中的编号
        
        if(vehicle_type == "ugv"):
            register_type = "四轮车"
        elif(vehicle_type == "uav"):
            register_type = "四旋翼"
        
        # 创建车辆节点
        timestamp = int(round(time.time() * 1000))
        _, self.Vehicle_, _ = self.Client_.register_node(node_type = register_type, 
                                                         node_name = self.name_,
                                                         node_no   = self.no_  ,
                                                         frame_timestamp = timestamp) 
        
        if(vehicle_type == "ugv"):   
            # 创建车辆节点并获取game类weapon类 
            _, self.game_, _ = self.Client_.get_game()
            _,self.weapon_,_,_=self.Vehicle_.get_weapon()
            
            # 读取车辆尺寸
            vehicle_size  = self.Vehicle_.get_size() # 获取车辆尺寸参数
            self.width_   = vehicle_size['y']        # 车辆宽度
            self.length_  = vehicle_size['x']        # 车辆长度
            self.height_  = vehicle_size['z']        # 车辆高度
            
            # 读取车辆颜色
            vehicle_rgb = self.Vehicle_.get_color()  # 读取车辆颜色参数
            self.color_ = np.mat([vehicle_rgb['r'],  # 车辆颜色的 [r, g, b] 值 
                                vehicle_rgb['g'], 
                                vehicle_rgb['b']])
            
            # 读取车辆的其他属性
            _, _, team, _, model, _ = self.Vehicle_.get_node_info()
            self.team_  = team  # 车辆编队归属（分为红/蓝两队） 
            self.model_ = model # 车辆型号
            self.id_    = self.Vehicle_.get_node_id()  # 车辆唯一标识 id

        # #读取地图比例及偏差
        # img,scale,world_offset,_,_=self.game_.get_road_network()
        # self.scale=scale
        # self.world_offset=world_offset
        # self.img=img
    
    def get_imu_data(self):
        vx, vy, vz, v, ax, ay, az, a, g, p, q, r, _,  = self.Vehicle_.get_velocity()
        vehicle_imu = struct.ImuData(vx, vy, vz, v, ax, ay, az, a, g, p, q, r)
        return vehicle_imu  
    
    def get_transform(self) -> struct.Transform:
        yaw, _, _, _, _ = self.Vehicle_.get_attitude()
        x, y, z, _ = self.Vehicle_.get_location()
        vehicle_pose = struct.Transform(x, y, yaw*math.pi/180, z)
        return vehicle_pose
    
    # 基本获取车辆状态函数，返回一个 State 对象
    def get_state(self) -> struct.State:  
        pose = self.get_transform()
        imu = self.get_imu_data()
        state = struct.State(pose, imu.v, imu.a, imu.rate_yaw)
        return state
    
    def apply_control(self, steer, throttle, brake, hand_brake):
        self._check_type("ugv", "此方法仅限地面车辆使用")
        # 油门为正是前进，油门为负是倒车
        throttle = min(throttle,  1)
        throttle = max(throttle, -1)

        # 无论前进还是倒车，刹车都为正
        brake = min(brake, 1)
        brake = max(brake, 0)

        # 转向的取值范围是 [-1, 1]
        steer = min(steer,  1)
        steer = max(steer, -1)

        # 调用接口控制车辆
        # 挡位设为 0 即可，不用调整
        _, status_code = self.Vehicle_.apply_vehicle_control(throttle, steer, brake, hand_brake, 0)

        return status_code
    
    def detect_fun(self):
        # self._check_type("ugv", "此方法仅限地面车辆使用")
        #number代表第几个障碍物
         #初始化
        t=0
        code=0
        global res
        res=[]
        infomation=[]
        Number=[]
        Type=[]
        x=[]
        y=[]
        z=[]
        position=[]
        yaw=[]
        Health=[]
        x_ue=[]
        y_ue=[]
        fire_flag = []
        #接入sdk
        # _,scale,world_offset,_,_=self.game_.get_road_network()
        t,code,res = self.Vehicle_.detect_situation()
        res=json.loads(res)
        #障碍物完整信息
        infomation=res
        #探测到障碍物个数
        Number=len(res)

        for i in res:
            #类型
            Type.append(i['Type_Name'])
            #血量
            # Health.append(i['HealthPoint'])
            #中心坐标  UE坐标  单位M
            x1=i['Position']['X']/100
            y1=i['Position']['Y']/100
            x_ue.append(x1)
            y_ue.append(y1)
            #变换到像素坐标
            x1=int(x1+2095)
            y1=int(y1+2637)
            x.append(x1)
            y.append(y1)
            z.append(i['Position']['Z']/100)
            position.append([x1,y1])  
            #障碍物航向角
            yaw.append(i['Rotation']['Yaw'])  
             
            #障碍物生命值
            if i.get("HealthPoint") is None:
                Health.append(0)
            else:
                Health.append(i['HealthPoint'])
            
        Detdata = struct.DetStruct(Type, Health, x, y, z, position, yaw ,x_ue ,y_ue)
        return Detdata
    
    def set_weapon(self , ob, fight_num):
        self._check_type("ugv", "此方法仅限地面车辆使用")
        # 读取武器接口
        _, ammo, frame_timestamp = self.weapon_.get_weapon_ammo()

        # 弹药数量
        zidan = int(ammo.x)
        paodan = int(ammo.y)
        fantanke = int(ammo.z)

        # 血量
        # target_health = item['HealthPoint']
        # 名称
        target_name = ob.Type[fight_num]

        # 弹药类型 0-子弹（95发）（伤害20）  1-40炮（15发）（伤害1600） 2-反坦克炮（2发）（伤害3000）
        if (target_name == 'Barricade_BP' or target_name =='Pillbox'): # 随机目标血量4200，一般是简易工事，旁边还会有士兵一起出现
            print('------识别到障碍物，进入开火打击--------')
            _, ammo, frame_timestamp = self.weapon_.get_weapon_ammo()
            self.weapon_.set_weapon_status(3, 1, ob.x_ue[fight_num], ob.y_ue[fight_num], 1.5)
            time.sleep(0.9)

        elif target_name == 'Stryker' and ob.Health[fight_num] > 0 : # 车辆，血量6000，需要判断是否剩余反坦克炮弹，否则调用40炮
            print('------识别到障碍物，进入开火打击--------')
            _, ammo, frame_timestamp = self.weapon_.get_weapon_ammo()

            self.weapon_.set_weapon_status(4, 1, ob.x_ue[fight_num], ob.y_ue[fight_num], 1.5)
            time.sleep(1.5)         

        elif target_name == 'Building' : # 有人员建筑目标，血量4800，用40炮
            print('------识别到障碍物，进入开火打击--------')
            _, ammo, frame_timestamp = self.weapon_.get_weapon_ammo()

            self.weapon_.set_weapon_status(3, 1, ob.x_ue[fight_num], ob.y_ue[fight_num], 1.5)
            time.sleep(0.8)

        elif target_name == 'Soldier' : # 士兵血量100，用子弹5发
            print('------识别到障碍物，进入开火打击--------')
            _, ammo, frame_timestamp = self.weapon_.get_weapon_ammo()

            # self.weapon_.set_weapon_status(6, 0, ob.x_ue[fight_num], ob.y_ue[fight_num], 1.5)
            self.weapon_.set_weapon_status(6, 0, ob.x_ue[fight_num], ob.y_ue[fight_num], ob.z[fight_num]+0.1)
            time.sleep(0.8)
        return  0
    
    def target_point_control(self, target_x, target_y, target_z, target_v):
        self._check_type("uav", "此方法仅限空中无人机使用")
        timestamp = int(round(time.time() * 1000))
        self.Vehicle_.control_kinetic_simply_global(target_x, target_y, target_z, target_v, timestamp)
        
    def _check_type(self, allowed_type, error_msg):
        """检查当前类型是否匹配允许的方法调用"""
        if self.type_ != allowed_type:
            raise PermissionError(f"{error_msg}（当前类型: {self.type_})")
        
        