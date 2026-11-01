import math
from . import data_struct as struct
from . import utils as utils

'''
创建 PID 控制器类
'''
class PID:
    previous_error_ = 0 # 上一个控制周期的误差
    accum_error_    = 0 # 累计误差
    
    def __init__(self, Kp, Ki, Kd):
        self.Kp_ = Kp # 初始化比例系数
        self.Ki_ = Ki # 初始化积分系数
        self.Kd_ = Kd # 初始化微分系数
        
    def update_control(self, target_v, current_v, dt):
        dt = max(dt, 1e-3)
        # 计算比例项
        error = target_v - current_v
        proportion_term = self.Kp_ * error
        
        # 计算积分项
        self.accum_error_ += error
        integral_term = self.Ki_ * self.accum_error_
        
        # 计算微分项
        derivative_term = self.Kd_ * (error - self.previous_error_) / dt
        
        # 计算控制量
        output = proportion_term + integral_term + derivative_term
        
        # 更新上一个控制周期的误差
        self.previous_error_ = error
            
        return output

'''
创建 Stanley 控制器类
'''
class Stanley:
    def __init__(self, k):
        self.k_ = k # 初始化预瞄距离增益参数
        
    def update_control(self, P1: struct.Transform, ego: struct.State):
        # 计算横向误差及控制量
        d_y = P1.y_ - ego.pose_.y_
        d_x = P1.x_ - ego.pose_.x_

        e_y = d_y * math.cos(P1.theta_) - d_x * math.sin(P1.theta_)
        
        delta_y = math.atan2(self.k_ * e_y, ego.v_ + 1e-3)
        
        # 计算航向误差及控制量
        theta_phi = utils.normalize_angle(
            utils.normalize_angle(P1.theta_) 
            - utils.normalize_angle(ego.pose_.theta_)
            ) 
        
        delta_theta = theta_phi

        # 计算最终的转向角度
        #delta = delta_theta + delta_y
        delta = delta_y * 0.2 + delta_theta * 0.3

        delta = delta * 0.45

        return delta 
    
    def StanleyControl(self, target_x, target_y, target_yaw, ego_x, ego_y, ego_yaw, ego_speed, target_speed):
        # k_y_ = 0.01 * ego_speed
        # 1.2
        # 0.091
        k_y_ = min(0.091, (1.215 / (ego_speed + 0.01)))
        # k_y_ = 0.02
        # print("k_y_ = " + str(k_y_))

        dy = target_y - ego_y
        dx = target_x - ego_x

        lat_error = dy * math.cos((target_yaw)) - dx * math.sin((target_yaw))

        # print(dy * math.cos(NormalizeAngle(target_yaw)))
        # print('\n\n')
        #  print(dx * math.sin(NormalizeAngle(target_yaw)))
        # lat_error = dy * math.cos(target_yaw) - dx * math.sin(target_yaw)
        # yaw_error = NormalizeAngle(NormalizeAngle(ego_yaw) - NormalizeAngle(target_yaw))

        yaw_jian = ego_yaw - target_yaw

        if yaw_jian > math.pi:
            yaw_error = yaw_jian - 2 * math.pi
        if yaw_jian <= math.pi and yaw_jian >= -math.pi:
            yaw_error = yaw_jian
        if yaw_jian < -math.pi:
            yaw_error = yaw_jian + 2 * math.pi

        #  print("lat_error: " + str(lat_error))
        # print("yaw_error: " + str(yaw_error))

        # 15.125
        # steer = - yaw_error * 0.37 + math.atan(k_y_ * lat_error / ego_speed)
        # steer = - yaw_error * 2 *  15.125 / ego_speed - NormalizeAngle(math.atan2(k_y_ * lat_error, max(ego_speed, 10)))
        if target_speed >= 0:
            steer = - yaw_error * 0.8 * min(0.9, 15.125 / (ego_speed + 0.01)) + math.atan2(k_y_ * lat_error,
                                                                                           max(ego_speed, 5))
        if target_speed < 0:
            steer = yaw_error * 0.8 * min(0.9, 15.125 / (ego_speed + 0.01)) + math.atan2(k_y_ * lat_error,
                                                                                         max(ego_speed, 5))

        if steer > math.pi / 6:
            steer = math.pi / 6
        elif steer < -math.pi / 6:
            steer = -math.pi / 6

        return steer, lat_error