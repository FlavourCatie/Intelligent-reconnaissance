'''
用于存储坐标信息的结构体
'''
class Transform:
    def __init__(self, x: float, y: float, theta: float, z=0):
        self.x_ = x
        self.y_ = y
        self.theta_ = theta
        self.z_ = z
        
'''
用于存储静态障碍物信息
'''
class StaticObstacle:
    def __init__(self, pose: Transform, radius: float):
        self.pose_   = pose
        self.radius_ = radius
        
'''
用于存储 Frenet 坐标系下的车辆状态信息
'''
class FrenetState:
    def __init__(self, s, s_dot, s_ddot, d, d_dot, d_ddot):
        self.s_      = s
        self.s_dot_  = s_dot
        self.s_ddot_ = s_ddot
        self.d_      = d
        self.d_dot_  = d_dot
        self.d_ddot_ = d_ddot

'''
用于存储车辆状态信息的结构体
'''
class State:
    def __init__(self, pose: Transform, v: float, a: float, kappa=0):
        self.pose_ = pose
        self.v_    = v
        self.a_    = a
        self.kappa_ = kappa

"""
存储速度信息的结构体
"""
class ImuData:
    def __init__(self, vx, vy, vz, v, ax, ay, az, a, g, rate_pitch, rate_roll, rate_yaw):
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.v  = v
        self.ax = ax
        self.ay = ay
        self.az = az
        self.a  = a
        self.g  = g
        self.rate_pitch = rate_pitch
        self.rate_roll  = rate_roll
        self.rate_yaw   = rate_yaw

class DetStruct:

    """
    存储探测信息的结构体

    """
    def __init__(self,Type, Health, x, y, z, position, yaw, x_ue, y_ue):
        self.Type=Type
        self.Health=Health
        self.x=x
        self.y=y
        self.z=z
        self.position=position
        self.yaw=yaw
        self.x_ue=x_ue
        self.y_ue=y_ue
        self.radius=[]
    def calculate_collision_radius(self):
            ##增加障碍物碰撞体积
        for i in self.Type:
            if i == 'BP_BarricadeHard':
                self.radius.append(15.14/2)
            elif i == 'Barricade_BP':
                self.radius.append(15.14/2)
            elif i == 'Stryker':
                self.radius.append(6.8/2)
            else :
                self.radius.append(0)
