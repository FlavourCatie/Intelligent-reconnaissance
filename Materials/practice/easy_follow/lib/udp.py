import socket
import time
import proto.init_pb2 as init_proto
import proto.flag_pb2 as flag_proto
import proto.task_pb2 as task_proto
import proto.vehicle_state_pb2 as vehicle_state_proto

def init_udp_receiver(host="127.0.0.1", port=5001):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    
    while(True):
        print("Waiting for init message...")
        
        # 接收数据
        data, addr = sock.recvfrom(4096)
        
        init_msg = init_proto.Init()
        init_msg.ParseFromString(data)
        
        return init_msg
    
def is_init(host="127.0.0.1", port = 5002):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    flag_msg = flag_proto.Flag()
    flag_msg.is_init = True
    sock.sendto(flag_msg.SerializeToString(), (host, port))
    print(f"Sent is_init message to {host}:{port}")
    
def task_receiver(host="127.0.0.1", port = 5003):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    
    while(True):
        is_init()

        data, addr = sock.recvfrom(65536)
        task_msg = task_proto.task()
        task_msg.ParseFromString(data)
        
        return task_msg
    
def state_pub(uavs, ugvs, host="127.0.0.1", port = 5004):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    state_msg = vehicle_state_proto.Update_state()
    
    # 暂时 任务一直显示执行中，
    state_msg.task_state = 1  # 任务状态：1 - 执行中
    
    state_msg.vehicle_num = len(uavs) + len(ugvs)
    
    for vehicle in uavs + ugvs:
        vehicle_msg = vehicle_state_proto.Vehicle()
        # vehicle_msg.id = vehicle.id
        vehicle_msg.id = 1

        # if(vehicle.type == "ugv"):
        vehicle_msg.type = "vehicle.imv.rb"
        # elif(vehicle.type == "uav"):
        #     vehicle_msg.type = "vehicle.hrt.m10"
        
        vehicle_msg.position.x = vehicle.get_state().pose_.x_
        vehicle_msg.position.y = vehicle.get_state().pose_.y_
        vehicle_msg.position.z = vehicle.get_state().pose_.z_
        
        vehicle_msg.speed = vehicle.get_state().v_
        
        vehicle_msg.yaw = 10
        
        # 暂时，所有车辆状态都设为1
        vehicle_msg.state = 1
        
        state_msg.vehicles.append(vehicle_msg)
        
    state_msg.timestamp = int(time.time() * 1000)  # 毫秒级时间戳
    
    sock.sendto(state_msg.SerializeToString(), (host, port))
    print(f"Sent State message to {host}:{port}")