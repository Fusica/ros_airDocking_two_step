import rospy
from mavros_msgs.msg import State # 获取飞控状态
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import AttitudeTarget, PositionTarget # 发送飞控控制指令
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, Int32
import numpy as np
import math
# from tf import transformations as tft
import time

# 一号机的home位置为世界坐标系原点

a = 6378137
b = 6356752.3142
f = (a - b) / a
e_sq = f * (2-f)
pi = 3.14159265359

def w84_to_ecef(lat, lon, h):
    # (lat, lon) in degrees
    # h in meters
    lamb = math.radians(lat)
    phi = math.radians(lon)
    s = math.sin(lamb)
    N = a / math.sqrt(1 - e_sq * s * s)
 
    sin_lambda = math.sin(lamb)
    cos_lambda = math.cos(lamb)
    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)
 
    x = (h + N) * cos_lambda * cos_phi
    y = (h + N) * cos_lambda * sin_phi
    z = (h + (1 - e_sq) * N) * sin_lambda
    
    pos = np.array([x,y,z])
 
    return pos


# 定义子机名称
quadrotor_name1 = 'child1'
quadrotor_name2 = 'child2'

#控制模式
control_mode = 0.

# 定义子机回传变量
child1_state = State()
child2_state = State()

child1_cmd = AttitudeTarget()
child2_cmd = AttitudeTarget()

child1_pose = PoseStamped()
child2_pose = PoseStamped()

child1_nominal_pid_pos = PoseStamped()
child2_nominal_pid_pos = PoseStamped()

global force_flag

force_flag = False

# 定义回传函数
def child1_state_callback(msg):
    global child1_state
    child1_state = msg
    
def child2_state_callback(msg):
    global child2_state
    child2_state = msg
    
def child1_pid_cmd_callback(msg):
    global child1_pid_cmd
    child1_pid_cmd = msg
    
def child2_pid_cmd_callback(msg):
    global child2_pid_cmd
    child2_pid_cmd = msg
    
def child2_rl_cmd_callback(msg):
    global child2_rl_cmd
    child2_rl_cmd = msg
    
def child1_pose_callback(msg):
    global child1_pose
    child1_pose = msg
    
def child2_pose_callback(msg):
    global child2_pose
    child2_pose = msg


def child1_GPS_callback(msg):
    global child1_GPS 
    child1_GPS = msg
    
def child2_GPS_callback(msg):
    global child2_GPS 
    child2_GPS = msg
    

    
# 通用功能类

# 悬停状态检测

class HoverCheck():
    def __init__(self, pose, vel, buffer_size=200):
        self.pose = pose
        self.vel = vel
        self.hover_flag = False
        
        self.buffer_size = buffer_size
        
        self.pose_buffer = np.zeros((buffer_size, 3))
        self.vel_buffer = np.zeros((buffer_size, 3))
        
        self.index = 0
        self.total_steps = 0
        
    def add_buffer(self, pose):
        
        self.pose_buffer[self.index, :] = pose
        #self.vel_buffer[self.index, :] = vel
        
        self.index = self.index + 1
        
        if self.index == self.buffer_size:
            self.index = 0
            
        self.total_steps = self.total_steps + 1
    
    def check(self, tolerance_variance_pos, minimun_measeure_size=500):
        
        if self.total_steps < minimun_measeure_size:
            return False
        
        std = np.std(self.pose_buffer)
        
        # sum_vel = np.sum(self.vel_buffer)
        rospy.loginfo("check std is {}".format(std))
        if std < tolerance_variance_pos :
            
            return True
        
        else:
            return False
        
    def reset(self):
        self.index = 0
        self.total_steps = 0
        self.pose_buffer = np.zeros((self.buffer_size, 3))
        self.vel_buffer = np.zeros((self.buffer_size, 3))
        
        
        
        
if __name__ == '__main__':
    
    ############################## 初始化 ################################
    # 1. 初始化节点
    # 2. 定义发布者
    # 3. 定义发布变量
    # 4. 定义订阅者
    # 5. 判断PX4是否连接成功
    ######################################################################
    rospy.init_node('transmitter')
    rate = rospy.Rate(100) # 取决于数传速度越快越好
    
    # 定义状态回传订阅者
    child1_state_sub = rospy.Subscriber(quadrotor_name1 + '/mavros/state',State,callback=child1_state_callback, queue_size=1)
    child2_state_sub = rospy.Subscriber(quadrotor_name2 + '/mavros/state',State,callback=child2_state_callback, queue_size=1)
    
    child1_pose_sub = rospy.Subscriber(quadrotor_name1 + '/mavros/local_position/pose',PoseStamped,callback=child1_pose_callback, queue_size=1)
    child2_pose_sub = rospy.Subscriber(quadrotor_name2 + '/mavros/local_position/pose',PoseStamped,callback=child2_pose_callback, queue_size=1)
    
    child1_GPS_sub =  rospy.Subscriber(quadrotor_name1 + '/mavros/global_position/global', NavSatFix,callback=child1_GPS_callback, queue_size=100)
    child2_GPS_sub =  rospy.Subscriber(quadrotor_name2 + '/mavros/global_position/global', NavSatFix,callback=child2_GPS_callback, queue_size=100)
    
    # 定义控制指令定订阅者,pid控制器
    child1_pid_cmd_sub = rospy.Subscriber('pid_cmd1', AttitudeTarget,callback=child1_pid_cmd_callback ,queue_size=1)
    child2_pid_cmd_sub = rospy.Subscriber('pid_cmd2', AttitudeTarget,callback=child2_pid_cmd_callback ,queue_size=1)
    
    # 定义控制指令定订阅者,rl控制器
    child2_rl_cmd_sub = rospy.Subscriber('rl_cmd2', AttitudeTarget,callback=child2_rl_cmd_callback ,queue_size=1)
    
    # 定义发布变量，并且mask掉不需要的控制量（姿态）
    child1_tartget_attitude = AttitudeTarget()
    child1_tartget_attitude.type_mask = 128
    child2_tartget_attitude = AttitudeTarget()
    child2_tartget_attitude.type_mask = 128
    
    child1_tartget_postion = PositionTarget()
    child1_tartget_postion.type_mask = 64+128+256+512+2048
    
    child1_tartget_postion.coordinate_frame = 1
    child1_tartget_postion.yaw = 0
    # child1_tartget_postion.position.x = 0
    # child1_tartget_postion.position.y = 0
    # child1_tartget_postion.position.z = 0
    
    
    child2_tartget_postion = PositionTarget()
    
    child2_tartget_postion.type_mask = 64+128+256+512+2048
    
    child2_tartget_postion.coordinate_frame = 1
    child2_tartget_postion.yaw = 0
    
    child1_tartget_vel = TwistStamped()
    child2_tartget_vel = TwistStamped()
    
    child1_tartget_vel.twist.angular.x = 0
    child1_tartget_vel.twist.angular.y = 0
    child1_tartget_vel.twist.angular.z = 0
    child2_tartget_vel.twist.linear.x = 0
    child2_tartget_vel.twist.linear.y = 0
    child2_tartget_vel.twist.linear.z = 0
    
    child2_tartget_vel.twist.angular.x = 0
    child2_tartget_vel.twist.angular.y = 0
    child2_tartget_vel.twist.angular.z = 0
    child2_tartget_vel.twist.linear.x = 0
    child2_tartget_vel.twist.linear.y = 0
    child2_tartget_vel.twist.linear.z = 0
    
    # 定义发布者
    child1_attitude_pub = rospy.Publisher(quadrotor_name1 + '/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
    child2_attitude_pub = rospy.Publisher(quadrotor_name2 + '/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
    
    child1_tartget_postion_pub = rospy.Publisher(quadrotor_name1 + '/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    child2_tartget_postion_pub = rospy.Publisher(quadrotor_name2 + '/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    
    virture_child1_tartget_postion_pub = rospy.Publisher('virture_child1_pos', PositionTarget, queue_size=1)

    child2_fixed_vec_pub = rospy.Publisher('fixed_vec', PoseStamped, queue_size=10)
    
    
    #debug
    far_away_flag_pub = rospy.Publisher('/far_away_flag', Int32, queue_size=1)
    
    
    # 俩架子机发送目标位置
    child1_nominal_pid_pub = rospy.Publisher('nominal_pid_pos1', PoseStamped, queue_size=1)
    child2_nominal_pid_pub = rospy.Publisher('nominal_pid_pos2', PoseStamped, queue_size=1)
    
    #control_mode
    control_mode_pub = rospy.Publisher('/control_mode', Float32, queue_size=1)
    
    # 功能插件
    
    # 子机二悬停检测
    child2_hover_check = HoverCheck(pose=np.zeros(3), vel=np.zeros(3), buffer_size=400)
    
    child2_hover_check2 = HoverCheck(pose=np.zeros(3), vel=np.zeros(3), buffer_size=400)
    
    # 判断PX4是否连接成功
    while not rospy.is_shutdown() and not child1_state.connected and not child2_state.connected:
        rospy.loginfo('waiting for connection...')
        rate.sleep()
    time.sleep(5)
    ##############################预处理阶段###########################
    # 1.获取一段时间俩架子机GPS LLA
    # 2.取这段时间的平均值
    # 3. 这样拿出来的数据不是很准确用气压计，换成GPS在尝试
    ##################################################################
    quad1_gpses = []
    quad2_gpses = []
    for _ in range(200):
        rate.sleep()
        gps1 = np.array([child1_GPS.latitude,child1_GPS.longitude,child1_GPS.altitude])
        gps2 = np.array([child2_GPS.latitude,child2_GPS.longitude,child2_GPS.altitude])
        
        quad1_gpses.append(gps1)
        quad2_gpses.append(gps2)
        
        rospy.loginfo('get original gps')
        
        
        
    quad1_gpses = np.vstack(quad1_gpses)
    quad2_gpses = np.vstack(quad2_gpses)
    
    
   
    mean_gps1 = np.mean(quad1_gpses, axis=0)
    mean_gps2 = np.mean(quad2_gpses, axis=0)
    print("quadrotor1 GPS position is {}".format(mean_gps1))
    print("quadrotor2 GPS position is {}".format(mean_gps2)) 
    # 转化成ecef
    ecef_pos1 = w84_to_ecef(mean_gps1[0],mean_gps1[1],mean_gps1[2])
    ecef_pos2 = w84_to_ecef(mean_gps2[0],mean_gps2[1],mean_gps2[2])
    
    print("quadrotor1 ecef position is {}".format(ecef_pos1))
    print("quadrotor2 ecef position is {}".format(ecef_pos2))
    
    # 一号机为坐标原点计算出二号机在一号机所在的位置
    delta_pos = ecef_pos2 - ecef_pos1
    
    print("orient vector is {}".format(delta_pos))
    
    mean_gps1 = mean_gps1 * 180.0 / math.pi
    # print(mean_gps1)
    S_matrix = np.array([
        [-math.sin(mean_gps1[1]), math.cos(mean_gps1[1]),0],
        [-math.sin(mean_gps1[0])*math.cos(mean_gps1[1]), -math.sin(mean_gps1[0])*math.sin(mean_gps1[1]), math.cos(mean_gps1[0])],
        [math.cos(mean_gps1[0])*math.cos(mean_gps1[1]),math.cos(mean_gps1[0])*math.sin(mean_gps1[1]), math.sin(mean_gps1[0])]
    ])
    
    enu_distance = S_matrix@delta_pos # 所有的直接相加即可，当作是一个向量
    
    
    fixed_vec = PoseStamped()
    
    fixed_vec.pose.position.x = enu_distance[0]
    fixed_vec.pose.position.y = enu_distance[1]
    fixed_vec.pose.position.z = enu_distance[2]
    
    time.sleep(10)
    
    
    # 虚拟一号机位置以二号机为原点高1.5m位置上
    
    
        
    
    ############################## 功能实现区 #########################
    # 1. 读取子机状态,通过子机状态判断对接任务阶段
    # 2. 当2号机处于offboard模式时，1号机处于悬停模式，2号机进去轨迹规划模式
    # 3. 当1号机和2号机都处于offboard模式时，进入对接控制器
    ######################################################################
    
    stage1_hover_flag = False
    
    child1_first_hover_flag = False
    
    first_stage2_terminal_flag = False
    
    far_away_flag = False
    
    while not rospy.is_shutdown():
        
        quadrotor1_mode = child1_state.mode
        quadrotor2_mode = child2_state.mode
        
        child2_fixed_vec_pub.publish(fixed_vec)
        
        child2_pose.pose.position.x = child2_pose.pose.position.x + fixed_vec.pose.position.x
        child2_pose.pose.position.y = child2_pose.pose.position.y + fixed_vec.pose.position.y
        child2_pose.pose.position.z = child2_pose.pose.position.z + fixed_vec.pose.position.z
        
        # 判断1号机是否进入offboard模式
        if quadrotor2_mode == 'OFFBOARD':
            if quadrotor1_mode != 'OFFBOARD':
                # 进入避障模式
                
                # 悬停检测
                child2_hover_check.add_buffer(pose=np.array([child2_pose.pose.position.x, child2_pose.pose.position.y, child2_pose.pose.position.z]))
                
                child2_hover_flag = child2_hover_check.check(tolerance_variance_pos=0.9) # 悬停检测 0.7
                
                if not stage1_hover_flag:
                    child1_postion_world_frame = np.array([child1_pose.pose.position.x, child1_pose.pose.position.y, child1_pose.pose.position.z])
                        
                    child2_target_position_world_frame = child1_postion_world_frame + np.array([1.5, 0, 0.])

                
                
                if child2_hover_flag :
                    stage1_hover_flag = True
                    
                if stage1_hover_flag:
                    
                    child2_tartget_postion.position.x = child2_target_position_world_frame[0]
                    child2_tartget_postion.position.y = child2_target_position_world_frame[1]
                    child2_tartget_postion.position.z = child2_target_position_world_frame[2]
                    
                    # 偏航
                    child2_tartget_postion.yaw = 1.570796
                    
                    child2_tartget_postion.type_mask = 64+128+256+512+2048+8+16+32
                    
                    child2_tartget_postion_pub.publish(child2_tartget_postion)
                    
                    rospy.loginfo(" stage1_hover  hover at target position: {}".format(child2_target_position_world_frame))
                    print(f"child2_target_position_world_frame is {child2_target_position_world_frame}")
                    print(f"====================================================")
                else:
                    rospy.loginfo("no hover UAV2 fly to target position: {}".format(child2_target_position_world_frame))
                    child2_attitude_pub.publish(child2_rl_cmd)
                    
                    

                
                # 悬停模式
                # 强制一号无人机悬停当前位置
                child1_tartget_postion.position.x = child1_postion_world_frame[0]
                child1_tartget_postion.position.y = child1_postion_world_frame[1]
                child1_tartget_postion.position.z = child1_postion_world_frame[2]
                
                # 偏航
                child1_tartget_postion.yaw = 1.570796
                child1_tartget_postion.type_mask = 64+128+256+512+2048+8+16+32
                
                child1_tartget_postion_pub.publish(child1_tartget_postion)
                control_mode = 1.0
                #control_mode_pub.publish(data=1.0) # rl
            else:
                # 进入对接模式
                ############################################################
                # 阶段一：二号无人机朝向一号无人机，一号无人机悬停
                # 对接装置在x轴，2号无人机沿着x轴正方向着对接装置飞行
                # 保持y,z轴不动
                ############################################################
                
                if not child1_first_hover_flag:
                    # 记录1号无人机的位置
                    #child1_postion_world_frame = np.array([child1_pose.pose.position.x, child1_pose.pose.position.y, child1_pose.pose.position.z])
                    
                    two_x = child1_postion_world_frame[0] + 1.5
                    child1_first_hover_flag = True
                    
                    # child1_nominal_pid_pos.pose.position.x = 
                    
                    child2_target_position_world_frame = child1_postion_world_frame + np.array([1.1, 0, 0])
                    
                    
                    
               
                
                # 强制一号无人机悬停当前位置
                child1_tartget_postion.position.x = child1_postion_world_frame[0]
                child1_tartget_postion.position.y = child1_postion_world_frame[1]
                child1_tartget_postion.position.z = child1_postion_world_frame[2]
                
                # 偏航
                child1_tartget_postion.yaw = 1.570796
                child1_tartget_postion.type_mask = 64+128+256+512+2048+8+16+32
                # child2_tartget_postion.type_mask = 64+128+256+512+2048+8+16+32
                
                # 套筒位置
                # child1_postion_world_frame = np.array([child1_pose.pose.position.x, child1_pose.pose.position.y, child1_pose.pose.position.z])
                    
                #child1_tong_position_world_frame = child1_postion_world_frame + np.array([0.44, 0, 0])
                
                # 获取姿态四元数
                pose1 = child1_pose.pose.orientation
                
                # euler1 = tft.quaternion_matrix
                
                # R1 = tft.quaternion_matrix([pose1.w,pose1.x,pose1.y,pose1.z])
                
                child1_tong_position_world_frame =  np.array([child1_pose.pose.position.x, child1_pose.pose.position.y, child1_pose.pose.position.z]) + np.array([0.44, 0, 0])
                
                # 强制二号无人机沿着x轴负方向飞行，位置控制，缓慢接近
                
                
                pose2 = child2_pose.pose.orientation
                
                # R2 = tft.quaternion_matrix([pose2.w,pose2.x,pose2.y,pose2.z])
                
                # 计算当前桶间位置
                child2_jian_position_world_frame = np.array([child2_pose.pose.position.x, child2_pose.pose.position.y, child2_pose.pose.position.z]) - np.array([0.38, 0, 0])
                print(f"桶尖位置 是：{child2_jian_position_world_frame}")
                # 如果tong和jian交叉距离大于0.1m，那么远离或者切换控制器
                
                yz_distance = np.linalg.norm(child2_jian_position_world_frame[1:3] - child1_tong_position_world_frame[1:3])
                print(f"yz_distance is {yz_distance}")
                tong_x = child1_tong_position_world_frame[0]
                jian_x = child2_jian_position_world_frame[0]
                
                # if yz_distance < 0.05:
                #     if tong_x - jian_x > 0.03:
                #         far_away_flag = True
                
                if not far_away_flag:
                    # 获取1，2号无人机的当前位置作为pid控制器的悬停位置
                    child1_nominal_pid_pos.pose.position.x = child1_pose.pose.position.x
                    child1_nominal_pid_pos.pose.position.y = child1_pose.pose.position.y
                    child1_nominal_pid_pos.pose.position.z = child1_pose.pose.position.z
                    #print(f"uav1 pid target is {child1_nominal_pid_pos}")
                            
                    child2_nominal_pid_pos.pose.position.x = child2_pose.pose.position.x
                    child2_nominal_pid_pos.pose.position.y = child2_pose.pose.position.y
                    child2_nominal_pid_pos.pose.position.z = child2_pose.pose.position.z
                    #print(f"uav2 pid target is {child2_nominal_pid_pos}")
                            
                if yz_distance < 0.05:
                    #pdb.set_trace()
                    if jian_x - tong_x < 0.5: #0.15
                        #pdb.set_trace()
                        far_away_flag = True
                        far_away_flag_ = 1
                        far_away_flag_pub.publish(data=far_away_flag_)
                        
        
                
                # 设置速度
                if far_away_flag:
                    two_x -= 0.1/100 # 1号无人机的速度 m/s
                    two_x = max(two_x, child1_pose.pose.position.x + 0.78)
                    rospy.loginfo("far away")
                else:
                    #vel_x -= 0.01
                    two_x -= 0.1 /100
                    two_x = max(two_x, child1_pose.pose.position.x + 0.9)
                    rospy.loginfo("close")
                    
                    print(f"child2_pose x is {child2_pose.pose.position.x}")
                    print(f"child2 nominal x is {two_x}")
                    
                    
                
                 # 悬停检测
                child2_hover_check2.add_buffer(pose=np.array([child2_pose.pose.position.x, child2_pose.pose.position.y, child2_pose.pose.position.z]))
                child2_hover_flag = child2_hover_check2.check(tolerance_variance_pos=0.7)
                
                
                next_child2_target_position_world_frame = np.array([two_x, child1_pose.pose.position.y, child1_pose.pose.position.z]) #+ np.array([vel_x, 0, 0]) / 100
                

                        
                child2_tartget_postion.position.x = next_child2_target_position_world_frame[0]
                child2_tartget_postion.position.y = next_child2_target_position_world_frame[1]
                child2_tartget_postion.position.z = next_child2_target_position_world_frame[2]
                
                child2_tartget_postion.yaw = 1.570796
                    
                child2_tartget_postion.type_mask = 64+128+256+512+2048+8+16+32
                
                
                
                
               
                
                
                # 发送控制指令
                
                # 1号无人机悬停
                """
                if far_away_flag:
                    
                    if not child2_hover_flag and not first_stage2_terminal_flag:
                        # 2号无人机悬停
                        
                        child2_tartget_postion.position.x = child2_target_position_world_frame[0]
                        child2_tartget_postion.position.y = child2_target_position_world_frame[1]
                        child2_tartget_postion.position.z = child2_target_position_world_frame[2]
                        
                        # 偏航
                        child2_tartget_postion.yaw = 1.570796
                        
                        child2_tartget_postion.type_mask = 64+128+256+512+2048+8+16+32
                        child2_tartget_postion_pub.publish(child2_tartget_postion)
                        child1_tartget_postion_pub.publish(child1_tartget_postion)
                        
                        child2_nominal_pid_pos.pose.position.x = child2_target_position_world_frame[0]
                        child2_nominal_pid_pos.pose.position.y = child2_target_position_world_frame[1]
                        child2_nominal_pid_pos.pose.position.z = child2_target_position_world_frame[2]
                        child2_nominal_pid_pub.publish(child2_nominal_pid_pos)
                        
                        
                        child1_nominal_pid_pos.pose.position.x = child1_postion_world_frame[0]
                        child1_nominal_pid_pos.pose.position.y = child1_postion_world_frame[1]
                        child1_nominal_pid_pos.pose.position.z = child1_postion_world_frame[2]
                        
                        child1_nominal_pid_pub.publish(child1_nominal_pid_pos)
                        
                        
                        print(f"mavros悬停判定")
                        print(f"/n")
                    else:
                        # 双机悬停
                         # 发布位置
                        
                            #pdb.set_trace()
                        first_stage2_terminal_flag = True
                        child1_nominal_pid_pub.publish(child1_nominal_pid_pos)
                        child2_nominal_pid_pub.publish(child2_nominal_pid_pos)
                        child1_attitude_pub.publish(child1_pid_cmd)
                        child2_attitude_pub.publish(child2_pid_cmd)
                        print(f"action is {child2_pid_cmd}")
                        control_mode = 3.0
                        print(f"进入pid")
                        
                    
                    #control_mode_pub.publish(data=3.0) #pid
                    
                else:
                    # if child2_hover_flag:
                    #     # 1号无人机悬停
                    #     child1_tartget_postion_pub.publish(child1_tartget_postion)
                    #     child2_attitude_pub.publish(child2_pid_cmd)
                    # else:
                     # 发布位置
                    child1_nominal_pid_pub.publish(child1_nominal_pid_pos)
                    child2_nominal_pid_pub.publish(child2_nominal_pid_pos)
                    child1_tartget_postion_pub.publish(child1_tartget_postion)
                    child2_tartget_postion_pub.publish(child2_tartget_postion)
                    control_mode = 2.0
                    #control_mode_pub.publish(data=2.0) # mavros
                """
                child1_nominal_pid_pub.publish(child1_nominal_pid_pos)
                child2_nominal_pid_pub.publish(child2_nominal_pid_pos)
                child1_tartget_postion_pub.publish(child1_tartget_postion)
                child2_tartget_postion_pub.publish(child2_tartget_postion)
                control_mode = 2.0
                               
               
        else:
            # 提前发送消息方便进入offboard模式
            child1_tartget_postion.velocity.x = 0
            child1_tartget_postion.velocity.y = 0
            child1_tartget_postion.velocity.z = 0
            child1_tartget_postion.yaw = 1.570796
            child1_tartget_postion.type_mask = 8+16+32+64+128+256+512+2048
            
            child2_tartget_postion.velocity.x = 0
            child2_tartget_postion.velocity.y = 0
            child2_tartget_postion.velocity.z = 0
            child2_tartget_postion.yaw = 1.570796
            child2_tartget_postion.type_mask = 8+16+32+64+128+256+512+2048
            
            child1_tartget_postion_pub.publish(child1_tartget_postion)
            child2_tartget_postion_pub.publish(child2_tartget_postion)
            
            rospy.loginfo('waiting for offboard mode...')
        print(f"control_mode is {control_mode}")
        control_mode_pub.publish(data=control_mode)    
        rate.sleep()
    