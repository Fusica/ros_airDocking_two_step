import rospy
from mavros_msgs.msg import State # 获取飞控状态
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import AttitudeTarget, PositionTarget # 发送飞控控制指令

import numpy as np


# 定义子机名称
quadrotor_name1 = 'child1'
quadrotor_name2 = 'child2'

# 定义子机回传变量
child1_state = State()
child2_state = State()

child1_cmd = AttitudeTarget()
child2_cmd = AttitudeTarget()

child1_pose = PoseStamped()
child2_pose = PoseStamped()

global force_flag

force_flag = False

# 定义回传函数
def child1_state_callback(msg):
    global child1_state
    child1_state = msg
    
def child2_state_callback(msg):
    global child2_state
    child2_state = msg
    
def child1_cmd_callback(msg):
    global child1_cmd
    child1_cmd = msg
    
def child2_cmd_callback(msg):
    global child2_cmd
    child2_cmd = msg
    
def child1_pose_callback(msg):
    global child1_pose
    child1_pose = msg
    
def child2_pose_callback(msg):
    global child2_pose
    child2_pose = msg
    
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
    
    # 定义控制指令定订阅者
    child1_cmd_sub = rospy.Subscriber('cmd1', AttitudeTarget,callback=child2_cmd_callback ,queue_size=1)
    child2_cmd_sub = rospy.Subscriber('cmd2', AttitudeTarget,callback=child2_cmd_callback ,queue_size=1)
    
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
    
    child1_vel_pub = rospy.Publisher(quadrotor_name1 + '/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
    child2_vel_pub = rospy.Publisher(quadrotor_name2 + '/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
    
    # 功能插件
    
    # 子机二悬停检测
    child2_hover_check = HoverCheck(pose=np.zeros(3), vel=np.zeros(3), buffer_size=400)
    
    
    # 判断PX4是否连接成功
    while not rospy.is_shutdown() and not child1_state.connected and not child2_state.connected:
        rospy.loginfo('waiting for connection...')
        rate.sleep()
    
    ############################## 功能实现区 #########################
    # 1. 读取子机状态,通过子机状态判断对接任务阶段
    # 2. 当2号机处于offboard模式时，1号机处于悬停模式，2号机进去轨迹规划模式
    # 3. 当1号机和2号机都处于offboard模式时，进入对接控制器
    ######################################################################
    
    stage1_hover_flag = False
    
    child1_first_hover_flag = False
    
    far_away_flag = False
    
    while not rospy.is_shutdown():
        
        quadrotor1_mode = child1_state.mode
        quadrotor2_mode = child2_state.mode
        
        # 判断1号机是否进入offboard模式
        if quadrotor2_mode == 'OFFBOARD':
            if quadrotor1_mode != 'OFFBOARD':
                # 进入避障模式
                
                # 悬停检测
                child2_hover_check.add_buffer(pose=np.array([child2_pose.pose.position.x, child2_pose.pose.position.y, child2_pose.pose.position.z]))
                
                child2_hover_flag = child2_hover_check.check(tolerance_variance_pos=0.7)
                
                child1_postion_world_frame = np.array([child1_pose.pose.position.x, child1_pose.pose.position.y, child1_pose.pose.position.z])
                    
                child2_target_position_world_frame = child1_postion_world_frame + np.array([1.0, 0, 0])
                
        
                
                
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
                else:
                    rospy.loginfo("no hover UAV2 fly to target position: {}".format(child2_target_position_world_frame))
                    child2_attitude_pub.publish(child2_cmd)
                    

                
                # 悬停模式
                # child2_vel_pub.publish(child2_tartget_vel)
                child1_tartget_postion.velocity.x = 0
                child1_tartget_postion.velocity.y = 0
                child1_tartget_postion.velocity.z = 0
                child1_tartget_postion.yaw = 0
                child1_tartget_postion.type_mask = 8+16+32+64+128+256+512+2048
                
                child1_tartget_postion_pub.publish(child1_tartget_postion)
            else:
                # 进入对接模式
                ############################################################
                # 阶段一：二号无人机朝向一号无人机，一号无人机悬停
                # 对接装置在x轴，2号无人机沿着x轴正方向着对接装置飞行
                # 保持y,z轴不动
                ############################################################
                
                if not child1_first_hover_flag:
                    # 记录1号无人机的位置
                    child1_postion_world_frame = np.array([child1_pose.pose.position.x, child1_pose.pose.position.y, child1_pose.pose.position.z])
                    
                    two_x = child1_postion_world_frame[0] + 1.0
                    child1_first_hover_flag = True
                # 强制一号无人机悬停当前位置
                child1_tartget_postion.position.x = child1_postion_world_frame[0]
                child1_tartget_postion.position.y = child1_postion_world_frame[1]
                child1_tartget_postion.position.z = child1_postion_world_frame[2]
                
                # 偏航
                child1_tartget_postion.yaw = 1.570796
                child1_tartget_postion.type_mask = 64+128+256+512+2048+8+16+32
                # child2_tartget_postion.type_mask = 64+128+256+512+2048+8+16+32
                
                # 套筒位置
                child1_tong_position_world_frame = child1_postion_world_frame + np.array([0.44, 0, 0])
                
                # 强制二号无人机沿着x轴负方向飞行，位置控制，缓慢接近
                
                
                # 计算当前桶间位置
                child2_jian_position_world_frame = np.array([child2_pose.pose.position.x, child2_pose.pose.position.y, child2_pose.pose.position.z]) - np.array([0.38, 0, 0])
                print(f"桶尖位置 是：{child2_jian_position_world_frame}")
                # 如果tong和jian交叉距离大于0.1m，那么远离或者切换控制器
                
                yz_distance = np.linalg.norm(child2_jian_position_world_frame[1:3] - child1_tong_position_world_frame[1:3])
                
                tong_x = child1_tong_position_world_frame[0]
                jian_x = child2_jian_position_world_frame[0]
                
                if yz_distance < 0.05:
                    if tong_x - jian_x > 0.03:
                        far_away_flag = True
                
                # 设置速度
                if far_away_flag:
                    two_x += 0.02/100 # 1号无人机的速度 m/s
                    rospy.loginfo("far away")
                else:
                    #vel_x -= 0.01
                    two_x -= 0.02 /100
                    rospy.loginfo("close")
                    
                    print(f"child2_pose x is {child2_pose.pose.position.x}")
                    print(f"child2 nominal x is {two_x}")
                    
                    
                

                
                   #child2_pose.pose.position.x
                
                
                next_child2_target_position_world_frame = np.array([two_x, child1_postion_world_frame[1], child1_postion_world_frame[2]]) #+ np.array([vel_x, 0, 0]) / 100
                

                        
                child2_tartget_postion.position.x = next_child2_target_position_world_frame[0]
                child2_tartget_postion.position.y = next_child2_target_position_world_frame[1]
                child2_tartget_postion.position.z = next_child2_target_position_world_frame[2]
                
                child2_tartget_postion.yaw = 1.570796
                    
                child2_tartget_postion.type_mask = 64+128+256+512+2048+8+16+32
                
                
                # 发送控制指令
                
                # 1号无人机悬停
                child1_tartget_postion_pub.publish(child1_tartget_postion)
                child2_tartget_postion_pub.publish(child2_tartget_postion)
                
                               
               
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
            
        rate.sleep()
                
        
        
    
    
    