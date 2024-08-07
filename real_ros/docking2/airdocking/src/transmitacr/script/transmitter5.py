import rospy
from mavros_msgs.msg import State # 获取飞控状态
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import AttitudeTarget, PositionTarget # 发送飞控控制指令
from std_msgs.msg import Float32, Int32
import numpy as np
import pdb


def quat2rot_change(quat):
    # global r_now
    
    r_now = np.zeros(9)
    w = quat[0]
    x = quat[1]
    y = quat[2] 
    z = quat[3]
    r_now[0] = 2*x*y + 2*z*w
    r_now[1] = 1.0 - 2*x*x - 2*z*z
    r_now[2] = 2*y*z - 2*x*w
    r_now[3] = 2*y*y + 2*z*z -1.0
    r_now[4] = 2*z*w - 2*x*y
    r_now[5] = -2*x*z - 2*y*w
    r_now[6] = 2*x*z - 2*y*w
    r_now[7] = 2*y*z + 2*x*w
    r_now[8] = 1.0 - 2*x*x -2*y*y
    
    return r_now.reshape(3,3)


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

tong_pose = PoseStamped()

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
    
    
def tong_pose_callback(msg):
    global tong_pose
    tong_pose = msg
    
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
        
        std = np.std(self.pose_buffer, axis=0)
        
        std_normm = np.linalg.norm(std)
        
        # sum_vel = np.sum(self.vel_buffer)
        rospy.loginfo("check std is {}".format(std_normm))
        if std_normm < tolerance_variance_pos :
            
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
    
    # 需要测量，端点在机体坐标系位置
    # TODO: 机体坐标系位置
    child1_p = np.array([0.44,0.01887965,-0.21177904])  # 注意考虑轴的半径
    child2_p = np.array([-0.40,0.02540905,-0.22631377])
    
    # 定义状态回传订阅者
    child1_state_sub = rospy.Subscriber(quadrotor_name1 + '/mavros/state',State,callback=child1_state_callback, queue_size=1)
    child2_state_sub = rospy.Subscriber(quadrotor_name2 + '/mavros/state',State,callback=child2_state_callback, queue_size=1)
    
    child1_pose_sub = rospy.Subscriber(quadrotor_name1 + '/mavros/local_position/pose',PoseStamped,callback=child1_pose_callback, queue_size=1)
    child2_pose_sub = rospy.Subscriber(quadrotor_name2 + '/mavros/local_position/pose',PoseStamped,callback=child2_pose_callback, queue_size=1)
    
    # tong_pose_sub = rospy.Subscriber('/vrpn_client_node1/tong/pose', PoseStamped, tong_pose_callback, queue_size=1)
    # 定义控制指令定订阅者,pid控制器
    # child1_pid_cmd_sub = rospy.Subscriber('pid_cmd1', AttitudeTarget,callback=child1_pid_cmd_callback ,queue_size=1)
    # child2_pid_cmd_sub = rospy.Subscriber('pid_cmd2', AttitudeTarget,callback=child2_pid_cmd_callback ,queue_size=1)
    
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
    
    RL_nominal_pos = PoseStamped()
    
    
    
    # 定义发布者
    child1_attitude_pub = rospy.Publisher(quadrotor_name1 + '/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
    child2_attitude_pub = rospy.Publisher(quadrotor_name2 + '/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
    
    child1_tartget_postion_pub = rospy.Publisher(quadrotor_name1 + '/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    child2_tartget_postion_pub = rospy.Publisher(quadrotor_name2 + '/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    
    RL_nominal_pos_pub = rospy.Publisher('/RL_nominal_pos', PoseStamped, queue_size=1)
    RL_nominal_pos = PoseStamped()
    
    child1_tong_pub = rospy.Publisher('tong_pos', PositionTarget, queue_size=1)
    child2_jian_pub = rospy.Publisher('jian_pos', PositionTarget, queue_size=1)
    
    tong_global_pos = PositionTarget()
    jian_global_pos = PositionTarget()
    
    #debug
    far_away_flag_pub = rospy.Publisher('/far_away_flag', Int32, queue_size=1)
    
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
    
    ############################## 功能实现区 #########################
    # 1. 读取子机状态,通过子机状态判断对接任务阶段
    # 2. 当2号机处于offboard模式时，1号机处于悬停模式，2号机进去轨迹规划模式
    # 3. 当1号机和2号机都处于offboard模式时，进入对接控制器
    ######################################################################
    
    stage1_hover_flag = False
    
    child1_first_hover_flag = False
    
    first_stage2_terminal_flag = False
    
    far_away_flag = False
    
    distance = 0.3
    
    while not rospy.is_shutdown():
        
        quadrotor1_mode = child1_state.mode
        quadrotor2_mode = child2_state.mode
        
        # 实际上支取yz方向
        # 直接用动捕的信息
        
        pos_child1 = np.array([child1_pose.pose.position.x, child1_pose.pose.position.y, child1_pose.pose.position.z])
        pos_child2 = np.array([child2_pose.pose.position.x, child2_pose.pose.position.y, child2_pose.pose.position.z])
        # child1_axis_position_world_frame = np.array([tong_pose.pose.position.x, tong_pose.pose.position.y, tong_pose.pose.position.z])
        # TODO: 校准杆长数据
        

                    
        child12_expected_position_world_frame = pos_child1 + np.array([1.2, 0.01289,0.00487])
                    
        print('expected pos:{}'.format(child12_expected_position_world_frame))
        
        RL_nominal_pos.pose.position.x = child12_expected_position_world_frame[0]
        RL_nominal_pos.pose.position.y = child12_expected_position_world_frame[1]
        RL_nominal_pos.pose.position.z = child12_expected_position_world_frame[2]
        
        RL_nominal_pos_pub.publish(RL_nominal_pos)
        
        # TODO: RL节点信息发送的更正。
        
        # 判断1号机是否进入offboard模式
        if quadrotor2_mode == 'OFFBOARD':
            if quadrotor1_mode != 'OFFBOARD':
                # 进入避障模式
                
                # 悬停检测
                child2_hover_check.add_buffer(pose=np.array([child2_pose.pose.position.x, child2_pose.pose.position.y, child2_pose.pose.position.z]))
                
                child2_hover_flag = child2_hover_check.check(tolerance_variance_pos=0.65) # 悬停检测 0.7 # TODO:重新测定
                
                if not stage1_hover_flag:
                    child1_postion_world_frame = np.array([child1_pose.pose.position.x, child1_pose.pose.position.y, child1_pose.pose.position.z])
                        
                    child2_target_position_world_frame = np.array([child2_pose.pose.position.x, child12_expected_position_world_frame[1], child12_expected_position_world_frame[2]])

                
                
                if child2_hover_flag :
                    stage1_hover_flag = True
                    
                if stage1_hover_flag:

                    child2_tartget_postion.position.x = child12_expected_position_world_frame[0]  # TODO: check
                    child2_tartget_postion.position.y = child12_expected_position_world_frame[1]
                    child2_tartget_postion.position.z = child12_expected_position_world_frame[2]
                    
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
                    
                    two_x = child2_target_position_world_frame[0]
                    child1_first_hover_flag = True
                    
                    # child1_nominal_pid_pos.pose.position.x = 
                    
                    # child2_target_position_world_frame = child1_postion_world_frame + np.array([1.1, 0, 0]) # 这是啥
                    
                    
                
                # 强制一号无人机悬停当前位置
                child1_tartget_postion.position.x = child1_postion_world_frame[0]
                child1_tartget_postion.position.y = child1_postion_world_frame[1]
                child1_tartget_postion.position.z = child1_postion_world_frame[2]
                
                # 偏航
                child1_tartget_postion.yaw = 1.570796
                child1_tartget_postion.type_mask = 64+128+256+512+2048+8+16+32
                
                
                # print(f"桶尖位置 是：{child2_jian_position_world_frame}")
                # 如果tong和jian交叉距离大于0.1m，那么远离或者切换控制器
                
                yz_distance = np.linalg.norm(pos_child2[1:3] + np.array([0.01276, 0]) - pos_child1[1:3])
                print(f"yz_distance is {yz_distance}")
                
                
                # if not far_away_flag:
                #     # 获取1，2号无人机的当前位置作为pid控制器的悬停位置
                #     child1_nominal_pid_pos.pose.position.x = child1_pose.pose.position.x
                #     child1_nominal_pid_pos.pose.position.y = child1_pose.pose.position.y
                #     child1_nominal_pid_pos.pose.position.z = child1_pose.pose.position.z
                #     #print(f"uav1 pid target is {child1_nominal_pid_pos}")
                            
                #     child2_nominal_pid_pos.pose.position.x = child2_pose.pose.position.x
                #     child2_nominal_pid_pos.pose.position.y = child2_pose.pose.position.y
                #     child2_nominal_pid_pos.pose.position.z = child2_pose.pose.position.z
                    #print(f"uav2 pid target is {child2_nominal_pid_pos}")
                            
                if yz_distance < 0.05:
                    #pdb.set_trace()
                    if pos_child2[0] - pos_child1[0] < 0.87: #0.15
                        #pdb.set_trace()
                        far_away_flag = True
                        far_away_flag_ = 1
                        far_away_flag_pub.publish(data=far_away_flag_)
                        
        
                
                #TODO:优化为尖端位置吧，这个太抽象了
                # 设置速度
                if far_away_flag:
                    two_x -= 0.1/100 # 1号无人机的速度 m/s
                    two_x = max(two_x, child1_pose.pose.position.x + 0.71521)
                    rospy.loginfo("far away")
                    control_mode = 3.0
                else:
                    #vel_x -= 0.01
                    two_x -= 0.1 /100
                    two_x = max(two_x, child1_pose.pose.position.x + 0.87)  # 数值修改
                    rospy.loginfo("close")
                    
                    print(f"child2_pose x is {child2_pose.pose.position.x}")
                    print(f"child2 nominal x is {two_x}")
                    control_mode = 2.0
                    
                    
                
                 # 悬停检测
                child2_hover_check2.add_buffer(pose=np.array([child2_pose.pose.position.x, child2_pose.pose.position.y, child2_pose.pose.position.z]))
                child2_hover_flag = child2_hover_check2.check(tolerance_variance_pos=0.08)
                
                
                next_child2_target_position_world_frame = np.array([two_x, child12_expected_position_world_frame[1], child12_expected_position_world_frame[2]]) #+ np.array([vel_x, 0, 0]) / 100
                

                        
                child2_tartget_postion.position.x = next_child2_target_position_world_frame[0]
                child2_tartget_postion.position.y = next_child2_target_position_world_frame[1]
                child2_tartget_postion.position.z = next_child2_target_position_world_frame[2]
                
                child2_tartget_postion.yaw = 1.570796
                    
                child2_tartget_postion.type_mask = 64+128+256+512+2048+8+16+32
                
                
            
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
        print(f"control_mode is {control_mode}")
        control_mode_pub.publish(data=control_mode)    
        rate.sleep()
    
