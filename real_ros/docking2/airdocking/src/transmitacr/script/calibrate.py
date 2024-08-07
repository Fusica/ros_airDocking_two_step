import rospy
from mavros_msgs.msg import State # 获取飞控状态
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import AttitudeTarget, PositionTarget # 发送飞控控制指令
from std_msgs.msg import Float32, Int32
import numpy as np


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

##################
# 校准轴线在机体坐标系下的位置
##################

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


quadrotor_state = State()
end_pose = PoseStamped()
quadrotor_pose = PoseStamped()

def quadrotor_state_callback(state):
    global quadrotor_state
    quadrotor_state = state
    
def end_pose_callback(state):
    global end_pose
    end_pose = state
    
def quadrotor_pose_callback(state):
    global quadrotor_pose
    quadrotor_pose = state
    

    
if __name__ == '__main__':
    
    rospy.init_node('calibrate')
    
    rate = rospy.Rate(50)
    
    quadrotor_state_sub = rospy.Subscriber('/mavros/state', State, quadrotor_state_callback, queue_size=1)
    quadrotor_pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, quadrotor_pose_callback, queue_size=1)
    
    end_pose_sub = rospy.Subscriber('/vrpn_client_node/tong/pose', PoseStamped, end_pose_callback, queue_size=1)
    
    quadrotor_target_pose_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    
    
    target_postion = PositionTarget()
    
    target_postion.coordinate_frame = 1
    
    while not rospy.is_shutdown() and not quadrotor_state.connected:
        rospy.loginfo('connecting...')
        rate.sleep()
        
        
    hover_check = HoverCheck(pose=np.zeros(3), vel=np.zeros(3), buffer_size=400)
    
    
    total_size = 200
    
    temp_pos = np.zeros(shape=(total_size,3))
    
    index = 0
        
    while not rospy.is_shutdown():
        
        current_postion = quadrotor_pose.pose.position
        
        
         # 获取端点坐标
        end_postion = np.array([end_pose.pose.position.x, end_pose.pose.position.y, end_pose.pose.position.z])
            
        # 获取无人机坐标
        quad_postion = np.array([quadrotor_pose.pose.position.x, quadrotor_pose.pose.position.y, quadrotor_pose.pose.position.z])
        
        hover_check.add_buffer(quad_postion)
        
        if quadrotor_state.mode != 'OFFBOARD':
            
            target_postion.velocity.x = 0
            target_postion.velocity.y = 0
            target_postion.velocity.z = 0
            
            target_postion.yaw = 1.570796
            

            
            target_postion.type_mask = 8+16+32+64+128+256+512+2048
            
            quadrotor_target_pose_pub.publish(target_postion)
            
            target = np.array([current_postion.x, current_postion.y, current_postion.z])
            
            # rospy.loginfo('waiting for offboard mode...')
        
        else:
            
           
            
            # 获取无人机姿态
            quad_quat = np.array([quadrotor_pose.pose.orientation.w, quadrotor_pose.pose.orientation.x, quadrotor_pose.pose.orientation.y, quadrotor_pose.pose.orientation.z])
            rot = quat2rot_change(quad_quat)
            
            # 获取端点在无人机坐标系下的坐标
            end_body = np.linalg.inv(rot)@(end_postion - quad_postion)
            
            
            
            target_postion.position.x = target[0]
            target_postion.position.y = target[1]
            target_postion.position.z = target[2]
            
            target_postion.yaw = 1.570796
            target_postion.type_mask = 64+128+256+512+2048+8+16+32
            
            quadrotor_target_pose_pub.publish(target_postion)
            
            index += 1
            
            flag = hover_check.check(tolerance_variance_pos=0.03)
            
            if flag:
                
                temp_pos[index%total_size] = end_body
            
            if index % total_size:
                # mean = np.mean(end_bodys, axis=0)
                
                # print('body position: {}'.format(mean))
                
                mean = np.mean(temp_pos, axis=0)
                
                print('body position accurate: {}'.format(mean))
            
        rate.sleep()