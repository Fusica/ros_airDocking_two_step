from model_predict import *
from params import *

from std_msgs.msg import Float64MultiArray, Bool, Int32
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from mavros_msgs.msg import State, AttitudeTarget

import rospy


def body2worldVel(rot,vel):
    r_now = rot.flatten()
    local_vel = np.zeros(3)
    x = vel[0]
    y = vel[1]
    z = vel[2]
    local_vel[0] = r_now[0] * x + r_now[1] * y + r_now[2] * z
    local_vel[1] = r_now[3] * x + r_now[4] * y + r_now[5] * z
    local_vel[2] = r_now[6] * x + r_now[7] * y + r_now[8] * z
    return local_vel


# 获取旋转矩阵
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


# 获取当前实时位置姿态
current_pose = PoseStamped()

def current_pose_cb(data):
    global current_pose
    
    current_pose = data
    
    
current_state = State()

def state_cb(data):
    global current_state
    current_state = data
    
# 获取速度

current_vel = TwistStamped()

def current_vel_cb(data):
    
    global current_vel
    
    current_vel = data
    
# 获取目标位置姿态

nominal_pose = PoseStamped()

# def nominal_pose_cb(data):
#     global nominal_pose
    
#     nominal_pose = data
    

    

if __name__ == '__main__':
    
    rospy.init_node('control_node')    
    rate = rospy.Rate(100) 
    
    # nominal_pose_sub = rospy.Subscriber("/nominal_position", PoseStamped, nominal_pose_cb, queue_size=1)
    current_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, current_pose_cb, queue_size=1)
    current_vel_sub = rospy.Subscriber("/mavros/local_position/velocity_body", TwistStamped, current_vel_cb, queue_size=1)
    
    state_sub = rospy.Subscriber('/mavros/state',State,callback=state_cb, queue_size=1)

    
    # px4_command_pub = rospy.Publisher("/des_attitude", AttitudeTarget, queue_size=1)
    state_error_pub = rospy.Publisher("/state_error", Float64MultiArray, queue_size=1)
    nominal_pos_enu_pub = rospy.Publisher("/nominal_pos_enu", PoseStamped, queue_size=1)
    
    
    attitude_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
    
    
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo('waiting for connection...')
        rate.sleep()
    
    first_sub = True
    # 发送指令
    
    while not rospy.is_shutdown():
        
        if current_state.mode == 'OFFBOARD':
            rospy.loginfo('OFFBOARD')
            
            if first_sub == True:
                nominal_pose.pose.position.x = current_pose.pose.position.x
                nominal_pose.pose.position.y = current_pose.pose.position.y
                nominal_pose.pose.position.z = current_pose.pose.position.z
                
                nominal_pose.pose.orientation.w = current_pose.pose.orientation.w
                nominal_pose.pose.orientation.x = current_pose.pose.orientation.x
                nominal_pose.pose.orientation.y = current_pose.pose.orientation.y
                nominal_pose.pose.orientation.z = current_pose.pose.orientation.z
                
                first_sub = False
                
        else:	
                rospy.loginfo('NOT OFFBOARD')
                nominal_pose.pose.position.x = current_pose.pose.position.x
                nominal_pose.pose.position.y = current_pose.pose.position.y
                nominal_pose.pose.position.z = current_pose.pose.position.z
                
                nominal_pose.pose.orientation.w = current_pose.pose.orientation.w
                nominal_pose.pose.orientation.x = current_pose.pose.orientation.x
                nominal_pose.pose.orientation.y = current_pose.pose.orientation.y
                nominal_pose.pose.orientation.z = current_pose.pose.orientation.z
                
        
        #nominal_pos_enu.x = nominal_pose.pose.position.x
        #nominal_pos_enu.y = nominal_pose.pose.position.y
        #nominal_pos_enu.z = nominal_pose.pose.position.z
        
        nominal_pos_enu_pub.publish(nominal_pose)
            
        
        
        # 获取当前位置误差
        # 这个都按照mavros坐标系来
        position_error = np.array(
            [
                current_pose.pose.position.x - nominal_pose.pose.position.x,
                current_pose.pose.position.y - nominal_pose.pose.position.y,
                current_pose.pose.position.z - nominal_pose.pose.position.z,
            ]
        ) # 在mavros下需要转换
        
        # 获取姿态矩阵
        current_quat = np.array([current_pose.pose.orientation.w,current_pose.pose.orientation.x,current_pose.pose.orientation.y,current_pose.pose.orientation.z])
        current_rot = quat2rot_change(current_quat)
        
        nominal_quat = np.array([nominal_pose.pose.orientation.w,nominal_pose.pose.orientation.x,nominal_pose.pose.orientation.y,nominal_pose.pose.orientation.z])
        nominal_rot = quat2rot_change(nominal_quat)
        
        
        error_rot = current_rot@np.linalg.inv(nominal_rot)
        
        error_rot = error_rot.flatten()
        
        c_vel = np.array([current_vel.twist.linear.x,
                    current_vel.twist.linear.y,
                    current_vel.twist.linear.z])
        
        
        # 这个地方要升级
        error_vel = body2worldVel(
            current_rot,
            c_vel
        )
        
        error_omega = np.array([current_vel.twist.angular.x,
                    current_vel.twist.angular.y,
                    current_vel.twist.angular.z])
        
        error_position = np.array([
            position_error[1],
            -position_error[0],
            position_error[2]
        ])
        
        state_error = np.array(
            [
                error_position[0], error_position[1], error_position[2],
                error_vel[0],error_vel[1],error_vel[2],
                error_rot[0], error_rot[1], error_rot[2],
                error_rot[3],error_rot[4],error_rot[5],
                error_rot[6], error_rot[7],error_rot[8],
                0,0,0,
                
            ]
        )
        
        
        state_error_pub.publish(data=state_error)
        
        action = modelPredict(state_error)
        
        px4_command.body_rate.x = action[1]
        px4_command.body_rate.y = action[2] 
        px4_command.body_rate.z = action[3]
        px4_command.thrust = (action[0] + 1) / 1.93 * 2 * 1.0 * 0.33 #0.50 #0.50
        attitude_pub.publish(px4_command)
        
        rospy.loginfo('thrust is {}'.format(px4_command.thrust))
        
        rate.sleep()
