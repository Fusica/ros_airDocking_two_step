import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np


quadrotor_pose = PoseStamped()

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


def quadrotor_pose_callback(state):
    global quadrotor_pose
    quadrotor_pose = state

if __name__ == '__main__':
    
    rospy.init_node('virtural')
    rate = rospy.Rate(100)
    
    child1_p = np.array([0.44,0.01887965,-0.21177904])
    
    
    quadrotor_pose_sub = rospy.Subscriber('/child1/mavros/local_position/pose', PoseStamped, quadrotor_pose_callback, queue_size=1)
    
    end_pose_pub = rospy.Publisher('/vrpn_client_node1/tong/pose', PoseStamped, queue_size=1)
    
    end_pose = PoseStamped()
    
    
    while not rospy.is_shutdown():
        
        quad_pos = np.array([quadrotor_pose.pose.position.x, quadrotor_pose.pose.position.y, quadrotor_pose.pose.position.z])
        
        quad_quat = np.array([quadrotor_pose.pose.orientation.w, quadrotor_pose.pose.orientation.x, quadrotor_pose.pose.orientation.y, quadrotor_pose.pose.orientation.z])
        
        quad_rot = quat2rot_change(quad_quat)
        
        end_pos = quad_pos + np.dot(quad_rot, child1_p)
        
        end_pose.pose.position.x = end_pos[0]
        
        end_pose.pose.position.y = end_pos[1]
        
        end_pose.pose.position.z = end_pos[2]
        
        end_pose_pub.publish(end_pose)
        
        rate.sleep()
