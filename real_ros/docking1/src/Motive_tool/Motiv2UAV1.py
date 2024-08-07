import numpy as np
import rospy
import math
from geometry_msgs.msg import PoseStamped
from math import sin, cos
# import tf

# import math
 
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        roll_x = math.degrees(roll_x)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        pitch_y = math.degrees(pitch_y)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        yaw_z = math.degrees(yaw_z)
     
        return roll_x, pitch_y, yaw_z # in radians

# def rpy2quaternion(roll, pitch, yaw):
#     x=sin(pitch/2)*sin(yaw/2)*cos(roll/2)+cos(pitch/2)*cos(yaw/2)*sin(roll/2)
#     y=sin(pitch/2)*cos(yaw/2)*cos(roll/2)+cos(pitch/2)*sin(yaw/2)*sin(roll/2)
#     z=cos(pitch/2)*sin(yaw/2)*cos(roll/2)-sin(pitch/2)*cos(yaw/2)*sin(roll/2)
#     w=cos(pitch/2)*cos(yaw/2)*cos(roll/2)-sin(pitch/2)*sin(yaw/2)*sin(roll/2)
#     return x, y, z, w


motive_pose = PoseStamped()

PX4_pose = PoseStamped()

def sub_motiv2(msg):
    global motive_pose
    motive_pose = msg
    
    
if __name__ == '__main__':
    rospy.init_node('Motiv2UAV1', anonymous=True)
    pub = rospy.Publisher('/child1/mavros/vision_pose/pose', PoseStamped, queue_size=1)
    sub = rospy.Subscriber('/vrpn_client_node1/UAV1/pose', PoseStamped, sub_motiv2)
    
    rate = rospy.Rate(50)
    
    while not rospy.is_shutdown():
        
        PX4_pose.pose.position.x = -motive_pose.pose.position.x
        PX4_pose.pose.position.y = motive_pose.pose.position.z
        PX4_pose.pose.position.z = motive_pose.pose.position.y
        
        # PX4_pose.pose.orientation.x = motive_pose.pose.orientation.x
        # PX4_pose.pose.orientation.y = motive_pose.pose.orientation.y
        # PX4_pose.pose.orientation.z = motive_pose.pose.orientation.z
        # PX4_pose.pose.orientation.w = motive_pose.pose.orientation.w
        
        
        
        # quat = [PX4_pose.pose.orientation.w, PX4_pose.pose.orientation.x, PX4_pose.pose.orientation.y, PX4_pose.pose.orientation.z]
        roll, pitch, yaw = euler_from_quaternion(motive_pose.pose.orientation.x, motive_pose.pose.orientation.z, motive_pose.pose.orientation.y, motive_pose.pose.orientation.w)
        
        # x, y, z, w = rpy2quaternion(roll, pitch, yaw=-yaw)
        
        PX4_pose.pose.orientation.x = motive_pose.pose.orientation.x
        PX4_pose.pose.orientation.y = motive_pose.pose.orientation.z
        PX4_pose.pose.orientation.z = motive_pose.pose.orientation.y
        PX4_pose.pose.orientation.w = motive_pose.pose.orientation.w
        
        PX4_pose.header.stamp = motive_pose.header.stamp
        PX4_pose.header.frame_id = motive_pose.header.frame_id
        PX4_pose.header.seq = motive_pose.header.seq
        
        
        pub.publish(PX4_pose)
        
        # print("yaw: ", yaw, "pitch: ", pitch, "roll: ", roll)
        
        rate.sleep()