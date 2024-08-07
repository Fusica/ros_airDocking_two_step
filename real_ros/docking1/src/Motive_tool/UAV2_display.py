import rospy
from geometry_msgs.msg import PoseStamped
import math

# motive_pose = PoseStamped()

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

PX4_pose = PoseStamped()

def sub_PX4_pose(msg):
    global PX4_pose
    PX4_pose = msg
    

if __name__ == '__main__':
    rospy.init_node('UAV2_display')
    sub = rospy.Subscriber('/child2/mavros/local_position/pose', PoseStamped, sub_PX4_pose)
    
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        
        quat = [PX4_pose.pose.orientation.w, PX4_pose.pose.orientation.x, PX4_pose.pose.orientation.y, PX4_pose.pose.orientation.z]
        roll, pitch,yaw = euler_from_quaternion(PX4_pose.pose.orientation.x, PX4_pose.pose.orientation.y, PX4_pose.pose.orientation.z, PX4_pose.pose.orientation.w)
        
        print("yaw: ", yaw, "pitch: ", pitch, "roll: ", roll)
        
        rate.sleep()