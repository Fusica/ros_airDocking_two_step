import rospy
from geometry_msgs.msg import PoseStamped

motive_pose = PoseStamped()

PX4_pose = PoseStamped()

def sub_motiv2(msg):
    global motive_pose
    motive_pose = msg
    



if __name__ == '__main__':
    rospy.init_node('Motiv2UAV2', anonymous=True)
    pub = rospy.Publisher('/child2/mavros/vision_pose/pose', PoseStamped, queue_size=1)
    sub = rospy.Subscriber('/vrpn_client_node2/UVA2/pose', PoseStamped, sub_motiv2)
    
    rate = rospy.Rate(50)
    
    while not rospy.is_shutdown():
        
        PX4_pose.pose.position.x = motive_pose.pose.position.x
        PX4_pose.pose.position.y = -motive_pose.pose.position.z
        PX4_pose.pose.position.z = motive_pose.pose.position.y
        
        PX4_pose.pose.orientation.x = motive_pose.pose.orientation.x
        PX4_pose.pose.orientation.y = motive_pose.pose.orientation.y
        PX4_pose.pose.orientation.z = motive_pose.pose.orientation.z
        PX4_pose.pose.orientation.w = motive_pose.pose.orientation.w
        
        pub.publish(PX4_pose)
        
        rate.sleep()