import rospy
from mavros_msgs.msg import State

def state_callback(msg):
    # 在这里处理接收到的消息
    print('Received state: %s' % msg)

def listener():
    rospy.init_node('state_listener', anonymous=True)
    rospy.Subscriber("/mavros/state", State, state_callback)
    rospy.spin()  # 保持监听直到节点被关闭

if __name__ == '__main__':
    listener()
