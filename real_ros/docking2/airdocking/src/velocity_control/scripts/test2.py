import rospy
from mavros_msgs.msg import State

def state_callback(msg):
    # 在这里处理接收到的消息
    mode = msg.mode
    print('Received state: %s' % mode)

def timecallback(event):
    print(1)
    

rospy.init_node('state_listener', anonymous=True)
rospy.Subscriber("/mavros/state", State, state_callback)
rospy.Timer(rospy.Duration(0.01), timecallback)
rospy.spin()  # 保持监听直到节点被关闭
