

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
//from mavros_msgs.msg import AttitudeTarget,PositionTarget

#include <stdlib.h>
#include <../include/transmit/erwema.h>
#include <mavros_msgs/RCIn.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	current_state = *msg;
}

mavros_msgs::RCIn get_rc_channel;
int sw;
int sw2;
void get_rc_channel_cb(const mavros_msgs::RCIn::ConstPtr& msg)
{
	get_rc_channel = *msg;
	sw= get_rc_channel.channels[9];
    sw2= get_rc_channel.channels[8]; //B 
}

mavros_msgs::AttitudeTarget target_attitude;
void target_attitude_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg)
{			
	target_attitude = *msg;
}

int step=1;
//void px4_command
int main(int argc, char **argv)
{
	ros::init(argc, argv, "offb_node");
	ros::NodeHandle nh;

	//新添加一句订阅local_position消息的代码
	state_sub = nh.subscribe<mavros_msgs::State>
		("/mavros/state", 10, state_cb);
	get_rc_channel_sub = nh.subscribe<mavros_msgs::RCIn>
		("/mavros/rc/in",10,get_rc_channel_cb);
	target_attitude_sub = nh.subscribe<mavros_msgs::AttitudeTarget>
		("/des_attitude", 10, target_attitude_cb);

	target_pub = nh.advertise<mavros_msgs::AttitudeTarget>
		("/mavros/setpoint_raw/attitude", 10);
	

	ros::Rate rate(100.0);

	while (ros::ok() && current_state.connected) {
		ros::spinOnce();
		rate.sleep();
	}
	
	std_msgs::Float32 servo;
	servo.data = 0.0f;

	for (int i = 100; ros::ok() && i > 0; --i) 
	{
		ROS_INFO("init target_attitude was %f", target_attitude);
		target_pub.publish(target_attitude);
	 	ros::spinOnce();
	 	rate.sleep();
	}
	while(ros::ok())
	{
		switch (step)
		{//起飞
			case 1:
			if(sw > 1500 && sw2 < 1500)
				{
                    //遥控器 sw也是arm
			    	servo.data = 0.0f;
				}
				else if(sw < 1500 && sw2 >1500)
				{
                    //轨迹
                	servo.data = 1.0f;
				}
                else if(sw < 1500 && sw2 < 1500)
                {
                    //定点
                    servo.data = 2.0f;
                }
		    //target_attitude.thrust = 0.5;
			// target_attitude.body_rate.x = 0.;
			// target_attitude.body_rate.y = 0.;
			// target_attitude.body_rate.z = 0.;
		    ROS_INFO("bodyrate x was %f", target_attitude.body_rate.x);
		    ROS_INFO("bodyrate y was %f", target_attitude.body_rate.y);
		    ROS_INFO("bodyrate z was %f", target_attitude.body_rate.z);
		    ROS_INFO("thrust was %f", target_attitude.thrust);
		    // target_attitude.type_mask=128;
            target_pub.publish(target_attitude);
			
			break;
			//悬停判定
			case 4: 
				ROS_INFO("悬停判定");
				ROS_INFO("target_attitude was %f", target_attitude);
				// target_attitude.type_mask=128;
				target_pub.publish(target_attitude);
				break;
				//降落模式
			case 5:
				
				break;
				 
		}
			//}
		if(step == 5)
		{
				
			break;
		}
		//acro下降落
		if(current_state.mode == "ACRO")
		{
			break;
		}
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
