#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>

ros::Subscriber target_attitude_sub;
ros::Subscriber state_sub;
ros::Subscriber get_rc_channel_sub;

ros::Publisher target_pub;


mavros_msgs::PositionTarget yaw_command_from_master_to_child(std_msgs::Float32 p,geometry_msgs::Vector3 v)
{
    mavros_msgs::PositionTarget yaw_tar;
    yaw_tar.yaw_rate= p.data ;
    yaw_tar.velocity.x=v.x;
    yaw_tar.velocity.y=v.y;
    yaw_tar.velocity.z=v.z;
    //yaw_tar.velocity.x=2;
    //yaw_tar.velocity.z=2;
    //yaw_tar.velocity.y=2;
    //yaw_tar.yaw_rate=0.3 ;
    
    return yaw_tar;
}
