#include <ros/ros.h>

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "mavros_msgs/State.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Point.h"


ros::Publisher velocity_pub;
ros::Publisher yaw_rate_pub;
ros::Publisher enu_nominal_pos;
ros::Publisher first_pos_pub;
ros::Publisher norminal_pos_pub;
ros::Publisher nwu_rot_pub;
ros::Publisher nwu_vel_pub;
ros::Publisher px4_command_pub_;
ros::Publisher net_state_pub;
ros::Publisher mode_pub;
// ros::Publisher net_state1_pub;
// ros::Publisher net_state2_pub;
// ros::Publisher net_state3_pub;
// ros::Publisher net_state4_pub;
// ros::Publisher net_state5_pub;
// ros::Publisher net_state6_pub;
// ros::Publisher net_state7_pub;
// ros::Publisher net_state8_pub;
// ros::Publisher net_state9_pub;
// ros::Publisher net_state10_pub;
// ros::Publisher net_state11_pub;
// ros::Publisher net_state12_pub;
// ros::Publisher net_state13_pub;
// ros::Publisher net_state14_pub;
// ros::Publisher net_state15_pub;
// ros::Publisher net_state16_pub;
// ros::Publisher net_state17_pub;


//std_msgs::Float32MultiArray rot_init;
float rot[9];
float r_d[9];
float r_now[9];
float n=0.;
float y_n=0.;

mavros_msgs::State mode_now;
mavros_msgs::State mode_code;
geometry_msgs::TwistStamped local_vel;
geometry_msgs::TwistStamped mav_vel_receive;


geometry_msgs::PoseStamped local_pos;
geometry_msgs::PoseStamped first_pos;
//randinit_pos;

geometry_msgs::Vector3 mav_vel;

geometry_msgs::Point nominal_position;
// std_msgs::Bool start_fly;
std_msgs::Bool start_fly;

geometry_msgs::Point enp;

// std_msgs::Float32 state0;
// std_msgs::Float32 state1;
// std_msgs::Float32 state2;
// std_msgs::Float32 state3;
// std_msgs::Float32 state4;
// std_msgs::Float32 state5;
// std_msgs::Float32 state6;
// std_msgs::Float32 state7;
// std_msgs::Float32 state8;
// std_msgs::Float32 state9;
// std_msgs::Float32 state10;
// std_msgs::Float32 state11;
// std_msgs::Float32 state12;
// std_msgs::Float32 state13;
// std_msgs::Float32 state14;
// std_msgs::Float32 state15;
// std_msgs::Float32 state16;
// std_msgs::Float32 state17;