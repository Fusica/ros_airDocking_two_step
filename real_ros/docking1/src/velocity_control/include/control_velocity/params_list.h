#include <ros/ros.h>

#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "mavros_msgs/State.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Point.h"
#include <Eigen/Dense>



ros::Publisher velocity_pub;
ros::Publisher yaw_rate_pub;
ros::Publisher px4_command_pub_;
ros::Publisher euler_pub;
ros::Publisher first_pos_pub;
ros::Publisher nominal_pos_enu_pub;
ros::Publisher offboard_start_pub;

geometry_msgs::Point state_pos;
geometry_msgs::Point state_vel;
geometry_msgs::Point state_rot1;
geometry_msgs::Point state_rot2;
geometry_msgs::Point state_rot3;
geometry_msgs::Point err_pos;
geometry_msgs::Point err_vel;
geometry_msgs::Point err_rot1;
geometry_msgs::Point err_rot2;
geometry_msgs::Point err_rot3;
geometry_msgs::Point real_eul;
geometry_msgs::Point nominal_pos;
geometry_msgs::Point nominal_euler;
geometry_msgs::TwistStamped state_omega;
geometry_msgs::PoseStamped local_pos;
geometry_msgs::TwistStamped mav_vel_receive;
geometry_msgs::TwistStamped world_vel;
Eigen::Vector3d real_eulerAngle;
float r_now[9];
float rot[9];
float r_d[9];
geometry_msgs::Vector3 mav_vel;
mavros_msgs::State mode;


