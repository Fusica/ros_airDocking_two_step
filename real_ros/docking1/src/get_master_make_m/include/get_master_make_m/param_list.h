/**
 * author: kaidi wang
 * list all the global param 
*/
#include <stdio.h>
#include <string.h>
//#include <get_master_make_m/get_master_make_m_node.h>

#define pi 3.1415926
#define MID 1498
int node_id = 0;//self ID, master node ID = 0 
int child_node_num=3;// the number of child node, in this system contain 3 child px4
bool get_home_position = 0;//if get the home_position, set to 1

//advertise a bool param, named child/arming...
ros::Publisher arm_child_flight;
//child/autoland
ros::Publisher takeoff_pub;
//child/kill
ros::Publisher normal_pub;

//nominal_position
ros::Publisher nominal_position_pub;
//nominal_eular_angles
ros::Publisher nominal_eular_angles_pub;
//controller input of main_position
ros::Publisher main_position_pub ;
//controller input of main_velocity
ros::Publisher main_velocity_pub ;
//controller input of main_eular_angles
ros::Publisher main_eular_angles_pub ;
//controller input of main_body_rates
ros::Publisher main_body_rates_pub ;


ros::Publisher init_euler_angles_cmd_pub;
//init cmd publish: position
ros::Publisher init_pos_cmd_pub ;
//init cmd publish: body_rates
ros::Publisher init_body_rates_cmd_pub ;
//init cmd publish: velocity
ros::Publisher init_velocity_cmd_pub ;

//control switch of when publish attitude to child node
ros::Publisher control_start_pub_att ;
ros::Publisher psi_bias_1_pub;
ros::Publisher psi_bias_2_pub;
ros::Publisher psi_bias_3_pub;	

//publish a topic that reset the controller
ros::Publisher reset_controller_pub;

ros::Publisher first_pose_pub;

//define some variables for publish
geometry_msgs::Point nominal_positon;//setpoint position
geometry_msgs::Point nominal_euler_angles;//euler angle setpoint position
//define setpoint last variables for integral last setpoint
geometry_msgs::Point nominal_position_last;//
geometry_msgs::Point nominal_euler_angles_last;	

geometry_msgs::Point main_position;
geometry_msgs::Point main_velocity;
geometry_msgs::Point main_eular_angles;
geometry_msgs::Point main_body_rates;

//define init switch setpoint msg
geometry_msgs::Point init_euler_angles;
geometry_msgs::Point init_position;
geometry_msgs::Point init_body_rate;
geometry_msgs::Point init_velocity;


//define message publish to pose_controller
geometry_msgs::Point nominal_pos_ned;
geometry_msgs::Point nominal_eular_angles_ned;
geometry_msgs::Point main_pos_ned;
geometry_msgs::Point main_velocity_ned;
geometry_msgs::Point main_eular_angles_ned;
geometry_msgs::Point main_body_rates_ned;

//define a var to trans euler
geometry_msgs::PoseStamped quat2euler;
//bias of yaw 
std_msgs::Float64 node1_yaw;
std_msgs::Float64 node2_yaw;
std_msgs::Float64 node3_yaw;

//reset controller
std_msgs::Float64 reset_conroller;
