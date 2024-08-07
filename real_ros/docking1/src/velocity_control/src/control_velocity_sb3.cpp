#include <control_velocity/control_velocity.h>
#include <control_velocity/params_list.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <Eigen/Dense>
#include <mavros_msgs/RCIn.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include "../network/ThrustOmega_model/1027vvvvv/ppo_F450_rot_spinz_pos_vel_01_01_102_05_baseself0815_0623_motor0.15_default/seed_001/network_evaluate.c"
//#include "../network/20230619_test/seed_002/network_evaluate.c"


#include <Python.h>
#include <numpy/arrayobject.h>
namespace py = boost::python;

#include <sstream>
#include <time.h>
using namespace Eigen;
int vv = 0;
float timev = 0;

void quat2rot_change(geometry_msgs::PoseStamped pos)
{
  float w = pos.pose.orientation.w ;
  float x = pos.pose.orientation.x ;
  float y = pos.pose.orientation.y ;
  float z = pos.pose.orientation.z ;

  r_now[0] = 2*x*y + 2*z*w;
  r_now[1] = 1.0 - 2*x*x - 2*z*z;
  r_now[2] = 2*y*z - 2*x*w;
  r_now[3] = 2*y*y + 2*z*z -1.0;
  r_now[4] = 2*z*w - 2*x*y;
  r_now[5] = -2*x*z - 2*y*w;
  r_now[6] = 2*x*z - 2*y*w;
  r_now[7] = 2*y*z + 2*x*w;
  r_now[8] = 1.0 - 2*x*x -2*y*y;
}

mavros_msgs::RCIn get_rc_channel;
int sw;
int start = 0;
int first = 0;
void get_rc_channel_cb(const mavros_msgs::RCIn::ConstPtr& msg)
{
	get_rc_channel = *msg;
  sw = get_rc_channel.channels[4];
  ROS_INFO_STREAM(sw);
	if(sw > 1900)
    {start = 1;}
}

void quat2rot_new(geometry_msgs::PoseStamped pos)
{
  float w = pos.pose.orientation.w ;
  float x = pos.pose.orientation.x ;
  float y = pos.pose.orientation.y ;
  float z = pos.pose.orientation.z ;
  Eigen::Quaterniond quaternionn(w,x,y,z);

  Eigen::Matrix3d rotation_matrixn;rotation_matrixn=quaternionn.toRotationMatrix();
  real_eulerAngle=quaternionn.matrix().eulerAngles(2,1,0);
  real_eul.x = real_eulerAngle(2);
  real_eul.y = real_eulerAngle(1);
  real_eul.z = real_eulerAngle(0);
  r_now[0] = rotation_matrixn(0,0);
  r_now[1] = rotation_matrixn(0,1);
  r_now[2] = rotation_matrixn(0,2);
  r_now[3] = rotation_matrixn(1,0);
  r_now[4] = rotation_matrixn(1,1);
  r_now[5] = rotation_matrixn(1,2);
  r_now[6] = rotation_matrixn(2,0);
  r_now[7] = rotation_matrixn(2,1);
  r_now[8] = rotation_matrixn(2,2);
  // ROS_INFO_STREAM("change rot check : "<<rot);
}

geometry_msgs::PoseStamped randinit_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	local_pos = *msg;
  float tmp;
  tmp = local_pos.pose.position.x;
  local_pos.pose.position.x = local_pos.pose.position.y;
  local_pos.pose.position.y = -tmp;
  
  vv  = 1;
  if(local_pos.pose.orientation.w < 0)
	{
    local_pos.pose.orientation.x =-local_pos.pose.orientation.x;
    local_pos.pose.orientation.y =-local_pos.pose.orientation.y;
    local_pos.pose.orientation.z =-local_pos.pose.orientation.z;
    local_pos.pose.orientation.w =-local_pos.pose.orientation.w;
	}
  if(first == 0 && start == 1)
  {
    randinit_pos = local_pos;
    first = 1;
  }
}

geometry_msgs::Point body2worldVel(float rotv[9])
{
  geometry_msgs::Point local_vel;
  float x = mav_vel_receive.twist.linear.x;
  float y = mav_vel_receive.twist.linear.y;
  float z = mav_vel_receive.twist.linear.z;
  //ROS_INFO_STREAM("vel sou : "<<local_vel);
  local_vel.x = rotv[0] * x + rotv[1] * y + rotv[2] * z;
  local_vel.y = rotv[3] * x + rotv[4] * y + rotv[5] * z;
  local_vel.z = rotv[6] * x + rotv[7] * y + rotv[8] * z;
  return local_vel;
  
  //ROS_INFO_STREAM("vel final : "<<local_vel);
}

void mav_vel_receive_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  mav_vel_receive = *msg;
}

void nominal_position_cb(const geometry_msgs::Point::ConstPtr& msg) {
	nominal_pos = *msg;
  // float tmp;
  // tmp = nominal_pos.x;
  // nominal_pos.x = nominal_pos.y;
  // nominal_pos.y = -tmp;
}

std_msgs::Bool arm;
void arm_cb(const std_msgs::Bool::ConstPtr& msg) {
	arm = *msg;
}

void nominal_euler_angles_cb(const geometry_msgs::Point::ConstPtr& msg) {
  nominal_euler = *msg;
	// ROS_INFO_STREAM("nomieuler: "<<nominal_euler);
  Eigen::Vector3d eulerAngle;
  eulerAngle.x() = nominal_euler.x;
  eulerAngle.y() = nominal_euler.y;
  eulerAngle.z() = nominal_euler.z;
  Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle(0),Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle(1),Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle(2),Vector3d::UnitZ())); 
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix=yawAngle*pitchAngle*rollAngle;
  r_d[0] = rotation_matrix(0,0);
  r_d[1] = rotation_matrix(0,1);
  r_d[2] = rotation_matrix(0,2);
  r_d[3] = rotation_matrix(1,0);
  r_d[4] = rotation_matrix(1,1);
  r_d[5] = rotation_matrix(1,2);
  r_d[6] = rotation_matrix(2,0);
  r_d[7] = rotation_matrix(2,1);
  r_d[8] = rotation_matrix(2,2);
}

void errorRot()
{
  // std_msgs::Float32MultiArray rot;
  rot[0] = r_d[0] * r_now[0] + r_d[3] * r_now[3] + r_d[6] * r_now[6];
  rot[1] = r_d[0] * r_now[1] + r_d[3] * r_now[4] + r_d[6] * r_now[7];
  rot[2] = r_d[0] * r_now[2] + r_d[3] * r_now[5] + r_d[6] * r_now[8];
  rot[3] = r_d[1] * r_now[0] + r_d[4] * r_now[3] + r_d[7] * r_now[6];
  rot[4] = r_d[1] * r_now[1] + r_d[4] * r_now[4] + r_d[7] * r_now[7];
  rot[5] = r_d[1] * r_now[2] + r_d[4] * r_now[5] + r_d[7] * r_now[8];
  rot[6] = r_d[2] * r_now[0] + r_d[5] * r_now[3] + r_d[8] * r_now[6];
  rot[7] = r_d[2] * r_now[1] + r_d[5] * r_now[4] + r_d[8] * r_now[7];
  rot[8] = r_d[2] * r_now[2] + r_d[5] * r_now[5] + r_d[8] * r_now[8];
}

void state_pos_cb(const geometry_msgs::Point::ConstPtr& msg){
  ROS_INFO("state call back");
  state_pos = *msg;
}

void state_vel_cb(const geometry_msgs::Point::ConstPtr& msg){
  ROS_INFO("state call back");
  state_vel = *msg;
}

void state_rot1_cb(const geometry_msgs::Point::ConstPtr& msg){
  ROS_INFO("state call back");
  state_rot1 = *msg;
}

void state_rot2_cb(const geometry_msgs::Point::ConstPtr& msg){
  ROS_INFO("state call back");
  state_rot2 = *msg;
}

void state_rot3_cb(const geometry_msgs::Point::ConstPtr& msg){
  ROS_INFO("state call back");
  state_rot3 = *msg;
}

void state_omega_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
  ROS_INFO("state call back");
  state_omega = *msg;
}

void mode_cb(const mavros_msgs::State::ConstPtr& msg){
  mode = *msg;  
}

float thrust_world2body(float thrust_world, float rotv[9]){
  return rotv[8] * thrust_world;
}

// 导入模块
py::object my_module = py::import("model_predict");
// 获取函数对象
py::object my_function = my_module.attr("modelPredict");
void networkEvaluate(&control_n,  state_input[18]){
    // 创建参数
    //std::vector<int> input = {1, 2, 3};
    py::list args;
    for (int value : state_input) {
        args.append(value);
    }

    // 调用函数
    py::object result = my_function(args);

    // 获取返回值
      control_n.thrust_0 = result[0];
      control_n.thrust_1 = result[1];
      control_n.thrust_2 = result[2];
      control_n.thrust_3 = result[3];

}

void timercallback(const ros::TimerEvent&){
  struct control_t_n control_n;
  geometry_msgs::Vector3 msg ;
  geometry_msgs::Point nominal_pos_enu;
  std_msgs::Float32 yaw_rate;
  std_msgs::Int32 offboard_start;


  if(start == 1){

    quat2rot_change(local_pos);
    // nominal_pos.x = randinit_pos.pose.position.x;
    // nominal_pos.y = randinit_pos.pose.position.y;
    // nominal_pos.z = randinit_pos.pose.position.z;
    float tmp;
    nominal_pos_enu.x = - nominal_pos.y;
    nominal_pos_enu.y =   nominal_pos.x;
    nominal_pos_enu.z =   nominal_pos.z;
    nominal_pos_enu_pub.publish(nominal_pos_enu);




    // nominal_pos.x = 0.;
    // nominal_pos.y = 0;
    // nominal_pos.z = 2.;
    first_pos_pub.publish(randinit_pos);
    timev = timev + 0.1;
    ROS_INFO_STREAM("nominal_pos is :"<<nominal_pos);
    err_pos.x = local_pos.pose.position.x - nominal_pos.x;
    err_pos.y = local_pos.pose.position.y - nominal_pos.y;
    err_pos.z = local_pos.pose.position.z - nominal_pos.z;
    err_vel = body2worldVel(r_now);
    r_d[0] = 1;
    r_d[1] = 0;
    r_d[2] = 0;
    r_d[3] = 0;
    r_d[4] = 1;
    r_d[5] = 0;
    r_d[6] = 0;
    r_d[7] = 0;
    r_d[8] = 1;
    // ROS_INFO_STREAM("r_d[8] "<<r_now[8]);
    errorRot();
    err_rot1.x = rot[0];
    err_rot1.y = rot[1];
    err_rot1.z = rot[2];
    err_rot2.x = rot[3];
    err_rot2.y = rot[4];
    err_rot2.z = rot[5];
    err_rot3.x = rot[6];
    err_rot3.y = rot[7];
    err_rot3.z = rot[8];
    // ROS_INFO_STREAM("err_rot1 is :"<<rot[8]);
    // ROS_INFO_STREAM("err_rot3.z is :"<< err_rot3.z);
    _Float64 state_input[18] = {err_pos.x, err_pos.y, err_pos.z,
                                err_vel.x, err_vel.y, err_vel.z,
                                err_rot1.x, err_rot1.y, err_rot1.z,
                                err_rot2.x, err_rot2.y, err_rot2.z,
                                err_rot3.x, err_rot3.y, err_rot3.z,
                                0.,0.,0.};
                                //mav_vel_receive.twist.angular.x, mav_vel_receive.twist.angular.y, mav_vel_receive.twist.angular.z};
        ROS_INFO_STREAM("state_input 0 is :"<<state_input[0]);
        ROS_INFO_STREAM("state_input 1 is :"<<state_input[1]);
        ROS_INFO_STREAM("state_input 2 is :"<<state_input[2]);
        ROS_INFO_STREAM("state_input 3 is :"<<state_input[3]);
        ROS_INFO_STREAM("state_input 4 is :"<<state_input[4]);
        ROS_INFO_STREAM("state_input 5 is :"<<state_input[5]);
        ROS_INFO_STREAM("state_input 6 is :"<<state_input[6]);
        ROS_INFO_STREAM("state_input 7 is :"<<state_input[7]);
        ROS_INFO_STREAM("state_input 8 is :"<<state_input[8]);
        ROS_INFO_STREAM("state_input 9 is :"<<state_input[9]);
        ROS_INFO_STREAM("state_input 10 is :"<<state_input[10]);
        ROS_INFO_STREAM("state_input 11 is :"<<state_input[11]);
        ROS_INFO_STREAM("state_input 12 is :"<<state_input[12]);
        ROS_INFO_STREAM("state_input 13 is :"<<state_input[13]);
        ROS_INFO_STREAM("state_input 14 is :"<<state_input[14]);
        ROS_INFO_STREAM("state_input 15 is :"<<state_input[15]);
        ROS_INFO_STREAM("state_input 16 is :"<<state_input[16]);
        ROS_INFO_STREAM("state_input 17 is :"<<state_input[17]);

    // for(int i = 0;i < 20;i++)
    //  {
    networkEvaluate(&control_n,  state_input);
    // }
    mavros_msgs::AttitudeTarget px4_command;
    px4_command.type_mask = 128;
		px4_command.body_rate.x = control_n.thrust_1 ; // 0.42
		px4_command.body_rate.y = control_n.thrust_2 ; // 0.42
		px4_command.body_rate.z = control_n.thrust_3 ; // 0.42
    px4_command.thrust = (control_n.thrust_0 + 1) / 1.93 * 2 * 1.0 * 0.50;
    //px4_command.thrust = control_n.thrust_0*0.4 + 0.50 ;
    
    px4_command_pub_.publish(px4_command);
  }
  else
  {
    ROS_INFO_STREAM("mode.system_status "<< mode.system_status);
    ROS_INFO("No RL");
  }
  offboard_start.data = start;
  offboard_start_pub.publish(offboard_start);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_velocity");
    ros::NodeHandle n;
 
	  px4_command_pub_ = n.advertise<mavros_msgs::AttitudeTarget>("/des_attitude", 10);
    nominal_pos_enu_pub = n.advertise<geometry_msgs::Point>("/nominal_pos_enu", 10);
    //euler_pub = n.advertise<geometry_msgs::Point>("/real", 10);
    first_pos_pub = n.advertise<geometry_msgs::PoseStamped>("/randinit_pos", 10);
    //px4_command_pub_ = n.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
    offboard_start_pub = n.advertise<std_msgs::Int32>("/offboard_start", 10);
    

    //subscribe state_array.covariance
    ros::Subscriber state_pos_sub = n.subscribe<geometry_msgs::Point>
		  ("/state_pos", 10, state_pos_cb);
	  ros::Subscriber nominal_position_sub = n.subscribe<geometry_msgs::Point>("/nominal_position",10, nominal_position_cb);
	  ros::Subscriber nominal_eular_angles_sub = n.subscribe<geometry_msgs::Point>("/nominal_euler_angles",10, nominal_euler_angles_cb);
    ros::Subscriber arm_sub = n.subscribe<std_msgs::Bool>("/start_pub_att", 10, arm_cb);
    ros::Subscriber state_vel_sub = n.subscribe<geometry_msgs::Point>
		  ("/state_vel", 10, state_vel_cb);
    ros::Subscriber state_rot1_sub = n.subscribe<geometry_msgs::Point>
		  ("/state_rot1", 10, state_rot1_cb);
    ros::Subscriber state_rot2_sub = n.subscribe<geometry_msgs::Point>
		  ("/state_rot2", 10, state_rot2_cb);
	  ros::Subscriber get_rc_channel_sub = n.subscribe<mavros_msgs::RCIn>
		  ("/mavros/rc/in",10,get_rc_channel_cb);
    ros::Subscriber state_rot3_sub = n.subscribe<geometry_msgs::Point>
		  ("/state_rot3", 10, state_rot3_cb);
    ros::Subscriber state_omega_sub = n.subscribe<geometry_msgs::TwistStamped>
		  ("/state_omega", 10, state_omega_cb);
   ros::Subscriber local_pos_sub = n.subscribe<geometry_msgs::PoseStamped>
		  ("/mavros/local_position/pose", 10, local_pos_cb);
    ros::Subscriber mav_vel_receive_sub = n.subscribe<geometry_msgs::TwistStamped>
      ("/mavros/local_position/velocity_body",10,mav_vel_receive_cb);

    

    ros::Subscriber mode_sub = n.subscribe<mavros_msgs::State>
		  ("/mavros/mode", 10, mode_cb);


    // 初始化Python解释器
    Py_Initialize();
    Py_SetPythonHome("/home/fyt/miniconda3/envs/pytorch");

    arm.data = false;
    ros::Timer timer = n.createTimer(ros::Duration(0.01), timercallback);
    
    ros::spin();

    Py_Finalize();

  return 0;
}
