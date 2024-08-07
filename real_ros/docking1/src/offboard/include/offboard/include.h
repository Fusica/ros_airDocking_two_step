#pragma once
#include <opencv2/opencv.hpp>
#include <fstream>
#include <vector>
#include <string.h>
#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <mavros_msgs/RCIn.h>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Bool.h>


#include <chrono>
#include <mutex>
#include <thread>
#include <pthread.h>
#include <condition_variable>

#include <random>


#define MID 1498//油门中点

using namespace Eigen;
using namespace std;
template<typename T>
T readParam(ros::NodeHandle &n, string name){
    T ans;
    //使用getParam函数读取launch里的参数
    if(n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded "<<name<<": "<<ans);
    }
    else{
        ROS_ERROR_STREAM("Failed to load "<<name);
        n.shutdown();
    }
    return ans;
}

Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
{
	Eigen::Vector3d n = R.col(0);
	Eigen::Vector3d o = R.col(1);
	Eigen::Vector3d a = R.col(2);

	Eigen::Vector3d ypr(3);
	double y = atan2(n(1), n(0));
	double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
	double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
	ypr(0) = y;
	ypr(1) = p;
	ypr(2) = r;

	return ypr / M_PI * 180.0;
}

class TicToc
{
  public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};


class sw_mapping
{
private:
    float fly_speed = 2; //unit 13.4m/s------ slower to 5m/s
    float yaw_speed = 2;    //1 rad/s
    float lift_speed = 1;   //5m/s
    float angular_velocity = 0.02;//0.02 rad/s

    int channel1_up = 1072;
    int channel1_down=1925;
    int channel1_mid =1502;

    int channel2_up = 1080;
    int channel2_down=1947;
    int channel2_mid =1510;

    int channel3_up = 1908;
    int channel3_down=1042;
    int channel3_mid =1475;

    int channel4_up = 1930;
    int channel4_down=1076;
    int channel4_mid =1499;

    // int mid = 1497;
    // int up = 1932;
    // int down = 1065;

    // int mid2 = 1493;
    // int mid3 = 1430;//thrush mid positon velue
    // int down3=1056;
    // int up3=1932;
public:
    float vel[4];
    float ang_vel[3];
    geometry_msgs::Point pos_setpoint;//setpoint position
    geometry_msgs::Point att_setpoint;//setpoint attitude
    void sw_map_vel(int* sw)
    {
        vel[0]=((float)(sw[0]-channel1_mid)/(float)(channel1_up-channel1_mid))*fly_speed;//channel 1
        vel[1]=((float)(sw[1]-channel2_mid)/(float)(channel2_up-channel2_mid))*fly_speed;//channel 2
        vel[2]=((float)(sw[2]-channel3_mid)/(float)(channel3_up-channel3_mid))*lift_speed;//channel 3
        vel[3]=-((float)(sw[3]-channel4_mid)/(float)(channel4_up-channel4_mid))*yaw_speed;//channel 4
        //debug line section 
        // for(int i=0;i<4;i++)
        // {
        //     //ROS_INFO_STREAM("sw "<<i<<" : "<<sw[i]);
        //     ROS_INFO_STREAM("vel "<<i<<" : "<<vel[i]);
        // }
    }

    //map the RC channel to angular velocity
    void sw_map_ang_vel(int* sw)
    {
        ang_vel[0]=((float)(sw[2]-channel3_mid)/(float)(channel3_up-channel3_mid))*angular_velocity;
        ang_vel[1]=((float)(sw[0]-channel1_mid)/(float)(channel1_up-channel1_mid))*angular_velocity;
        ang_vel[2]=-((float)(sw[3]-channel4_mid)/(float)(channel4_up-channel4_mid))*yaw_speed;
    }

    //calc the setpoint position and setpoint attitude 
    void calc_setpoint
    (geometry_msgs::Point pos_last, geometry_msgs::Point att_last,float *vel,float t)
    {
        pos_setpoint.x = pos_last.x + vel[1]*t;
        pos_setpoint.y = pos_last.y + vel[0]*t;
        pos_setpoint.z = pos_last.z + vel[2]*t;
        att_setpoint.x = att_last.x ;
        att_setpoint.y = att_last.y ;
        att_setpoint.z = att_last.z + vel[3]*t;
        //ROS_INFO_STREAM("yaw: "<<att_last.z);

    } 


    void calc_setpoint_attitude
    (geometry_msgs::Point pos_last, geometry_msgs::Point att_last,float *vel,float t)
    {  
        pos_setpoint.x = pos_last.x ;
        pos_setpoint.y = pos_last.y ;
        pos_setpoint.z = pos_last.z ;
        att_setpoint.x = att_last.x + ang_vel[1]*t;
        att_setpoint.y = att_last.y + ang_vel[0]*t;
        att_setpoint.z = att_last.z + ang_vel[2]*t;
    }
    sw_mapping(/* args */);
    ~sw_mapping();
};

sw_mapping::sw_mapping(/* args */)
{
    for(int i=0;i<4;i++)
    {
        vel[i]=0;
    }
    // att_last.x = 0;
    // att_last.y = 0;
    // att_last.z = 0;
}

sw_mapping::~sw_mapping(/* args */)
{
    
}


/*****    customize function define list    *****/
//trans PX4 attitude to controller input, position
geometry_msgs::Point make_main_position(geometry_msgs::PoseStamped local_pos)
{
	geometry_msgs::Point main_pos;
	main_pos.x = local_pos.pose.position.x;
	main_pos.y = local_pos.pose.position.y;
	main_pos.z = local_pos.pose.position.z;
	return main_pos;
}

//tran PX4 attitude to controller input, eular angles
geometry_msgs::Point make_eular_angles(geometry_msgs::PoseStamped local_pos)
{
	geometry_msgs::Point main_eul_ang;
	double roll, pitch, yaw;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(local_pos.pose.orientation,quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

	main_eul_ang.x=roll;
	main_eul_ang.y=pitch;
	main_eul_ang.z=yaw;
	//ROS_INFO_STREAM("x: "<<main_eul_ang.x);
	//ROS_INFO_STREAM("y: "<<main_eul_ang.y);
	//ROS_INFO_STREAM("z: "<<main_eul_ang.z);
	return main_eul_ang;
} 



geometry_msgs::Point enu2ned_pos(geometry_msgs::Point enu_pos)
{
    geometry_msgs::Point ned_pos;
    ned_pos.x = enu_pos.y;
    ned_pos.y = enu_pos.x;
    ned_pos.z = -enu_pos.z; 
    return ned_pos;
}
//Coordinate System transform, enu to ned, velcity
geometry_msgs::Point enu2ned_vel(geometry_msgs::Point enu_vel)
{
    geometry_msgs::Point ned_vel;
    ned_vel.x = enu_vel.y;
    ned_vel.y = enu_vel.x;
    ned_vel.z = -enu_vel.z; 
    return ned_vel;
}

//Coordinate System transform, enu to ned, euler
// geometry_msgs::Point enu2ned_euler(geometry_msgs::Point enu_euler)
// {   
//     geometry_msgs::Point ned_euler;
//     ned_euler.x = enu_euler.y;
//     ned_euler.y = enu_euler.x;
//     ned_euler.z = -enu_euler.z + pi/2;
//     return ned_euler;
// }
//Coordinate System transform, enu to ned, euler_rate
geometry_msgs::Point enu2ned_euler_rate(geometry_msgs::Point enu_euler_rate)
{   
    geometry_msgs::Point ned_euler_rate;
    ned_euler_rate.x = enu_euler_rate.y;
    ned_euler_rate.y = enu_euler_rate.x;
    ned_euler_rate.z = -enu_euler_rate.z;
    return ned_euler_rate;
}
//Coordinate System transform, ned to nwu, euler_rate
geometry_msgs::Point enu2nwu_pos(geometry_msgs::Point enu_pos)
{
    geometry_msgs::Point nwu_pos;
    nwu_pos.x = enu_pos.y;
    nwu_pos.y = -enu_pos.x;
    nwu_pos.z = enu_pos.z; 
    return nwu_pos;
}