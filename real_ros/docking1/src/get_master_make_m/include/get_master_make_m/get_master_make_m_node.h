/*author: kaidi wang*/
/*date:2020.12.17*/

//define some functions used in get_master_make_m_node.cpp

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <get_master_make_m/param_list.h>//include the param list 
#include <fstream>//output log file

//define some function
//Coordinate System transform, enu to ned, position
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
geometry_msgs::Point enu2ned_euler(geometry_msgs::Point enu_euler)
{   
    geometry_msgs::Point ned_euler;
    ned_euler.x = enu_euler.y;
    ned_euler.y = enu_euler.x;
    ned_euler.z = -enu_euler.z + pi/2;
    return ned_euler;
}
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


//define some class
//class list
class vector3
{
private:
    /* data */
public:
    float x;
    float y;
    float z;
    vector3(/* args */);
    ~vector3();
};

vector3::vector3(/* args */)
{
}

vector3::~vector3()
{
}

class vector4
{
private:
    /* data */
public:
    float x;
    float y;
    float z;
    float w;
    vector4(/* args */);
    ~vector4();
};

vector4::vector4(/* args */)
{
}

vector4::~vector4()
{
}


class home_position_pose
{
private:
    /* data */
public:

    vector3 pos;
    vector4 quaternion;
    vector3 linear_velocity;
    vector3 angular_velocity;
    //float ;
    home_position_pose(/* args */);
    ~home_position_pose();
};


//init data of original position and pose
home_position_pose::home_position_pose(/* args */)
{
 
}

home_position_pose::~home_position_pose()
{
}


class sw_mapping
{
private:
    float fly_speed = 1; //unit 13.4m/s------ slower to 5m/s
    float yaw_speed = 1;    //1 rad/s
    float lift_speed = 0.5;   //5m/s
    float angular_velocity = 0.02;//0.02 rad/s

    int channel1_up = 1932;
    int channel1_down=1065;
    int channel1_mid =1498;

    int channel2_up = 1908;
    int channel2_down=1042;
    int channel2_mid =1475;

    int channel3_up = 1930;
    int channel3_down=1067;
    int channel3_mid =1498;

    int channel4_up = 1932;
    int channel4_down=1065;
    int channel4_mid =1498;

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

//function of output log txt file 
//input: setpoint pos , real pos, 
//output: a txt file that record input information.
void get_master_log_txt
(geometry_msgs::Point pos_setpoint, 
    geometry_msgs::Point att_setpoint,
    geometry_msgs::Point main_position,
    geometry_msgs::Point main_eular_angles,std::ofstream &out)
{
    //std::ofstream out;
    //write a timestamp
    out<<"Time is: "<<ros::Time::now()<<std::endl;
    out<<"setpoint position, x:"<<pos_setpoint.x<<" y:"<<pos_setpoint.y<<" z:"<<pos_setpoint.z<<std::endl;
    out<<"real position,     x:"<<main_position.x<<" y:"<<main_position.y<<" z:"<<main_position.z<<std::endl;
    out<<"setpoint attitude, roll:"<<att_setpoint.x<<" pitch:"<<att_setpoint.y<<" yaw:"<<att_setpoint.z<<std::endl;
    out<<"real attitude,     roll:"<<main_eular_angles.x<<" pitch:"<<main_eular_angles.y<<" yaw:"<<main_eular_angles.z<<std::endl;
    //write a enter line as a 
    out<<std::endl;
    //return out;
}
