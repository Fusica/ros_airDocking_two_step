#ifndef __CONTROL_VELOCITY_H__
#define __CONTROL_VELOCITY_H__

#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Point.h"



void init_vel_func(geometry_msgs::TwistStamped &init_vel);

void init_pos_func(geometry_msgs::PoseStamped &init_pos);


std_msgs::Float32MultiArray quat2rot(geometry_msgs::PoseStamped pos);



#endif