//author  : kaidi wang
//date    : 2020.12.20
//descript: master policy decision node 
#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/RCIn.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <get_master_make_m/get_master_make_m_node.h>

//synchron message
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <chrono>

//define arm_bool param
std_msgs::Bool arm_bool;
std_msgs::Bool autoland_bool;
std_msgs::Bool normal_bool;
//define the msg of when start to publish attitude
std_msgs::Bool start_pub_att;

//define sw class
sw_mapping sw;
int thrust_last_time=0;
int count = 0;
bool let_fly = false;//bool fly switch
bool init_controller = false; //bool init controller


//callback fucntion list
//current state callback function
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	current_state = *msg;
}

//callback function of get_rc data
mavros_msgs::ManualControl get_rc;
void get_rc_setpoint_cb(const mavros_msgs::ManualControl::ConstPtr& msg)
{
    //ROS_INFO("enter the rc callback function....");
    get_rc = *msg;
}

//callback function of get_rc_channel_cb
mavros_msgs::RCIn get_rc_channel;
int sw_arm;
int sw_auto_land;
int sw_kill;
int sw_fly;
int init_pos_flag;
//thrust is same as a channel of switch_yaw_xyz[i]
int thrust;
int switch_yaw_xyz[4];//xyz,yaw

//define a val to record the switch mode of postion and attitude
int switch_position_attitude=0;
void get_rc_channel_cb(const mavros_msgs::RCIn::ConstPtr& msg)
{
	get_rc_channel = *msg;
	sw_arm = get_rc_channel.channels[9];
	sw_auto_land = get_rc_channel.channels[8];
	sw_kill = get_rc_channel.channels[6];
    sw_fly = get_rc_channel.channels[7];

	init_pos_flag = get_rc_channel.channels[4];
	//map a channel of RC to the val, the channel number should be changed 
	switch_position_attitude = get_rc_channel.channels[5];

	//get the thrust volume
	thrust = get_rc_channel.channels[2];
	for(int i =0 ; i<4; i++)
	{
		switch_yaw_xyz[i]=get_rc_channel.channels[i];
	}
	// ROS_INFO_STREAM("channel 1: "<<get_rc_channel.channels[0]<<" "<<switch_yaw_xyz[0]);
	// ROS_INFO_STREAM("channel 2: "<<get_rc_channel.channels[1]<<" "<<switch_yaw_xyz[1]);
	// ROS_INFO_STREAM("channel 3: "<<get_rc_channel.channels[2]<<" "<<switch_yaw_xyz[2]);
	// ROS_INFO_STREAM("channel 4: "<<get_rc_channel.channels[3]<<" "<<switch_yaw_xyz[3]);
	// ROS_INFO_STREAM("channel 7: "<<get_rc_channel.channels[6]);
	// ROS_INFO_STREAM("channel 9: "<<get_rc_channel.channels[8]);
	// ROS_INFO_STREAM("channel 10: "<<get_rc_channel.channels[9]);

}

//is_all_connected callback function
std_msgs::Bool is_all_connected;
void is_all_connected_cb(const std_msgs::Bool::ConstPtr& msg)
{
	is_all_connected = *msg;
	ROS_INFO_STREAM("receive info of is_all_connected:"<<is_all_connected);
}

//is_all_armd callback function
std_msgs::Bool is_all_armd;
void is_all_armd_cb(const std_msgs::Bool::ConstPtr& msg)
{
	is_all_armd = *msg;
	ROS_INFO_STREAM("receive info of is_all_armd:"<<is_all_armd);
}

//local pos callback function
geometry_msgs::PoseStamped local_pos;
geometry_msgs::PoseStamped first_pose;
int firstget = 0;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	local_pos = *msg;
	if (firstget == 0)
	{
	first_pose = *msg;
	firstget = 1;
	}
	// ROS_INFO_STREAM("enu pos x: "<<local_pos.pose.position.x);
	// ROS_INFO_STREAM("enu pos y: "<<local_pos.pose.position.y);
	// ROS_INFO_STREAM("enu pos z: "<<local_pos.pose.position.z);
}

//local velocity callback function
geometry_msgs::TwistStamped local_vel;
void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	local_vel = *msg;
	//ROS_INFO_STREAM("local vel x: "<<local_pos.pose.position.x);
	//ROS_INFO_STREAM("local vel x: "<<local_pos.pose.position.y);
	//ROS_INFO_STREAM("local vel x: "<<local_pos.pose.position.z);
}

//combain sensor callback function
void multi_sensor_callback(const geometry_msgs::PoseStamped::ConstPtr& msg_pose,
const geometry_msgs::TwistStamped::ConstPtr& msg_twist)
{
	local_pos = *msg_pose;
	local_vel = *msg_twist;
}


//main body rate simulate callback function
geometry_msgs::Point sim_main_body_rates;
void main_body_rates_sim_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	sim_main_body_rates = *msg;
}
//main euler angles simulate callback function 
geometry_msgs::Point sim_main_euler_angles;
void main_euler_angles_sim_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	sim_main_euler_angles = *msg;
}
//main position simulate callback function 
geometry_msgs::Point sim_main_position;
void main_position_sim_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	sim_main_position = *msg;
}
//main velocity simulate callback function 
geometry_msgs::Point sim_main_velocity;
void main_velocity_sim_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	sim_main_velocity = *msg;
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
	main_eul_ang.y=-pitch;
	main_eul_ang.z=-yaw+pi/2;
	//ROS_INFO_STREAM("x: "<<main_eul_ang.x);
	//ROS_INFO_STREAM("y: "<<main_eul_ang.y);
	//ROS_INFO_STREAM("z: "<<main_eul_ang.z);
	return main_eul_ang;
} 


//tran PX4 attitude to controller input, velocity
geometry_msgs::Point make_main_velocity(geometry_msgs::TwistStamped local_vel)
{
	geometry_msgs::Point main_velocity;
	main_velocity.x = local_vel.twist.linear.y;
	main_velocity.y = -local_vel.twist.linear.x;
	main_velocity.z = -local_vel.twist.linear.z;
	return main_velocity;
}


//tran PX4 attitude to controller input, body_rates
geometry_msgs::Point make_main_body_rates(geometry_msgs::TwistStamped local_vel)
{
	geometry_msgs::Point main_body_rates;
	main_body_rates.x = local_vel.twist.angular.x;
	main_body_rates.y = local_vel.twist.angular.y;
	main_body_rates.z = local_vel.twist.angular.z;
	return main_body_rates;
}

//init this param zero
geometry_msgs::Point init_euler_angles_test;
void timercallback(const ros::TimerEvent&)
{  

	reset_conroller.data = 0;

	//psi_bias_1_pub.publish(0);
	if( sw_arm > 1500)
		{
			//ROS_INFO_STREAM("arm");
			arm_bool.data=true;
			autoland_bool.data = false;
			normal_bool.data = false;
		}
		// else if((sw_auto_land > 1500 && sw_arm > 1500)||(sw_auto_land > 1500 && sw_arm < 1500))
		// {
		// 	//take off switch
		// 	//ROS_INFO_STREAM("takeoff");
		// 	arm_bool.data=false;
		// 	autoland_bool.data = true;
		// 	normal_bool.data = false;

		// }
		else if( sw_arm < 1500)
		{
			//normal crc switch
			//ROS_INFO_STREAM("normal");
			arm_bool.data=false;
			autoland_bool.data = false;
			normal_bool.data = true;
		}
		//judge if the thrust is up to middle
        if(sw_fly < 1500 )
		{if(arm_bool.data==true && thrust_last_time <= MID && thrust >= MID)
		{
			//ROS_INFO_STREAM("armed and the uav will fly.. pass MID");
			let_fly = true;
			init_controller = true;
			count ++ ;//init only the first through middle
			
		}
		else if (arm_bool.data==false /* condition */)
		{
			/* code */
			let_fly = false;
			init_controller = false;
			count = 0;

			reset_conroller.data = 1;

		}
        }
        else if(sw_fly > 1500)
        {if(arm_bool.data==true )
		{
			//ROS_INFO_STREAM("armed and the uav will fly..");
			let_fly = true;
			init_controller = true;
			//count ++ ;//init only the first through middle
			
		}
		else if (arm_bool.data==false /* condition */)
		{
			/* code */
			let_fly = false;
			init_controller = false;
			count = 0;

			reset_conroller.data = 1;

		}
        }
		//init main,cmd,and setpoinit
		// ROS_INFO_STREAM("init controller ...."<< init_controller);
		// ROS_INFO_STREAM("init count ...."<< count);
		//ROS_INFO_STREAM(" init_pos_flag ...."<< init_pos_flag);
		if (init_pos_flag > 1900){
			count += 1;
		}
		if (init_pos_flag < 1900){
			count = 0;
		}
		if(init_controller && count == 1 && init_pos_flag > 1900)
		{
			//close the init controller
			init_controller = false;
			count += 1;
			ROS_INFO_STREAM("init controller ....");
			ROS_INFO_STREAM("count " << count);
			ROS_INFO_STREAM(" init_pos_flag ...."<< init_pos_flag);
			//get the main body's data
			ROS_INFO_STREAM("local_pos z is " << local_pos);
			main_position = make_main_position(local_pos);
			main_pos_ned  = enu2ned_pos(main_position);
			
			main_eular_angles = make_eular_angles(local_pos);
			main_eular_angles_ned = enu2ned_euler(main_eular_angles);

			main_velocity = make_main_velocity(local_vel);
			main_velocity_ned = enu2ned_vel(main_velocity);

			main_body_rates=make_main_body_rates(local_vel);
			main_body_rates_ned = enu2ned_euler_rate(main_body_rates);

			nominal_position_last.x = main_position.x;
			nominal_position_last.y = main_position.y;
			nominal_position_last.z = main_position.z;
			nominal_euler_angles_last.x=0;
			nominal_euler_angles_last.y=0;
			nominal_euler_angles_last.z=main_eular_angles_ned.z;

			init_body_rate.x=0;
			init_body_rate.y=0;
			init_body_rate.z=0;

			init_euler_angles.x=main_eular_angles_ned.x;
			init_euler_angles.y=main_eular_angles_ned.y;
			init_euler_angles.z=main_eular_angles_ned.z;
			//init_euler_angles = enu2ned_euler(init_euler_angles);

			init_velocity.x=0;
			init_velocity.y=0;
			init_velocity.z=0;

			init_position.x= main_position.y;
			init_position.y= main_position.x;
			init_position.z=-main_position.z;


			//init setpoint to controller
			nominal_position_last = enu2nwu_pos(nominal_position_last);
			nominal_euler_angles_last = enu2ned_euler(nominal_euler_angles_last);

			nominal_position_pub.publish(nominal_position_last);
			nominal_eular_angles_pub.publish(nominal_euler_angles_last);

			//init cmd to controller
			init_euler_angles_cmd_pub.publish(init_euler_angles);
			init_velocity_cmd_pub.publish(init_velocity);
			init_pos_cmd_pub.publish(init_position);
			init_body_rates_cmd_pub.publish(init_body_rate);


			nominal_position_last.x = main_position.x;
			nominal_position_last.y = main_position.y;
			nominal_position_last.z = main_position.z;
			nominal_euler_angles_last.x=0;
			nominal_euler_angles_last.y=0;
			nominal_euler_angles_last.z=main_eular_angles.z;
			//main_position = make_main_position(local_pos);

			node1_yaw.data=0;
			node2_yaw.data=0;
			node3_yaw.data=0;		
			
			psi_bias_1_pub.publish(node1_yaw);
			psi_bias_2_pub.publish(node2_yaw);
			psi_bias_3_pub.publish(node3_yaw);

			main_position_pub.publish(main_pos_ned);
			main_eular_angles_pub.publish(main_eular_angles_ned);
			main_velocity_pub.publish(main_velocity_ned);
			main_body_rates_pub.publish(main_body_rates_ned);
		}

		//normol function
		if(let_fly && init_pos_flag > 1900)//aim to protect the vehicle when fly
		{
			//enter the postion RC mode
			// if(switch_position_attitude>1500)
			// {
				// tran sw to vel
				// ROS_INFO_STREAM(" init_pos_flag ...."<< init_pos_flag);
				sw.sw_map_vel(switch_yaw_xyz);
				// ROS_INFO_STREAM("vel : "<<sw.vel[0]);
				sw.calc_setpoint(nominal_position_last,nominal_euler_angles_last,sw.vel,0.002);
				nominal_position_last = sw.pos_setpoint;
				nominal_euler_angles_last = sw.att_setpoint;

				
			// }
			// else if(switch_position_attitude<1500)//enter the attitude RC mode 
			// {
			// 	// tran sw to vel
			// 	sw.sw_map_ang_vel(switch_yaw_xyz);
			// 	// ROS_INFO_STREAM("vel : "<<sw.vel[0]);
			// 	sw.calc_setpoint_attitude(nominal_position_last,nominal_euler_angles_last,sw.vel,0.002);
			// 	nominal_position_last = sw.pos_setpoint;
			// 	nominal_euler_angles_last = sw.att_setpoint;

			// }
			//trans the enu ot ned sueset
			//ROS_INFO_STREAM("get adding...");
			geometry_msgs::Point nominal_pos_nwu;
			nominal_pos_nwu = enu2nwu_pos(sw.pos_setpoint);
			nominal_eular_angles_ned = enu2ned_euler(sw.att_setpoint);

			nominal_position_pub.publish(nominal_pos_nwu);
	
			nominal_eular_angles_pub.publish(nominal_eular_angles_ned);
			// ROS_INFO_STREAM("forward and backward: "<<nominal_pos_nwu.x);
			// ROS_INFO_STREAM("lefe and right: "<<nominal_pos_nwu.y);
			// ROS_INFO_STREAM("up and down: "<<nominal_pos_nwu.z);
			//ROS_INFO_STREAM("yaw: "<<nominal_eular_angles_ned.z);
			// ROS_INFO_STREAM("roll: "<<nominal_eular_angles_ned.x);
			// ROS_INFO_STREAM("pitch: "<<nominal_eular_angles_ned.y);
			// ROS_INFO_STREAM("yaw: "<<nominal_eular_angles_ned.z);
			//decete if all child nodes are armd
			//if(is_all_armd.data)
			//{
			//publish the real position,velocity,eular angles,eular rates
			main_position = make_main_position(local_pos);
			main_pos_ned  = enu2ned_pos(main_position);
			main_position_pub.publish(main_pos_ned);
			
			main_eular_angles = make_eular_angles(local_pos);
			main_eular_angles_ned = enu2ned_euler(main_eular_angles);
			main_eular_angles_pub.publish(main_eular_angles_ned);

			// ROS_INFO_STREAM("eular angles x: "<<main_eular_angles.x);
			// ROS_INFO_STREAM("eular angles y: "<<main_eular_angles.y);
			// ROS_INFO_STREAM("eular angles z: "<<main_eular_angles.z);
			main_velocity = make_main_velocity(local_vel);
			main_velocity_ned = enu2ned_vel(main_velocity);
			main_velocity_pub.publish(main_velocity_ned);


			main_body_rates=make_main_body_rates(local_vel);
			main_body_rates_ned = enu2ned_euler_rate(main_body_rates);
			main_body_rates_pub.publish(main_body_rates_ned);


			//publish messages to simulate ros node, plant
			/**
			 * geometry_msgs::Point sim_main_body_rates;
			 * geometry_msgs::Point sim_main_euler_angles;
			 * geometry_msgs::Point sim_main_position;
			 * geometry_msgs::Point sim_main_velocity;
			 ***/


			//  main_body_rates_pub.publish(sim_main_body_rates);
			//  main_eular_angles_pub.publish(sim_main_euler_angles);
			//  main_position_pub.publish(sim_main_position);
			//  main_velocity_pub.publish(sim_main_velocity);
			//  ROS_INFO_STREAM("set x: "<<sw.pos_setpoint.x<<" simu_x: "<<sim_main_position.y);
			//  ROS_INFO_STREAM("set y: "<<sw.pos_setpoint.y<<" sim_y: "<<sim_main_position.x);
			//  ROS_INFO_STREAM("set z: "<<sw.pos_setpoint.z<<" sim_z: "<<-sim_main_position.z);
			//  ROS_INFO_STREAM("set yaw: "<<sw.att_setpoint.z<<" sim_yaw: "<<-sim_main_euler_angles.z+1.5708);

			//}
			//else
			//{
				/*list which child node is not armd*/
				//ROS_INFO_STREAM("not armd node id : ");
			//}
		}
		else
		{
			/* code */
			ROS_INFO_STREAM("not arm..");

		}
		start_pub_att.data = let_fly;
		control_start_pub_att.publish(start_pub_att);
		first_pose_pub.publish(first_pose); 
	
		// arm_child_flight.publish(arm_bool);
		// takeoff_pub.publish(autoland_bool);
		// normal_pub.publish(normal_bool);

		reset_controller_pub.publish(reset_conroller);
		//storage the current thrust as the lastest volume
		thrust_last_time = thrust;
}

//main function 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "get_master_make_m");
	ros::NodeHandle nh;

	/**subscribe list**/
	ros::Subscriber state_sub     = nh.subscribe<mavros_msgs::State>
		("/mavros/state", 10, state_cb);
	//subscribe manual control
    ros::Subscriber get_rc_sub = nh.subscribe<mavros_msgs::ManualControl>
		("/mavros/manual_control/control",100,get_rc_setpoint_cb);
	//subacribe RC channels value
	ros::Subscriber get_rc_channel_sub = nh.subscribe<mavros_msgs::RCIn>
		("/mavros/rc/in",100,get_rc_channel_cb);
	//subscribe if all child nodes have already connected to PX4
	ros::Subscriber is_all_connected_sub = nh.subscribe<std_msgs::Bool>
		("is_all_connected",10,is_all_connected_cb);
	//subscribe if all child nodes have already armd
	ros::Subscriber is_all_armd_sub = nh.subscribe<std_msgs::Bool>
		("is_all_armd",10,is_all_armd_cb);
	//subscribe local position
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
		("/mavros/local_position/pose", 10, local_pos_cb);
	//subscriber local_velocity , NWU frame of body rate
	ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
		("/mavros/local_position/velocity_body",10,local_vel_cb);
	


	//simulation of the ros node message 
	ros::Subscriber main_body_rates_sim_sub = nh.subscribe<geometry_msgs::Point>
		("/main_body_rates_sim",100,main_body_rates_sim_sub_cb);
	ros::Subscriber main_euler_angles_sim_sub = nh.subscribe<geometry_msgs::Point>
		("/main_euler_angles_sim",100,main_euler_angles_sim_sub_cb);
	ros::Subscriber main_position_sim_sub = nh.subscribe<geometry_msgs::Point>
		("/main_position_sim",100,main_position_sim_sub_cb);
	ros::Subscriber main_velocity_sim_sub = nh.subscribe<geometry_msgs::Point>
		("/main_velocity_sim",100,main_velocity_sim_sub_cb);


	message_filters::Subscriber<geometry_msgs::PoseStamped> subscriber_pose(nh,"/uav1/mavros/local_position/pose",1000,ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<geometry_msgs::TwistStamped> subscriber_twist(nh,"/uav1/mavros/local_position/velocity_body",1000,ros::TransportHints().tcpNoDelay());
    
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped,geometry_msgs::TwistStamped> syncPolicy;
    //message_filters::TimeSynchronizer<sensor_msgs::LaserScan,geometry_msgs::PoseWithCovarianceStamped> sync(subscriber_laser, subscriber_pose, 10);
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), subscriber_pose, subscriber_twist);  
    sync.registerCallback(boost::bind(&multi_sensor_callback, _1, _2));


	/**publish list**/
	//advertise a bool param, named child/arming...
	// arm_child_flight    = nh.advertise<std_msgs::Bool>("child/arming",10);
	// //child/autoland
	// takeoff_pub = nh.advertise<std_msgs::Bool>("child/takeoff",10);
	// //child/kill
	// normal_pub = nh.advertise<std_msgs::Bool>("child/normal",10);

	//nominal_position
	nominal_position_pub = nh.advertise<geometry_msgs::Point>("/nominal_position",100);
	//nominal_eular_angles
	nominal_eular_angles_pub = nh.advertise<geometry_msgs::Point>("/nominal_euler_angles",100);
	//controller input of main_position
	main_position_pub = nh.advertise<geometry_msgs::Point>("/main_position",100);
	//controller input of main_velocity
	main_velocity_pub = nh.advertise<geometry_msgs::Point>("/main_velocity",100);
	//controller input of main_eular_angles
	main_eular_angles_pub = nh.advertise<geometry_msgs::Point>("/main_euler_angles",100);
	//controller input of main_body_rates
	main_body_rates_pub = nh.advertise<geometry_msgs::Point>("/main_body_rates",100);
	//first time pose 
	first_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/first_pose",100);
	
	//init cmd publish: euler angles
	init_euler_angles_cmd_pub = nh.advertise<geometry_msgs::Point>("/init_euler_angles_cmd",100);
	//init cmd publish: position
	init_pos_cmd_pub = nh.advertise<geometry_msgs::Point>("/init_position_cmd",100);
	//init cmd publish: body_rates
	init_body_rates_cmd_pub = nh.advertise<geometry_msgs::Point>("/init_body_rates_cmd",100);
	//init cmd publish: velocity
	init_velocity_cmd_pub = nh.advertise<geometry_msgs::Point>("/init_velocity_cmd",100);
	
	//control switch of when publish attitude to child node
	control_start_pub_att = nh.advertise<std_msgs::Bool>("start_pub_att",100);
	psi_bias_1_pub=nh.advertise<std_msgs::Float64>("/psi_bias_1",10);
	psi_bias_2_pub=nh.advertise<std_msgs::Float64>("/psi_bias_2",10);
	psi_bias_3_pub=nh.advertise<std_msgs::Float64>("/psi_bias_3",10);	

	//publish the reset topic
	reset_controller_pub=nh.advertise<std_msgs::Float64>("/controller_reset",10);
	//test line,open the armd section
	is_all_armd.data=true;
	
	//wait the px4 flight to be connected 
	ros::Rate rate(500.0);
	//ROS_INFO("current_state_connected is %d",current_state.connected);
	// while (ros::ok() && !current_state.connected && !is_all_connected.data) {
	// 	ROS_INFO("px4 is unconnected...");
	// 	ros::spinOnce();
	// 	rate.sleep();
	// }

	//ros::Timer timer = nh.createTimer(ros::Duration(0.005),boost::bind(&timercallback,_1,serial_pub));
	ros::Timer timer = nh.createTimer(ros::Duration(0.004),timercallback);
	
	arm_bool.data =false;//init arm_bool to false to ensure safety
	autoland_bool.data = false;
	normal_bool.data = false;

	//ros::spinOnce();
	//rate.sleep();
	ros::spin();
	return 0;
}
