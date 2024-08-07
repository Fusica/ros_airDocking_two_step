#include "./../include/offboard/include.h"
/*****************************************/
//register pulishher and subscriber
ros::Subscriber local_pos_sub;
//mavros state
ros::Subscriber state_sub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
//RC channel pwm value
ros::Subscriber get_rc_channel_sub;
//ground-truth for airsim
ros::Subscriber airsim_gt_sub;
//Arm state
ros::Publisher	control_start_pub_att;

//target position in global frame for mpc controller
ros::Publisher	nominal_position_pub;
//target euler in global frame for mpc controller
ros::Publisher nominal_eular_angles_pub;
//interested point in global frame for perception aware mpc
ros::Publisher interest_pub;
//(x,y) corresponding to (u,v) in camera normalized frame, z is the target yaw(degree) in global frame computing by the interested point and local position. 
ros::Publisher perception_pub;
//yolo vekf
ros::Subscriber vekf;


/*****************************************/
/***
 * pos: target potion for mpc in global frame
 * original_pos: original position
 ***/ 
geometry_msgs::Point pos,original_pos;
//target attitude (roll pitch yaw)
geometry_msgs::Point quat;
std_msgs::Bool arm;
mutex m_buf;
/*****************************************/
int RC_Control = -1;
//trajectory setting values
int Rate = 10;
int trajector_mode = 0;
float R = 1;
int step = 0;
int sametimes = 0;
int time_body = 0;
double xita = 10; //   degree/s default
double xita_loop; // du/loop loop_rate = 10hz
int loop_n; //loop number
double to_rad = 3.1415926/180.0;
double to_degree = 1.0/to_rad;
//threshold
double th_x = 0.4;
double th_y = 0.4;
double th_z = 0.4;
//end point
double end_z = 1.5;
double add_speed = 0.02;
//numble of circle loop
int circle_n = 1;

/*****************************************/
//calculating (u,v) in camera normalized frame 
double p_x, p_y, p_z;//drone
double q_w, q_x, q_y, q_z;
double p_F_x, p_F_y, p_F_z;//position 
double t_B_C_x, t_B_C_y, t_B_C_z;
double q_B_C_w, q_B_C_x, q_B_C_y, q_B_C_z;
double epsilon = 0.1;
//test
Vector3d p_drone, p_roa, p_bc;
Quaterniond q_drone, q_bc;
//relative to original point in yolo sim
double uav_target_x = 3;
double uav_target_y = 0;
double uav_target_z = 1.5;
//ground truth of uav form airsim NED
Quaterniond q_gt = Quaterniond::Identity();
Vector3d t_gt(0, 0, 0);
Matrix3d R_ned_enu, R_enu_ned, R_FRD_FLU;//R_ned_enu = R_enu_ned
Matrix3d KR_cb, K_in, K_in_inv;
Vector3d Kt_cb;
Vector3d u_v_1;//ground truth of yolo output
Vector3d u_v_1_n;//add noise
//noise for yolo result (u,v)
const double mean = 0.0;//均值
const double stddev = 3;//标准差 pixle
std::default_random_engine generator;
std::normal_distribution<double> dist(mean, stddev);


/*****************************************/
//callback function of get_rc_channel_cb
mavros_msgs::RCIn get_rc_channel;
sw_mapping sw;
int sw_7_arm = 0;
int firstlow = 0;
int sw_kill=0;
int offboard_start = 0;

int sw_fly=5000;
//thrust is same as a channel of switch_yaw_xyz[i]
int thrust = 0;
int thrust_last_time=0;
int switch_yaw_xyz[4];//xyz,yaw
bool init_controller = false; //bool init controller
bool let_fly = false;//bool fly switch

//define a val to record the switch mode of postion and attitude
//int switch_position_attitude=0;
void get_rc_channel_cb(const mavros_msgs::RCIn::ConstPtr& msg)
{
	get_rc_channel = *msg;
	sw_7_arm = get_rc_channel.channels[9];
	offboard_start = get_rc_channel.channels[4];
    sw_fly = 1200;
	//map a channel of RC to the val, the channel number should be changed 
	//switch_position_attitude = get_rc_channel.channels[5];

	//get the thrust volume
	thrust_last_time = thrust;
	thrust = get_rc_channel.channels[2];
	//ROS_INFO_STREAM("thrust "<< thrust);
	m_buf.lock();
	for(int i =0 ; i<4; i++)
	{
		switch_yaw_xyz[i]=get_rc_channel.channels[i];
	}
	m_buf.unlock();
	if(sw_7_arm < 1500)
	{
		firstlow = 1;
	}
}


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	current_state = *msg;
	if(current_state.armed){
		arm.data = true;
	}else{
		arm.data = false;
	}
}

//callback of local_position topic
geometry_msgs::PoseStamped local_pos, local_pos_cur;
geometry_msgs::PointStamped interest_pos, perception_uv, line_point1, line_point2;
Vector3d p_w_target;
list<geometry_msgs::PoseStamped> pos_buffer;


int n_buffer = 100;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	local_pos = *msg;
	m_buf.lock();
	if(pos_buffer.size()>n_buffer){
		pos_buffer.pop_front();
	}
	pos_buffer.push_back(local_pos);
	m_buf.unlock();
}
bool target_pose = false;


/*Odometry in NED frame(global) wrt take-off point*/
//todo:目前只适配了RC_process
//airsim中的机体坐标系是前右下
bool yolo_sim = false;//yolo模拟器
double original_gt_x, original_gt_y, original_gt_z;
bool first_call_gt = true;
void airsim_gt_cal(const nav_msgs::Odometry::ConstPtr& msg){
	if(first_call_gt){
		original_gt_x = msg->pose.pose.position.x;
		original_gt_y = msg->pose.pose.position.y;
		original_gt_z = msg->pose.pose.position.z;
		first_call_gt = false;
	}
	target_pose = true;
	yolo_sim = true;
	q_gt = Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
	//ROS_INFO_STREAM("R = "<<"\n"<<q_gt.toRotationMatrix().matrix());
	q_gt = q_gt*R_FRD_FLU;//机体坐标系从前右下转 到 前左上
	//ROS_INFO_STREAM("R = "<<"\n"<<q_gt.toRotationMatrix().matrix()); 
	t_gt = Vector3d(msg->pose.pose.position.x - original_gt_x, msg->pose.pose.position.y - original_gt_y, msg->pose.pose.position.z - original_gt_z);
	//ROS_INFO_STREAM("p = "<<t_gt.matrix());
	Eigen::Vector3d target_w_enu = Vector3d(uav_target_x, uav_target_y, uav_target_z);
	//ROS_INFO_STREAM("target in enu = ("<<target_w_enu.x()<<", "<<target_w_enu.y()<<", "<<target_w_enu.z()<<")");
	Eigen::Vector3d target_w_ned =  R_ned_enu*target_w_enu;
	//ROS_INFO_STREAM("target in ned = ("<<target_w_ned.x()<<", "<<target_w_ned.y()<<", "<<target_w_ned.z()<<")");
	Eigen::Vector3d target_body = (q_gt.inverse()*target_w_ned - q_gt.inverse()*t_gt);
	//ROS_INFO_STREAM("target in body = ("<<target_body.x()<<", "<<target_body.y()<<", "<<target_body.z()<<")");
	//todo:验证下面的坐标系
	//ROS_INFO_STREAM("R_bc = "<<"\n"<<q_bc.toRotationMatrix().matrix());
	Eigen::Vector3d target_nomalized_cam = q_bc.inverse()*target_body -q_bc.inverse()*p_bc;
	//ROS_INFO_STREAM("target in cam = ("<<target_nomalized_cam.x()<<", "<<target_nomalized_cam.y()<<", "<<target_nomalized_cam.z()<<")");
	u_v_1 = KR_cb*target_body + Kt_cb;
	u_v_1 = u_v_1/u_v_1.z();
	//if()
	//ROS_INFO_STREAM("(u, v) = ("<<u_v_1.x()<<", "<<u_v_1.y()<<")" );
 	u_v_1_n = u_v_1 + Vector3d(dist(generator), dist(generator), 0);
	//ROS_INFO_STREAM("noise (u, v) = ("<<u_v_1_n.x()<<", "<<u_v_1_n.y()<<")" );
}


/*****************************************/
//define setpoint last variables for integral last setpoint
geometry_msgs::Point nominal_position_last;//
geometry_msgs::Point nominal_euler_angles_last;	

geometry_msgs::Point main_position;
geometry_msgs::Point main_eular_angles;

geometry_msgs::Point main_pos_ned;
geometry_msgs::Point main_eular_angles_ned;

void Process(){
// 	/***************************************************/
// 	//initialization
	ros::Rate rate(Rate);
	ROS_INFO_STREAM("Pricess Enter ");
// 	//channal 7
 	while (ros::ok())
 	{
		ROS_INFO_STREAM("sw_7_arm is "<< sw_7_arm);
 		if (sw_7_arm > 1500 )
 			break;
 		rate.sleep();
 	}

	//calculate the average of local_position
	double ave_x = 0;
	double ave_y = 0;
	double ave_z = 0;
	double ave_yaw = 0;
	m_buf.lock();
	for(auto pose_local: pos_buffer){
		ave_x += pose_local.pose.position.x;
		ave_y += pose_local.pose.position.y;
		ave_z += pose_local.pose.position.z;
		geometry_msgs::Point rpy;
		rpy = make_eular_angles(pose_local);
		ave_yaw += rpy.z;
	}
	m_buf.unlock();
	original_pos.x = ave_x/n_buffer;
	original_pos.y = ave_y/n_buffer;
	original_pos.z = ave_z/n_buffer;
	ave_yaw = ave_yaw/n_buffer;


	ROS_INFO_STREAM("original point"<<original_pos);
// 	ROS_INFO_STREAM("original yaw = "<<ave_yaw<< " degree   local_pose_yaw = "<<local_rpy.z);
	pos = original_pos;
// 	// nominal_eular_angles_pub.publish(local_rpy);
	nominal_position_pub.publish(pos);

	line_point1.point.y = original_pos.y + R;
	line_point1.point.x = original_pos.x;
	line_point2.point.y = original_pos.y - R;
	line_point2.point.x = original_pos.x;
 
	ROS_INFO_STREAM("SW_7 "<<sw_7_arm);
	ROS_INFO_STREAM("offboard_start "<< offboard_start);
	while (sw_7_arm > 1500 )
	{
		if(offboard_start > 1900) {
    
			control_start_pub_att.publish(arm);
			local_pos_cur = local_pos;
			circle_n--;
		
			ROS_INFO("Taking off!");
			ROS_INFO_STREAM("local_pos_cur.pose.position.z "<<local_pos_cur.pose.position.z);
			ROS_INFO_STREAM("original_pos.z "<<original_pos.z);
			ROS_INFO_STREAM("end_z "<<end_z);
			ROS_INFO_STREAM("th_z "<<th_z);
			ROS_INFO_STREAM("pos.z "<<pos.z);

			ROS_INFO("Flying circle trajectory!");
			ROS_DEBUG_STREAM("local_y = "<<local_pos_cur.pose.position.y);
			
			ROS_DEBUG_STREAM("xita ="<<xita_loop*time_body);
			pos.x = (original_pos.x+R) + R*cos((xita_loop*time_body+180)*to_rad);//起点在圆上
			pos.y = (original_pos.y)+R*sin((xita_loop*time_body+180)*to_rad);
			pos.z = original_pos.z+end_z;
			if(time_body < loop_n)
			{time_body++;}
		}
		nominal_position_pub.publish(pos);
		rate.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "offb_node");
	ros::NodeHandle nh("~");
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
	//local posisition from mavros
	local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
		("/mavros/local_position/pose", 10, local_pos_cb);
	//mavros state
	state_sub = nh.subscribe<mavros_msgs::State>
		("/mavros/state", 10, state_cb);
	arming_client = nh.serviceClient<mavros_msgs::CommandBool>
		("/mavros/cmd/arming");
	set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
		("/mavros/set_mode");
	//RC channel pwm value
	get_rc_channel_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in",100,get_rc_channel_cb);
	//gt from airsim
	airsim_gt_sub = nh.subscribe<nav_msgs::Odometry>("/airsim_node/PX4/odom_local_ned",100,airsim_gt_cal);
	//Arm state
	control_start_pub_att = nh.advertise<std_msgs::Bool> ("/start_pub_att",100);

	/**********************/
	//target position in global frame for mpc controller
	nominal_position_pub = nh.advertise<geometry_msgs::Point>("/nominal_position",100);
	//target euler in global frame for mpc controller
	nominal_eular_angles_pub = nh.advertise<geometry_msgs::Point>("/nominal_euler_angles",100);
	//interested point in global frame for perception aware mpc
	interest_pub = nh.advertise<geometry_msgs::PointStamped>("/mpc/point_of_interest",100);
	//(x,y) corresponding to (u,v) in camera normalized frame, z is the target yaw(degree) in global frame computing by the interested point and local position. 
	perception_pub = nh.advertise<geometry_msgs::PointStamped>("/perception",100);
	//yolo
	//vekf = nh.subscribe<geometry_msgs::PointStamped>("/vekf",100,VEKF);
	

	pos.x = 0;
	pos.y = 0;
	pos.z = 0;
	original_pos.x = 0;
	original_pos.y = 0;
	original_pos.z = 0;
	quat.x = 0;
	quat.y = 0;
	quat.z = 0;
	
	//read parameters form yalm file
	string setting_file;
	setting_file = readParam<string>(nh, "setting");
	ROS_INFO_STREAM("setting path = "<<setting_file);
	cv::FileStorage fsSettings(setting_file.c_str(), cv::FileStorage::READ);
	fsSettings["RC_Control"]>>RC_Control;
    if(RC_Control == 0){
		ROS_INFO("Fly trajectory ");
	}else if(RC_Control == 1){
		ROS_INFO("Fly with RC");
	}else{
		ROS_ERROR("Wrong Mode!!");
		return 0;
	}
	fsSettings["trajectory_mode"]>>trajector_mode;
    if(trajector_mode == 0){
		ROS_INFO_STREAM("trajectory mode "<<trajector_mode<<" : Start on the original of circle");
	}else if(trajector_mode == 1){
		ROS_INFO_STREAM("trajectory mode "<<trajector_mode<<" : Start on the circle");
	}else if(trajector_mode == 2){
		ROS_INFO_STREAM("trajectory mode "<<trajector_mode<<" : Line trajectory");
	}else{
		ROS_ERROR("Wrong Mode!!");
		return 0;
	}
	fsSettings["XITA"]>>xita;
    ROS_INFO_STREAM("xita: "<<xita<<" degree/s");
	//ROS
	fsSettings["R"]>>R;
    ROS_INFO_STREAM("R: "<<R<<" m");
	xita_loop = xita/Rate; // du/loop loop_rate = 10hz
	loop_n = 360/xita_loop; //loop number
	fsSettings["threshold_x"]>>th_x;
    ROS_INFO_STREAM("threshold_x: "<<th_x<<" m");
	fsSettings["threshold_y"]>>th_y;
    ROS_INFO_STREAM("threshold_y: "<<th_y<<" m");
	fsSettings["threshold_z"]>>th_z;
    ROS_INFO_STREAM("threshold_z: "<<th_z<<" m");
	fsSettings["end_z"]>>end_z;
    ROS_INFO_STREAM("end point: "<<R<<", "<<0<<", "<<end_z);
	fsSettings["add_speed"]>>add_speed;
    ROS_INFO_STREAM("add_speed: "<<add_speed<<" m/loop");
	fsSettings["circle_n"]>>circle_n;
    ROS_INFO_STREAM("circle_n: "<<circle_n);

	
	//read camera extrinsic matrix from setting file
	// Optional parameters
    //std::vector<double> p_B_C(3), q_B_C(4);
    fsSettings["p_B_C_x"]>>t_B_C_x; fsSettings["p_B_C_y"]>>t_B_C_y; fsSettings["p_B_C_z"]>>t_B_C_z;
    ROS_INFO_STREAM("p_B_C: "<<t_B_C_x<<", "<<t_B_C_y<<", "<<t_B_C_z);
	fsSettings["q_B_C_w"]>>q_B_C_w; fsSettings["q_B_C_x"]>>q_B_C_x; fsSettings["q_B_C_y"]>>q_B_C_y; fsSettings["q_B_C_z"]>>q_B_C_z;
	ROS_INFO_STREAM("q_B_C: "<<q_B_C_w<<", "<<q_B_C_x<<", "<<q_B_C_y<<", "<<q_B_C_z);

	std::thread process;
	if(RC_Control == 0){
		process = std::thread(&Process);
	}
	ros::spin();
	return 0;
}