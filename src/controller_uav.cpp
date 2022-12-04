#include "offboard_control/controller_uav.h"
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

/******************************Define***********************************/
#define ON_MEAS					1
#define ON_ERROR				0

#define PI						3.14159265
#define RADIAN_TO_DEGREE(x)		(x * (180 / PI))
#define DEGREE_TO_RADIAN(x)		(x * (PI / 180))
#define	DISTANCE_ON_MARKER(x)	x*(-1)

#define USE_MARKER				0
#define USE_BODY_OFFSET			1


using namespace Eigen;
using namespace std;

Eigen::Vector3d toEigen(const geometry_msgs::Point &p) {
	Eigen::Vector3d ev3(p.x, p.y, p.z);
	return ev3;
}

inline Eigen::Vector3d toEigen(const geometry_msgs::Vector3 &v3) {
	Eigen::Vector3d ev3(v3.x, v3.y, v3.z);
	return ev3;
}


double get_Euler_from_quat(double xq, double yq, double zq, double wq, Axis type)
{
	tf2::Quaternion q;
	double roll, pitch, yaw, res;

	q.setValue(xq, yq, zq, wq);
	tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
	switch (type) {
		case Axis::ROLL:
			res = roll;
			break;
		case Axis::PTICH:
			res = pitch;
			break;
		default:
			res = yaw;
			break;
	}

	return res;
}

int index_setpoint = 0;

velocityCtrl::velocityCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
		: nh_(nh), nh_private_(nh_private), node_state(WAITING_FOR_HOME_POSE), flight_mode(POSITION_MODE), sim_enable_(true)

{
	std::string marker_pose_topic_name;

	nh_private_.param<bool>("test_fly_waypoint", test_fly_waypoint_, false);
	nh_private_.param("marker_pose_topic", marker_pose_topic_name, std::string("/aruco_detect/pose"));

	mavposeSub_ = nh_.subscribe("mavros/local_position/pose", 1, &velocityCtrl::mavposeCallback, this, ros::TransportHints().tcpNoDelay());

	mavtwistSub_ = nh_.subscribe("mavros/local_position/velocity_local", 1, &velocityCtrl::mavtwistCallback, this,
								ros::TransportHints().tcpNoDelay());

	cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &velocityCtrl::cmdloopCallback,this);  // Define timer for constant loop rate

	setRaw_pub = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10);

	target_pose = nh_.subscribe("/target_pos", 10, &velocityCtrl::targetPositioncallback, this, ros::TransportHints().tcpNoDelay());

	setpoint_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

	statusloop_timer_ = nh_.createTimer(ros::Duration(0.1), &velocityCtrl::statusloopCallback, this);

	land_service_ = nh_.advertiseService("land", &velocityCtrl::landCallback, this);

	velocity_pub_   = nh_.advertise <geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10 );

	term_PID_z = nh_.advertise<geometry_msgs::Vector3>("/pid_term_z", 10);

	debug_target = nh_.advertise<geometry_msgs::Vector3> ("/debug_term", 10);

	marker_pose_sub = nh_.subscribe
		(marker_pose_topic_name, 1, &velocityCtrl::ReceivedMarkerPose_Callback, this, ros::TransportHints().tcpNoDelay());

	decrese_height_sub = nh_.subscribe
		("/decrease_height", 1, &velocityCtrl::CheckAllowDecreaseHeight_Callback, this, ros::TransportHints().tcpNoDelay());

	start_land_service_ = nh_.advertiseService("start_landing", &velocityCtrl::EnableLand_Service, this);

	nh_private_.param<double>("init_pos_x", initTargetPos_x_, 0.0);
	nh_private_.param<double>("init_pos_y", initTargetPos_y_, 0.0);
	nh_private_.param<double>("init_pos_z", initTargetPos_z_, 1.0);
	nh_private_.param<double>("init_yaw", initTarget_yaw_, 0.0);

	nh_private_.param<double>("Kp_x", kpx_, 1.4);
	nh_private_.param<double>("Kd_x", kdx_, 1.1);
	nh_private_.param<double>("Ki_x", kix_, 0.3);

	nh_private_.param<double>("Kp_y", kpy_, 1.4);
	nh_private_.param<double>("Kd_y", kdy_, 1.1);
	nh_private_.param<double>("Ki_y", kiy_, 0.3);

	nh_private_.param<double>("Kp_z", kpz_, 0.7);
	nh_private_.param<double>("Kd_z", kdz_, 0.52);
	nh_private_.param<double>("Ki_z", kiz_, 0.2);

	nh_private_.param<double>("kp_yaw_", kpyaw_, 0.5);
	nh_private_.param<double>("kd_yaw_", kdyaw_, 0.5);
	nh_private_.param<double>("ki_yaw_", kiyaw_, 0.0);

	nh_private_.param<double>("max_out", max_out_, 0.2);
	nh_private_.param<double>("min_out", min_out_, -0.2);

	/* Get param set points */
	if (test_fly_waypoint_) {
		nh_.getParam("/point1", setPoint_[0]);
		nh_.getParam("/point2", setPoint_[1]);
		nh_.getParam("/point3", setPoint_[2]);
		nh_.getParam("/point4", setPoint_[3]);

		ROS_INFO_STREAM("Set Point 1: x: " << setPoint_[0].at(0) << " y: " << setPoint_[0].at(1) << " z: " << setPoint_[0].at(2) << " yaw: " << setPoint_[0].at(3));
		ROS_INFO_STREAM("Set Point 2: x: " << setPoint_[1].at(0) << " y: " << setPoint_[1].at(1) << " z: " << setPoint_[1].at(2) << " yaw: " << setPoint_[1].at(3));
		ROS_INFO_STREAM("Set Point 3: x: " << setPoint_[2].at(0) << " y: " << setPoint_[2].at(1) << " z: " << setPoint_[2].at(2) << " yaw: " << setPoint_[2].at(3));
		ROS_INFO_STREAM("Set Point 4: x: " << setPoint_[3].at(0) << " y: " << setPoint_[3].at(1) << " z: " << setPoint_[3].at(2) << " yaw: " << setPoint_[3].at(3));
	}

	targetPos_ << initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;

	targetYaw_ = initTarget_yaw_;

	PID_x.setKp(kpx_);
	PID_x.setKd(kdx_);
	PID_x.setKi(kix_);
	PID_x.setUMax(max_out_);
	PID_x.setUMin(min_out_);
	PID_x.SetModeP(ON_ERROR);
	PID_x.SetModeD(ON_ERROR);


	PID_y.setKp(kpy_);
	PID_y.setKd(kdy_);
	PID_y.setKi(kiy_);
	PID_y.setUMax(max_out_);
	PID_y.setUMin(min_out_);
	PID_y.SetModeP(ON_ERROR);
	PID_y.SetModeD(ON_ERROR);

	PID_z.setKp(kpz_);
	PID_z.setKd(kdz_);
	PID_z.setKi(kiz_);
	PID_z.setUMax(max_out_);
	PID_z.setUMin(min_out_);
	PID_z.SetModeP(ON_ERROR);
	PID_z.SetModeD(ON_ERROR);

	PID_yaw.setKp(kpyaw_);
	PID_yaw.setKd(kdyaw_);
	PID_yaw.setKi(kiyaw_);
	PID_yaw.setUMax(0.1);
	PID_yaw.setUMin(-0.1);
	PID_yaw.SetModeP(ON_ERROR);
	PID_yaw.SetModeD(ON_ERROR);

	error = 0.15;

	cam2drone_matrix_ << 0.0 , -1.0 , 0.0 , -1.0 , 0.0 , 0.0 , 0.0 , 0.0 , -1.0;
}

void velocityCtrl::ReceivedMarkerPose_Callback(const geometry_msgs::PoseStamped &msg){

	Eigen::Vector3d markerInCamFrame;
	/* Marker ----> Drone Frame*/
	markerInCamFrame << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;

	markerPosInBodyFrame_ = cam2drone_matrix_ * markerInCamFrame;

	// Round cm
	markerPosInBodyFrame_(0) = round(markerPosInBodyFrame_(0)*100) / 100;
	markerPosInBodyFrame_(1) = round(markerPosInBodyFrame_(1)*100) / 100;
	markerPosInBodyFrame_(2) = round(markerPosInBodyFrame_(2)*100) / 100;

	targetPosPredict_(0) = markerPosInBodyFrame_(0);
	targetPosPredict_(1) = markerPosInBodyFrame_(1);

	range_err = sqrt(pow(markerPosInBodyFrame_(0), 2) + pow(markerPosInBodyFrame_(1), 2));

	// ROS_INFO_STREAM("Distance to Marker: " << markerPosInBodyFrame_);
	marker_pose_status = RECEIVED_POSE;
	// std::cout << "point des" << point_des << std::endl;
};

void velocityCtrl::targetPositioncallback(const geometry_msgs::PoseStamped &msg){

	targetPos_ = toEigen(msg.pose.position);

}

void velocityCtrl::CheckAllowDecreaseHeight_Callback(const std_msgs::Bool &msg){

	AllowDecreaseHeight_ = msg.data;
}

bool velocityCtrl::EnableLand_Service(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {

	ROS_INFO_STREAM("Start landing on marker.");
	StartLanding_ = true;

	return true;
}

void velocityCtrl::mavposeCallback(const geometry_msgs::PoseStamped &msg) {

	if (!received_home_pose)
	{
		received_home_pose = true;
		home_pose_ = msg.pose;
		ROS_INFO_STREAM("Home pose initialized to: " << home_pose_);
	}

	mavPos_ = toEigen(msg.pose.position);

	mavAtt_(0) = msg.pose.orientation.w;
	mavAtt_(1) = msg.pose.orientation.x;
	mavAtt_(2) = msg.pose.orientation.y;
	mavAtt_(3) = msg.pose.orientation.z;

	mavCurrYaw_ = get_Euler_from_quat(msg.pose.orientation.x,
										msg.pose.orientation.y,
										msg.pose.orientation.z,
										msg.pose.orientation.w,
										Axis::YAW);

	/* making a eigen quaternion */
	Quaterniond quat;

	quat = Eigen::Quaterniond(msg.pose.orientation.w,
								msg.pose.orientation.x,
								msg.pose.orientation.y,
								msg.pose.orientation.z);
	/* making rotation matrix from quaternion */
	RotationBodyToNEU = quat.toRotationMatrix();


}

void velocityCtrl::mavtwistCallback(const geometry_msgs::TwistStamped &msg) {

	mavVel_ = toEigen(msg.twist.linear);
	mavRate_ = toEigen(msg.twist.angular);
}


void velocityCtrl::statusloopCallback(const ros::TimerEvent &event) {
	// if (sim_enable_) {

	// 	// Enable OFFBoard mode and arm automatically
	// 	// This is only run if the vehicle is simulated
	// 	arm_cmd_.request.value = true;
	// 	offb_set_mode_.request.custom_mode = "OFFBOARD";

	// 	if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(5.0))) {

	// 		if (set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent) {

	// 			ROS_INFO("Offboard enabled");
	// 		}

	// 		last_request_ = ros::Time::now();
	// 	}
	// 	else {
	// 		if (!current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0))) {

	// 			if (arming_client_.call(arm_cmd_) && arm_cmd_.response.success) {
	// 				ROS_INFO("Vehicle armed");
	// 			}

	// 			last_request_ = ros::Time::now();
	// 		}
	// 	}

	// }


	if(mavPos_(2) > 20.0){
		node_state = LANDING;
	}

}

bool velocityCtrl::landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {

	node_state = LANDING;
	ROS_WARN("Dinh Lam");
	return true;
}

bool velocityCtrl::check_position(float error, Eigen::Vector3d current, Eigen::Vector3d target){

	Eigen::Vector3d stop;
	stop << target - current;
	double a = stop.norm();

	if (a <= error){

		// cout << "Reach target!" << endl;
		// cout << a << endl;

		return true;
	}
	else
		return false;
}

double velocityCtrl::Query_DecreaseAltitude()
{
	if ((sTransitionPoint_1.range < range_err) && (mavPos_(2) >= sTransitionPoint_1.atitule)) {

		return ALLOW_DECREASE;
	}
	else if ((sTransitionPoint_2.range < range_err) && (mavPos_(2) >= sTransitionPoint_2.atitule)) {
		return ALLOW_DECREASE;
	}
	else if ((sTransitionPoint_3.range < range_err) && (mavPos_(2) >= sTransitionPoint_3.atitule)) {
		return ALLOW_DECREASE;
	}
	else {
		return NOT_ALLOW_DECREASE;
	}
}

void velocityCtrl::cmdloopCallback(const ros::TimerEvent &event)
{
	if (StartLanding_ && marker_pose_status == RECEIVED_POSE && calculate_range == NOT_CALCULATED) {

		sTransitionPoint_1.range = mavPos_(2) * tan(ANGLE_1 * PI/180);
		sTransitionPoint_1.atitule = mavPos_(2) * 2/3;

		sTransitionPoint_2.range = mavPos_(2) * tan(ANGLE_2 * PI/180) * 2/3;
		sTransitionPoint_2.atitule = mavPos_(2) * 1/3;

		sTransitionPoint_3.range = mavPos_(2) * tan(ANGLE_3 * PI/180) * 1/3;
		sTransitionPoint_3.atitule = 0.0;

		calculate_range = CALCULATED;

		ROS_INFO("Update MAX MIN PID");
		PID_x.setUMax(0.1);
		PID_x.setUMin(-0.1);

		PID_y.setUMax(0.1);
		PID_y.setUMin(-0.1);

		PID_z.setUMax(0.1);
		PID_z.setUMin(-0.1);
	}

	switch (node_state) {
		case WAITING_FOR_HOME_POSE:
		{
			// waitForPredicate(&received_home_pose, "Waiting for home pose...");
			ROS_INFO("Got pose! Drone Ready to be armed.");
			node_state = MISSION_EXECUTION;

			break;
		}

		case MISSION_EXECUTION:
		{
			switch (flight_mode)
			{
				case POSITION_MODE:
				{
					if(check_position(error, mavPos_, targetPos_))
					{
						flight_mode = VELOCITY_MODE;
					}

					pubPosition(targetPos_);

					break;
				}
				case VELOCITY_MODE:
				{
					// ROS_INFO("Got pose! Drone Velocity mode");
					Eigen::Vector4d velocity_vector;
					Eigen::Vector3d ErrorDistance;

					if(test_fly_waypoint_
						&& !StartLanding_
						&& check_position(error, mavPos_, targetPos_)
						&& index_setpoint < 4)
					{
						targetPos_(1) = setPoint_[index_setpoint].at(1);
						targetPos_(0) = setPoint_[index_setpoint].at(0);
						targetPos_(2) = setPoint_[index_setpoint].at(2);

						index_setpoint ++;
						cout << "index: " << index_setpoint << endl;
						// index_setpoint = (index_setpoint  % 4);
						ROS_INFO_STREAM(targetPos_);
					}

					// Service start landing is called
					if (StartLanding_) {

						targetPosPredict_(0) = targetPosPredict_(0) - (0.01 * mavVel_(0));
						targetPosPredict_(1) = targetPosPredict_(1) - (0.01 * mavVel_(1));

						if (Query_DecreaseAltitude() == ALLOW_DECREASE) {

							targetPos_(2) =  markerPosInBodyFrame_(2);

						} else {

							targetPos_(2) =  0.0;
						}

						targetPosPredict_(2) = targetPos_(2);

						if (mavPos_(2) < 1.0) {
							/* Change control mode */
							node_state = LANDING;
						}

						if (targetPos_(0) != markerPosInBodyFrame_(0)
							&& targetPos_(1) != markerPosInBodyFrame_(1)) {

							targetPos_(0) = markerPosInBodyFrame_(0);
							targetPos_(1) = markerPosInBodyFrame_(1);

							getErrorDistanceToTarget(targetPos_, Frame::UAV_BODY_OFFSET_FRAME, ErrorDistance);
						}
						else {
							// ROS_INFO("Error: [%f, %f]", targetPosPredict_(0), targetPosPredict_(1));
							getErrorDistanceToTarget(targetPosPredict_, Frame::UAV_BODY_OFFSET_FRAME, ErrorDistance);
						}
					}

					/*
					 * UAV_NEU_FRAME: If target is a point in the NEU frame
					 * UAV_BODY_FRAME: If target is a point offset in the BODY frame
					*/
					if (!StartLanding_) {
						getErrorDistanceToTarget(targetPos_, Frame::UAV_NEU_FRAME, ErrorDistance);
					}

					velocity_vector(0) = PID_x.compute(ErrorDistance(0), 0);
					velocity_vector(1) = PID_y.compute(ErrorDistance(1), 0);
					velocity_vector(2) = PID_z.compute(ErrorDistance(2), 0);


					publish_PIDterm(PID_z.getPTerm(),PID_z.getITerm(), PID_z.getDTerm());
					// ROS_INFO_STREAM("Got pose! Drone Velocity x " << velocity_vector(0) << " y " << velocity_vector(1) << " z " << velocity_vector(2));

					geometry_msgs::Vector3 msg;

					msg.x = targetPos_(0);
					msg.y = targetPos_(1);
					msg.z = targetPos_(2);
					debug_target.publish(msg);

					pubVelocity(velocity_vector);

					break;
				}

			}
			break;
		}

		case LANDING:
		{
			offb_set_mode_.request.custom_mode = "AUTO.LAND";

			if( set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent)
			{
				ROS_INFO("[ INFO] --------------- LAND ---------------\n");
			}
			node_state = LANDED;

			ros::spinOnce();

			break;
		}

		case LANDED:
		{
			ROS_INFO("Landed. Please set to position control and disarm.");
			cmdloop_timer_.stop();

			break;
		}
	}
}

void velocityCtrl::getErrorDistanceToTarget(const Eigen::Vector3d &target_position, Frame FrameType, Eigen::Vector3d &ErrorDistance) {

	switch (FrameType) {
		case Frame::UAV_BODY_OFFSET_FRAME: {

			ErrorDistance(0) = target_position(0);
			ErrorDistance(1) = target_position(1);
			ErrorDistance(2) = target_position(2);
		}
			break;
		case Frame::UAV_NEU_FRAME: {

			ErrorDistance(0) = target_position(0) - mavPos_(0);
			ErrorDistance(1) = target_position(1) - mavPos_(1);
			ErrorDistance(2) = target_position(2) - mavPos_(2);
		}
			break;
		default:
			ROS_WARN("Get error distance don't support this Frame");
			break;
	}

}

void velocityCtrl::convertPointFromOffsetBodyToNEU(const Eigen::Vector3d &PointBody, Eigen::Vector3d &PointNEU) {

	Eigen::Vector3d OffsetPointNEU;
	OffsetPointNEU = RotationBodyToNEU * PointBody;

	PointNEU(0) = OffsetPointNEU(0) + mavPos_(0);
	PointNEU(1) = OffsetPointNEU(1) + mavPos_(1);
	PointNEU(2) = OffsetPointNEU(2) + mavPos_(2);
}

void velocityCtrl::pubPosition(const Eigen::Vector3d &target_position){
	geometry_msgs::PoseStamped target_pose_;

	target_pose_.header.stamp = ros::Time::now();

	target_pose_.pose.position.x = target_position(0);
	target_pose_.pose.position.y = target_position(1);
	target_pose_.pose.position.z = target_position(2);

	target_pose_.pose.orientation.w = 1.0;
	target_pose_.pose.orientation.x = 0.0;
	target_pose_.pose.orientation.y = 0.0;
	target_pose_.pose.orientation.z = 0.0;

	setpoint_pose_pub_.publish(target_pose_);
}

void velocityCtrl::publish_PIDterm(double pTerm, double iTerm, double dTerm){
	geometry_msgs::Vector3 msg;
	msg.x = pTerm;
	msg.y = iTerm;
	msg.z = dTerm;
	term_PID_z.publish(msg);
}


void velocityCtrl::pubVelocity(const Eigen::Vector4d &desire_velicity_){

	mavros_msgs::PositionTarget setpoint_local;

	setpoint_local.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;

	setpoint_local.type_mask = 1987;

	// setpoint_local.position.z = mavPos_(2) - 0.3;

	setpoint_local.velocity.x = desire_velicity_(0);
	setpoint_local.velocity.y = desire_velicity_(1);
	setpoint_local.velocity.z = desire_velicity_(2);

	// setpoint_local.yaw = DEGREE_TO_RADIAN(90);
	// setpoint_local.yaw_rate = desire_velicity_(3);

	setRaw_pub.publish(setpoint_local);
}

void velocityCtrl::dynamicReconfigureCallback(velocity_controller::VelocityControllerConfig &config, uint32_t level) {

	if (PID_x.getKp() != config.kpx) {
		PID_x.setKp(config.kpx);
		ROS_INFO("Reconfigure request : kpx_ = %.2f ", config.kpx);
	}
	else if (PID_x.getKi() != config.kix) {
		PID_x.setKi(config.kix);
		ROS_INFO("Reconfigure request : kix_ = %.2f ", config.kix);
	}
	else if (PID_x.getKd() != config.kdx) {
		PID_x.setKd(config.kdx);
		ROS_INFO("Reconfigure request : kdx_ = %.2f ", config.kdx);
	}

	else if (PID_y.getKp() != config.kpy) {
		PID_y.setKp(config.kpy);
		ROS_INFO("Reconfigure request : kpy_ = %.2f ", config.kpy);
	}
	else if (PID_y.getKi() != config.kiy) {
		PID_y.setKi(config.kiy);
		ROS_INFO("Reconfigure request : kiy_ = %.2f ", config.kiy);
	}
	else if (PID_y.getKd() != config.kdy) {
		PID_y.setKd(config.kdy);
		ROS_INFO("Reconfigure request : kdy_ = %.2f ", config.kdy);
	}



	else if (PID_z.getKp() != config.kpz) {
		PID_z.setKp(config.kpz);
		ROS_INFO("Reconfigure request : kpz_ = %.2f ", config.kpz);
	}
	else if (PID_z.getKi() != config.kiz) {
		PID_z.setKi(config.kiz);
		ROS_INFO("Reconfigure request : kiz_ = %.2f ", config.kiz);
	}
	else if (PID_z.getKd() != config.kdz) {
		PID_z.setKd(config.kdz);
		ROS_INFO("Reconfigure request : kdz_ = %.2f ", config.kdz);
	}



	else if (PID_yaw.getKp() != config.kp_yaw) {
		PID_yaw.setKp(config.kp_yaw);
		ROS_INFO("Reconfigure request : kpyaw_ = %.2f ", config.kp_yaw);
	}
	else if (PID_yaw.getKi() != config.ki_yaw) {
		PID_yaw.setKi(config.ki_yaw);
		ROS_INFO("Reconfigure request : kiyaw_ = %.2f ", config.ki_yaw);
	}
	else if (PID_yaw.getKd() != config.kd_yaw) {
		PID_yaw.setKd(config.kd_yaw);
		ROS_INFO("Reconfigure request : kdyaw_ = %.2f ", config.kd_yaw);
	}

	else if (PID_yaw.getUMax() != config.UMax_yaw) {
		PID_yaw.setUMax(config.UMax_yaw);
		ROS_INFO("Reconfigure request : UMax_yaw = %.2f ", config.UMax_yaw);
	}
	else if (PID_yaw.getUMin() != config.UMin_yaw) {
		PID_yaw.setUMin(config.UMin_yaw);
		ROS_INFO("Reconfigure request : UMin = %.2f ", config.UMin_yaw);
	}

	else if (PID_x.getUMax() != config.UMax_x) {
		PID_x.setUMax(config.UMax_x);
		ROS_INFO("Reconfigure request : UMax_x = %.2f ", config.UMax_x);
	}
	else if (PID_x.getUMin() != config.UMin_x) {
		PID_x.setUMin(config.UMin_x);
		ROS_INFO("Reconfigure request : UMin_x = %.2f ", config.UMin_x);
	}

	else if (PID_y.getUMax() != config.UMax_y) {
		PID_y.setUMax(config.UMax_y);
		ROS_INFO("Reconfigure request : UMax_y = %.2f ", config.UMax_y);
	}
	else if (PID_y.getUMin() != config.UMin_y) {
		PID_y.setUMin(config.UMin_y);
		ROS_INFO("Reconfigure request : UMin_y = %.2f ", config.UMin_y);
	}

	else if (PID_z.getUMax() != config.UMax_z) {
		PID_yaw.setUMax(config.UMax_z);
		ROS_INFO("Reconfigure request : UMax_z = %.2f ", config.UMax_z);
	}

	else if (PID_z.getUMin() != config.UMin_z) {
		PID_z.setUMin(config.UMin_z);
		ROS_INFO("Reconfigure request : UMin_z = %.2f ", config.UMin_z);
	}
}


