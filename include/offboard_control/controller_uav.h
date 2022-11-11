#ifndef CONTROLLER_UAV_H
#define CONTROLLER_UAV_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <ros/subscribe_options.h>

#include <stdio.h>
#include <cstdlib>
#include <sstream>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
// #include <Eigen/Dense>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>


#include <dynamic_reconfigure/server.h>
#include <offboard_control/VelocityControllerConfig.h>

#include <std_srvs/SetBool.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

// #include "offboard_control/common.h"
#include "offboard_control/pid_controller_base.h"

using namespace std;
using namespace Eigen;

class velocityCtrl
{
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;

		ros::Subscriber multiDOFJointSub_;
		ros::Subscriber mavstateSub_;
		ros::Subscriber mavposeSub_;
		ros::Subscriber mavtwistSub_;
		ros::Subscriber yawreferenceSub_;
		ros::Subscriber target_pose;

		ros::ServiceClient arming_client_;
		ros::ServiceClient set_mode_client_;

		ros::Timer cmdloop_timer_, statusloop_timer_;
		ros::ServiceServer land_service_;

		Eigen::Vector3d mavPos_, mavVel_, mavRate_;
		Eigen::Vector4d mavAtt_;

		geometry_msgs::Pose home_pose_;

		mavros_msgs::State current_state_;
		mavros_msgs::SetMode offb_set_mode_;
		mavros_msgs::CommandBool arm_cmd_;

		ros::Publisher term_PID_z;
		ros::Publisher debug_target;

		ros::Publisher setRaw_pub;
		ros::Publisher setpoint_pose_pub_; // publish target pose to drone
		ros::Publisher velocity_pub_;
		ros::Time last_request_;

		bool sim_enable_;
		bool test_fly_waypoint_;
		bool received_home_pose;

		double error;
		template <class T>
		void waitForPredicate(const T *pred, const std::string &msg, double hz = 2.0) {

			ros::Rate pause(hz);
			ROS_INFO_STREAM(msg);

			while (ros::ok() && !(*pred)) {
				ros::spinOnce();
				pause.sleep();
			}
		};

		enum FlightState { WAITING_FOR_HOME_POSE, MISSION_EXECUTION, LANDING, LANDED } node_state;
		enum FlightMode {POSITION_MODE, VELOCITY_MODE} flight_mode;

		double max_out_, min_out_;
		double kpx_,kdx_, kix_;
		double kpy_,kdy_, kiy_;
		double kpz_,kdz_, kiz_;
		double kpyaw_,kdyaw_, kiyaw_;

		double initTargetPos_x_, initTargetPos_y_, initTargetPos_z_, initTarget_yaw_;

		double targetYaw_;
		double mavCurrYaw_;
		Eigen::Vector3d targetPos_;

		PidControllerBase PID_x, PID_y, PID_z, PID_yaw;

		std::vector<double> setPoint_[4];

	public:
		velocityCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
		void dynamicReconfigureCallback(velocity_controller::VelocityControllerConfig &config, uint32_t level);

		void mavposeCallback(const geometry_msgs::PoseStamped &msg);
		void statusloopCallback(const ros::TimerEvent &event);
		void mavtwistCallback(const geometry_msgs::TwistStamped &msg);
		void cmdloopCallback(const ros::TimerEvent &event);
		void targetPositioncallback(const geometry_msgs::PoseStamped &msg);
		void pubPosition(const Vector3d &target_position);
		void pubVelocity(const Eigen::Vector4d &desire_velicity_);
		bool landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);
		bool check_position(float error, Eigen::Vector3d current, Eigen::Vector3d target);
		void publish_PIDterm( double pTerm, double iTerm, double dTerm);

};

#endif