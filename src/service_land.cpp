#include "ros/ros.h"
#include <cstdlib>
#include <std_srvs/SetBool.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "land_services");
	ros::NodeHandle n;

	// ros::ServiceClient client1 = n.serviceClient<std_srvs::SetBool>("start_land");
	ros::ServiceClient client2 = n.serviceClient<std_srvs::SetBool>("start_landing");

	std_srvs::SetBool land_cmd;
	land_cmd.request.data = true;

	// client1.call(land_cmd);
	client2.call(land_cmd);

	ROS_WARN_STREAM("Done");

	return 0;
}
