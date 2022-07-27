#include <std_srvs/Empty.h>
#include "ros/ros.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "unpause_gazebo");
	ros::NodeHandle nh;
	
	ros::ServiceClient pauseGazebo = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
	
	std_srvs::Empty emptySrv;
	
	pauseGazebo.call(emptySrv);
	
	return 0;
}


