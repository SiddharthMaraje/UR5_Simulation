#include "ros/ros.h"
#include <std_srvs/Empty.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "unpause_gazebo");
  ros::NodeHandle n;

  ros::ServiceClient pauseGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
  	
  //ros::service::waitForService("/gazebo/unpause_physics", -1);
  
  	//ros::Duration(4).sleep();
  	
	std_srvs::Empty emptySrv;
	pauseGazebo.call(emptySrv);  
	
	//ros::spin();

  return 0;
}



