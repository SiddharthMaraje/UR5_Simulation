#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <math.h>

int main(int argc, char **argv)
{

ros::init(argc, argv, "elbow_motion");
ros::NodeHandle n;

ros::Publisher pube = n.advertise<std_msgs::Float64>("/elbow_joint_position_controller/command", 1000);


ros::Rate loop_rate(20);


while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::Float64 msg;
    
     
    for (int i = 0; i < 2; i++) {
  	double secs =ros::Time::now().toSec(); 
  	msg.data = sin(0.05*secs);
	 
	 pube.publish(msg);
	 
	 
	}
        	
    loop_rate.sleep();
    
    ros::spinOnce();
    
  }
  
}
