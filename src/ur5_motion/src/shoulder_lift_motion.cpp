 #include "ros/ros.h"
 #include "std_msgs/Float64.h"
 #include <math.h>

 int main(int argc, char **argv)
 {

 ros::init(argc, argv, "shoulder_lift_motion");
 ros::NodeHandle n;


 ros::Publisher pubsl = n.advertise<std_msgs::Float64>("/shoulder_lift_joint_position_controller/command", 1000);

 ros::Rate loop_rate(1000);

 const double pi = 3.14159;
 
 while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
     
	std_msgs::Float64 msg;
    
	double secs =ros::Time::now().toSec(); 
	
	float ang; 
	
	for (int i = 0; i < 2; i++) {
	
	ang = (((pi/4)*sin(0.05*secs))-pi/2);
	msg.data = ang;
	pubsl.publish(msg);	
	//std::cout<<"Angle is: "<<ang<<"\n";
	}
	
	loop_rate.sleep();
    
	ros::spinOnce();
    
  }
  
}
