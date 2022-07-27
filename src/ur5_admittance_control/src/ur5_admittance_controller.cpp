#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include <math.h>
#include <cmath>

#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

// Get joint positions from gazebo
const int Joints = 6;
KDL::JntArray jnt_pos_start(Joints);

void get_shoulder_pan_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg){jnt_pos_start(0) = ctr_msg->process_value;}

void get_shoulder_lift_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg){jnt_pos_start(1) = ctr_msg->process_value;}

void get_elbow_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg){jnt_pos_start(2) = ctr_msg->process_value;}

void get_wrist_1_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg){jnt_pos_start(3) = ctr_msg->process_value;}

void get_wrist_2_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg){jnt_pos_start(4) = ctr_msg->process_value;}

void get_wrist_3_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg){jnt_pos_start(5) = ctr_msg->process_value;}

// Compute joint motion from current start to current goal
float compute_linear(double q_start, double q_goal, float t, float t_max) {return((q_goal - q_start) * (t/t_max) + q_start);}

const int loop_rate_val = 5;

int main(int argc, char **argv)
{
	std::string urdf_path = ros::package::getPath("ur5-joint-position-control");
	if(urdf_path.empty()) {
		ROS_ERROR("ur5-joint-position-control package path was not found");
	}
	urdf_path += "/urdf/ur5_jnt_pos_ctrl.urdf";
	
	ros::init(argc, argv, "tcp_control"); // Initialize the Node

	ros::NodeHandle n; // Create node handle

	ros::Rate loop_rate(loop_rate_val); // Define rate variable

	//Create subscriber for all joint states
		ros::Subscriber shoulder_pan_joint_sub = n.subscribe("/shoulder_pan_joint_position_controller/state", 1000, get_shoulder_pan_joint_position);
		ros::Subscriber shoulder_lift_joint_sub = n.subscribe("/shoulder_lift_joint_position_controller/state", 1000, get_shoulder_lift_joint_position);
		ros::Subscriber elbow_joint_sub = n.subscribe("/elbow_joint_position_controller/state", 1000, get_elbow_joint_position);
		ros::Subscriber wrist_1_joint_sub = n.subscribe("/wrist_1_joint_position_controller/state", 1000, get_wrist_1_joint_position);
		ros::Subscriber wrist_2_joint_sub = n.subscribe("/wrist_2_joint_position_controller/state", 1000, get_wrist_2_joint_position);
		ros::Subscriber wrist_3_joint_sub = n.subscribe("/wrist_3_joint_position_controller/state", 1000, get_wrist_3_joint_position);

	//Create publisher to send position commands to all joints
		ros::Publisher joint_com_pub[6]; 
		joint_com_pub[0] = n.advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command", 1000);
		joint_com_pub[1] = n.advertise<std_msgs::Float64>("/shoulder_lift_joint_position_controller/command", 1000);
		joint_com_pub[2] = n.advertise<std_msgs::Float64>("/elbow_joint_position_controller/command", 1000);
		joint_com_pub[3] = n.advertise<std_msgs::Float64>("/wrist_1_joint_position_controller/command", 1000);
		joint_com_pub[4] = n.advertise<std_msgs::Float64>("/wrist_2_joint_position_controller/command", 1000);
		joint_com_pub[5] = n.advertise<std_msgs::Float64>("/wrist_3_joint_position_controller/command", 1000);

	//Parse urdf model and generate KDL tree
		KDL::Tree ur5_tree;
		if (!kdl_parser::treeFromFile(urdf_path, ur5_tree)){
			ROS_ERROR("Failed to construct kdl tree");
	   		return false;
		}

	//Generate a kinematic chain from the robot base to its TCP (Tool Centre Point)
		KDL::Chain ur5_chain;
		ur5_tree.getChain("base_link", "wrist_3_link", ur5_chain);

	//Create forward and inverse solvers
		KDL::ChainFkSolverPos_recursive fk_solver(ur5_chain);
		KDL::ChainIkSolverVel_pinv vel_ik_solver(ur5_chain, 0.0001, 1000);
		KDL::ChainIkSolverPos_NR ik_solver(ur5_chain, fk_solver, vel_ik_solver, 1000);

	//Make sure we have received proper joint angles already
		for(int i=0; i< 2; i++) {
			ros::spinOnce();
		 	loop_rate.sleep(); 
		}
		
	// Compute initial TCP pose
		KDL::Frame tcp_pos_initial;
		fk_solver.JntToCart(jnt_pos_start, tcp_pos_initial);	
		
	// Compute distance of the tool frame from the origin 
		//float d;
		//d = sqrt(pow((tcp_pos_initial.p(0)-0),2)+pow((tcp_pos_initial.p(1)-0),2)+pow((tcp_pos_initial.p(2)-0),2));
	
	// Input Force Vector
		float Fx{0}, Fy{0}, Fz{0};
		std::cout <<"\n"<< "<------------------------------>"<<"\n";
		std::cout << "Enter non-zero Fx or Fy or Fz: ";
		std::cin>>Fx>>Fy>>Fz;
    		float F[3][1] = {{Fx}, {Fy}, {Fz}};
    		
    	// Input Admittance Gain
    		float Ka_x{0.005},Ka_y{0.01},Ka_z{0.01}; 
		//std::cout << "Enter desired admittance gain: ";
    		//std::cin>>Ka_x>>Ka_y>>Ka_z;
    		//float Ka_m[3][3]={{Ka_x,0,0},{0,Ka_y,0},{0,0,Ka_z}};
    		
    	// Compute target TCP velocities Xd
		float Xd[3][1] = {{Ka_x*Fx},{Ka_y*Fy},{Ka_z*Fz}};
		
	const float t_step = 1/((float)loop_rate_val);
	
	// #### Main loop 
	
	while (ros::ok()) {
	 				
	//Compute current TCP position
		KDL::Frame tcp_pos_start;
		fk_solver.JntToCart(jnt_pos_start, tcp_pos_start);

	//ROS_INFO("Current tcp Position/Twist KDL:");				
		
		std::cout<<"TCP: "<<"x="<<tcp_pos_start.p(0)<<"  y="<<tcp_pos_start.p(1)<<"  z="<<tcp_pos_start.p(2)<<",";
		
	// Compute reference position Xr
		float Xr_x,Xr_y,Xr_z,td{0.01};
		
		Xr_x = tcp_pos_initial.p(0);
		Xr_y = tcp_pos_initial.p(1);
		Xr_z = tcp_pos_initial.p(2);
	
		if (Fx != 0){Xr_x = (tcp_pos_start.p(0) + ((Xd[0][0])*td));} 	
		if (Fy != 0){Xr_y = (tcp_pos_start.p(1) + ((Xd[1][0])*td));}		
		if (Fz != 0){Xr_z = (tcp_pos_start.p(2) + ((Xd[2][0])*td));}
					
	// Compute theta for Xr
		KDL::Vector vec_tcp_pos_goal(Xr_x, Xr_y, Xr_z);
		KDL::Frame tcp_pos_goal(tcp_pos_initial.M, vec_tcp_pos_goal);		
		
	//Compute inverse kinematics
		KDL::JntArray jnt_pos_goal(Joints);
		ik_solver.CartToJnt(jnt_pos_start, tcp_pos_goal, jnt_pos_goal);
			 
	float d_sing{2.2};
		
		
	// (Pi-joint_elbow) to identify singular pose - Not an exhaustive indicator! Other singularities may occur e.g phi3 = 0
		float phi3;
		//= sqrt(pow((tcp_pos_goal.p(0)-0),2)+pow((tcp_pos_goal.p(1)-0),2)+pow((tcp_pos_goal.p(2)-0.1),2));
		phi3 = (3.142 - jnt_pos_goal(2));
		std::cout<<" phi3 ="<<(3.14-jnt_pos_goal(2))<<"\n";
	
	//Publishing joint positions
		std_msgs::Float64 position[6];
		
		float t = 0.0; 
		float t_max = loop_rate.sleep();
	
		while(t<t_max && phi3 < d_sing) {
			std_msgs::Float64 position[6];
			
			//Compute next position step for all joints
			for(int i=0; i<Joints; i++){
				position[i].data = compute_linear(jnt_pos_start(i), jnt_pos_goal(i), t, t_max);
				joint_com_pub[i].publish(position[i]);}

			ros::spinOnce();
			//loop_rate.sleep();	
			 
			t += t_step;}
		
			if(phi3 >= d_sing){std::cout<<"\n"<< "<-- Warning: Robot close to singularity! Relaunch the simulation to start from initial pose -->"<<"\n";}

	} // #### Closing main loop
	
	return 0;
}
