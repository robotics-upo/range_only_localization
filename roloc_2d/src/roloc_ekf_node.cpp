#include <ros/ros.h>
#include "roloc_2d_ekf.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "roloc_ekf_node");  
  
	// Ranhe-only localization instance
	RoLoc2DEKF loc;
	
	// Spin for ever 
	ros::spin(); 
}




