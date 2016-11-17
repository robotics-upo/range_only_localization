#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <range_msgs/P2PRange.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include "roloc_2d_graph_node.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "roloc_graph_node");  
  
	// Ranhe-only localization instance
	RoLoc2DGraphNode loc("roloc_graph_node");
	
	// Spin for ever 
	ros::spin(); 
}




