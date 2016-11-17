#include <ros/ros.h>
#include <range_msgs/P2PRange.h>
#include "swarm_bee.hpp"

int main(int argc, char** argv) 
{
	SwarmBeeDev node;
	SwarmBeeDev::RangeData data;
	ros::init(argc, argv, "nanotron_node"); 
	ros::NodeHandle nh; 
	ros::NodeHandle lnh("~");
	
	// Read node parameters
	int iniId, endId;
	std::string device, frame_id;
	if(!lnh.getParam("device", device))
	{
		std::cout << "[nanotron] No device specified! Assumming '/dev/ttyUSB0' as default" << std::endl;
		device = "/dev/ttyUSB0";	
	}
	if(!lnh.getParam("frame_id", frame_id))
		frame_id = "/nanotron";	
	if(!lnh.getParam("ini_id", iniId))
		iniId = 0;	
	if(!lnh.getParam("end_id", endId))
		endId = 255;	

	// Initialize serial device
	if(!node.init(device))
	{
		std::cout << "[nanotron] Error opening serial device " << device << std::endl;
		return -1;
	}	
	printf("Node Id: 0x%012x (%d)\n", node.readBaseId(), node.readBaseId()); 
	int selfId =  node.readBaseId();
	
	// Set node ranging list
	node.setRangeList(iniId, endId);
	
	// Advertise publishers
	ros::Publisher pubRange = nh.advertise<range_msgs::P2PRange>("/nanotron_node/range", 1);
	range_msgs::P2PRange rangeMsg;
	
	// Read range messages and publish
	int seq = 0;
	int id = 0;
	while(ros::ok())
	{
		// Wait until new range is received
		node.rangeToList(data);
		printf("Range %012x --> %012x: %f m, %f %%\n", data.emitterId, data.receiverId, data.range, data.rssi);
		
		// Fill-up msg and publish
		rangeMsg.header.seq = seq++;
		rangeMsg.header.frame_id = frame_id;
		rangeMsg.header.stamp = ros::Time::now();
		rangeMsg.radiation_type = range_msgs::P2PRange::RADIO;
		rangeMsg.source_id = data.emitterId;
		rangeMsg.source_type = range_msgs::P2PRange::ANCHOR;
		rangeMsg.destination_id = data.receiverId;
		rangeMsg.destination_type = range_msgs::P2PRange::BASE;
		rangeMsg.rssi = data.rssi;
		rangeMsg.range = data.range;
		rangeMsg.variance = 0.4*0.4;
		pubRange.publish(rangeMsg);
		
		// Spin
		ros::spinOnce();
	}
	
	// Close serial comms
	node.finish();

	return 0;
}

