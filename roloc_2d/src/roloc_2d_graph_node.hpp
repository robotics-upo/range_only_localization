#ifndef __ROLOC_2D_GRAPH_NODE_HPP__
#define __ROLOC_2D_GRAPH_NODE_HPP__

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <range_msgs/P2PRangeWithPose.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <vector>
#include "roloc_2d_graph.hpp"

class RoLoc2DGraphNode
{
public:

	struct rangeData
	{
		uint64_t id;
		float x;
		float y;
		float z;
		std::vector<float> range;
		float variance;
		float getMeanRange(void)
		{
			if(range.size() == 0)
				return -1.0;
			float mean = 0;
			for(size_t i=0; i<range.size(); i++)
				mean += range[i];
			mean = mean/((float)range.size());
			return mean;
		}
	};
	
	// Class constructor
    RoLoc2DGraphNode(std::string nodeName) : graph(nodeName)
    {
		// Load parameters
		ros::NodeHandle lnh("~");
		if(!lnh.getParam("update_rate", m_updateRate))
			m_updateRate = 5.0;
		if(!lnh.getParam("update_min_d", m_dTh))
			m_dTh = 0.3;
		if(!lnh.getParam("update_min_a", m_aTh))
			m_aTh = 0.1;
		if(!lnh.getParam("init_x", m_initX))
			m_initX = 10000000.0;	
		if(!lnh.getParam("init_y", m_initY))
			m_initY = 10000000.0;	
		if(!lnh.getParam("init_a", m_initA))
			m_initA = 10000000.0;	
		if(!lnh.getParam("height_correction", m_heightCorrection))
			m_heightCorrection = 0.0;
		if(!lnh.getParam("base_frame_id", m_baseFrameId))
			m_baseFrameId = "base_link";	
		if(!lnh.getParam("odom_frame_id", m_odomFrameId))
			m_odomFrameId = "odom";	
		if(!lnh.getParam("global_frame_id", m_globalFrameId))
			m_globalFrameId = "map";
		if(!lnh.getParam("transform_tolerance", m_transform_tolerance))
			m_transform_tolerance = 0.1;	

		// Initialization flag to false
		m_isInit = false;
		m_tryInit = false;
		m_lastGlobalTf.setIdentity();
		m_lastOdomTf.setIdentity();
		if(m_initX < 1000000.0 || m_initY < 1000000.0 || m_initA < 1000000.0)
			m_tryInit = true;
					
		// Subscribe to range data topic and initial pose
        m_rangeSub = m_nh.subscribe("/range", 10, &RoLoc2DGraphNode::rangeDataCallback, this);
        m_scanSub = m_nh.subscribe("/scan", 10, &RoLoc2DGraphNode::scanDataCallback, this);
        m_initialPoseSub = lnh.subscribe("/roloc_graph_node/initial_pose", 2, &RoLoc2DGraphNode::initialPose, this);
        m_visPub = lnh.advertise<visualization_msgs::MarkerArray>("/roloc_graph_node/robot_position", 0);
        m_visOdomPub = lnh.advertise<visualization_msgs::MarkerArray>("/roloc_graph_node/graph_odom", 0);
        
        // Launch updater timer
		m_updateTimer = m_nh.createTimer(ros::Duration(1.0/m_updateRate), &RoLoc2DGraphNode::checkUpdateThresholdsTimer, this);
		
		// Temp
		m_t1 = m_t2 = m_t3 = m_t4 = ros::Time::now();
    }

    // Class destructor
    ~RoLoc2DGraphNode(void)
    {
	}
	
	// Callback for initial pose
	void initialPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
	{
		// We only accept initial pose estimates in the global frame
		if(msg->header.frame_id != m_globalFrameId)
		{
			ROS_WARN("Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
			msg->header.frame_id.c_str(),
			m_globalFrameId.c_str());
			return;	
		}
		
		// Transform into the global frame
		tf::Pose pose;
		tf::poseMsgToTF(msg->pose.pose, pose);
		ROS_INFO("Setting pose (%.6f) => x: %.3f y: %.3f theta: %.3f", ros::Time::now().toSec(), pose.getOrigin().x(), pose.getOrigin().y(), getYawFromTf(pose));
		
		// Initialize the filter
		initFilter(pose.getOrigin().x(), pose.getOrigin().y(), getYawFromTf(pose));
	}

	// Check motion thresholds for filter update
	bool checkUpdateThresholds(void)
	{
		bool m_doUpdate = false;
		
		// If the filter is not initialized then exit
		if(!m_isInit && m_tryInit)
		{
			m_tryInit = false;
			if(!initFilter(m_initX, m_initY, m_initA))
			{
				m_tryInit = true;
				return false;
			}
		}
		
		// Publish current TF from odom to global
		m_tfBr.sendTransform(tf::StampedTransform(m_lastGlobalTf, ros::Time::now() + ros::Duration(0.2), m_globalFrameId, m_odomFrameId));

		// Compute odometric translation and rotation since last update 
		tf::StampedTransform odomTf;
		try
		{
			m_tfListener.waitForTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), ros::Duration(0.5));
			m_tfListener.lookupTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), odomTf);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("RoLoc2DGraph error: %s",ex.what());
			return false;
		}
		tf::Transform T = m_lastOdomTf.inverse()*odomTf;
			
		// Check translation threshold
		if(T.getOrigin().length() > m_dTh)
		{
            ROS_INFO("Translation update");
            m_doUpdate = true;
		}
		
		// Check yaw threshold
		if(fabs(getYawFromTf(T)) > m_aTh)
		{
            ROS_INFO("Rotation update");
			m_doUpdate = true;
		}
		
		// Perform graph update if required
		if(m_doUpdate)
		{
			// Estimate dt and da from odometry
			float dt, da;
			da = atan2(T.getOrigin().y(), fabs(T.getOrigin().x()));
			dt = T.getOrigin().x()/cos(da);
			da = getYawFromTf(T);
			
			// Store last odom
			m_lastOdomTf = odomTf;
			
			// Add current odom constraint
			if(m_data.size() > 0)
				graph.addOdomContrains(dt, da, m_scanX, m_scanY);
			else
				graph.addOdomContrains(dt, da);
			
			// If we have range measurements, add them and optimize
			if(m_data.size() > 0)
			{   
				// Debug
				std::cout << "Ranging to: " << std::endl;
				for(int i=0; i< m_data.size(); i++)
				{
					std::cout << "\tNode: " <<  m_data[i].id << ", Range: " << m_data[i].getMeanRange() << std::endl;
				}

				// Add range constrains
				for(int i=0; i<m_data.size(); i++)
				{
					// Get node position
					float px, py, pz;
					px = m_data[i].x;
					py = m_data[i].y;
					pz = m_data[i].z;
			
					// Get 2D projection of the 3D meassurement considering the sensor height
					float r = m_data[i].getMeanRange();
					r = sqrt(r*r - (pz-m_heightCorrection)*(pz-m_heightCorrection)); 
					
					// Add constraints
					graph.addRangeConstraints(px, py, r);
				}
				
				// Clean range measurement vector
				m_data.clear();

				// Optimize graph
				if(graph.optimize())
				{
					int i = graph.m_vertexes.size()-1;
					double x, y, a;
					x = graph.m_vertexes[i].x;
					y = graph.m_vertexes[i].y;
					a = graph.m_vertexes[i].a;
					
					// Update global transform
					m_lastGlobalTf = tf::Transform(tf::Quaternion(0.0, 0.0, sin(a*0.5), cos(a*0.5)), tf::Vector3(x, y, 0))*m_lastOdomTf.inverse();
				}
			}
		}
		
		
		return false;
	}

private:

	// Periodically check motion thresholds
	void checkUpdateThresholdsTimer(const ros::TimerEvent& event)
	{
		checkUpdateThresholds();
	}
	
	// Range data callback
	void rangeDataCallback(const range_msgs::P2PRangeWithPose::ConstPtr& msg)
	{
		switch(msg->source_id)
		{
			case 0xdeca010010000a5f:
			
				if((msg->header.stamp-m_t1).toSec() < 10)
					return;
				else
					m_t1 = msg->header.stamp;
				break;
				
			case 0xdeca0100100007a0:
			
				if((msg->header.stamp-m_t2).toSec() < 10)
					return;
				else
					m_t2 = msg->header.stamp;
				break;
				
			case 0xdeca010010000a60:
			
				if((msg->header.stamp-m_t3).toSec() < 10)
					return;
				else
					m_t3 = msg->header.stamp;
				break;
			
			case 0xdeca010010000a5e:
			
				if((msg->header.stamp-m_t4).toSec() < 10)
					return;
				else
					m_t4 = msg->header.stamp;
				break;
				
			default:
					return;
		}
		
		// Do we have already measurements from this anchor?
		int index = -1;
		for(size_t i = 0; i<m_data.size(); i++)
			if(msg->source_id == m_data[i].id)
			{
				index = i;
				break;
			}
	
		// Save range meaurement
		if(index < 0)
		{
			rangeData anchor;
			anchor.id = msg->source_id;
			anchor.x =  msg->position.point.x;
			anchor.y =  msg->position.point.y;
			anchor.z =  msg->position.point.z;
			anchor.variance = msg->variance;
			anchor.range.push_back(msg->range);
			m_data.push_back(anchor);
		}
		else
			m_data[index].range.push_back(msg->range);	
	}
	
	double getYawFromTf(tf::Pose &pose)
	{
		double r, p, y;
		pose.getBasis().getRPY(r, p, y);
		return y;
	}
	
	bool initFilter(double x, double y, double a)
	{
		// Init graph
		graph.clearGraph();
		graph.setInitPos(x, y, a);
		
		// Init odom TF
		try
		{
			m_tfListener.waitForTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), ros::Duration(m_transform_tolerance));
			m_tfListener.lookupTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), m_lastOdomTf);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("RoLoc2DGraph error: %s",ex.what());
			return false;
		}
		
		// Init global TF
		m_lastGlobalTf = tf::Transform(tf::Quaternion(0.0, 0.0, sin(a*0.5), cos(a*0.5)), tf::Vector3(x, y, 0))*m_lastOdomTf.inverse();
		m_auxTf = m_lastGlobalTf;

		// Set init flag
		m_isInit = true;
		std::cout  << "[roloc_2d_graph] Initialized at (" << x << ", " << y << ") with yaw = " << a << "rad" << std::endl;
		
		return true;
	}
	
	void scanDataCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
	{
		sensor_msgs::PointCloud cloud;
		laser_geometry::LaserProjection scanProjector;
		
		// Get the point cloud of every scan in base frame
		try
		{
			scanProjector.transformLaserScanToPointCloud(m_baseFrameId, *scan, cloud, m_tfListener);
		}
		catch (tf::TransformException& e)
		{
			std::cout << e.what();
			return;
		}
		
		// Store the point-cloud
		m_scanX.resize(cloud.points.size());
		m_scanY.resize(cloud.points.size());
		for(size_t i=0; i < cloud.points.size(); i++)
		{
			m_scanX[i] = cloud.points[i].x;
			m_scanY[i] = cloud.points[i].y;
		}
	}
   
	// Parameters
	std::string m_baseFrameId, m_odomFrameId, m_globalFrameId;
	double m_rangeDev;
	double m_odomDDev, m_odomADev;
	double m_initX, m_initY, m_initA;
	double m_initXDev, m_initYDev, m_initADev;
	double m_heightCorrection;
	double m_dTh, m_aTh;
	double m_updateRate;
	double m_transform_tolerance;
	bool m_isInit, m_tryInit;
	
	// Last odom transform
	tf::StampedTransform m_lastOdomTf;
	
	// Last global TF estimated
	tf::Transform m_lastGlobalTf, m_auxTf;
	
	// Range measurements
	std::vector<rangeData> m_data;
	
	// Ro-localization optimization graph
	RoLoc2DGraph graph;
	
	// Subscriber data and timer
	ros::NodeHandle m_nh;
	ros::Subscriber m_rangeSub, m_initialPoseSub, m_scanSub;
	ros::Timer m_updateTimer;
	tf::TransformBroadcaster m_tfBr;
	tf::TransformListener m_tfListener;
	ros::Publisher m_visPub, m_visOdomPub;
	
	// Laser scan stuff
	std::vector<float> m_scanX, m_scanY;
	
	// Temporal
	ros::Time m_t1, m_t2, m_t3, m_t4;
};

#endif




