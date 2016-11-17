#ifndef __ROLOC2DEKF_HPP__
#define __ROLOC2DEKF_HPP__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <range_msgs/P2PRange.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>

class RoLoc2DEKF
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
    RoLoc2DEKF(void)
    {
		// Load parameters
		ros::NodeHandle lnh("~");
        if(!lnh.getParam("range_dev", m_rangeDev))
            m_rangeDev = -1.0;
		if(!lnh.getParam("update_rate", m_updateRate))
			m_updateRate = 5.0;
		if(!lnh.getParam("odom_d_dev", m_odomDDev))
			m_odomDDev = 0.1;
		if(!lnh.getParam("odom_a_dev", m_odomADev))
			m_odomADev = 0.1;
		if(!lnh.getParam("update_min_d", m_dTh))
			m_dTh = 0.2;
		if(!lnh.getParam("update_min_a", m_aTh))
			m_aTh = 0.1;
		if(!lnh.getParam("init_x_dev", m_initXDev))
			m_initXDev = 0.1;	
		if(!lnh.getParam("init_y_dev", m_initYDev))
			m_initYDev = 0.1;	
		if(!lnh.getParam("init_a_dev", m_initADev))
			m_initADev = 0.1;
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

		// Initialization flag to false
		m_isInit = false;
		m_tryInit = false;
		m_lastGlobalTf.setIdentity();
		m_lastOdomTf.setIdentity();
		if(m_initX < 1000000.0 || m_initY < 1000000.0 || m_initA < 1000000.0)
			m_tryInit = true;
					
		// Subscribe to range data topic and initial pose
        m_rangeSub = m_nh.subscribe("/range", 10, &RoLoc2DEKF::rangeDataCallback, this);
        m_initialPoseSub = lnh.subscribe("/initial_pose", 2, &RoLoc2DEKF::initialPose, this);
        m_visPub = lnh.advertise<visualization_msgs::MarkerArray>("/robot_position", 0);
        
        // Launch updater timer
		m_updateTimer = m_nh.createTimer(ros::Duration(1.0/m_updateRate), &RoLoc2DEKF::checkUpdateThresholdsTimer, this);
		
		// Temp
		m_t1 = m_t2 = m_t3 = m_t4 = ros::Time::now();
    }

    // Class destructor
    ~RoLoc2DEKF(void)
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
		m_tfBr.sendTransform(tf::StampedTransform(m_lastGlobalTf, ros::Time::now(), m_globalFrameId, m_odomFrameId));

		// Compute odometric trasnlation and rotation since last update 
		tf::StampedTransform odomTf;
		try
		{
			m_tfListener.waitForTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), ros::Duration(0.5));
			m_tfListener.lookupTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), odomTf);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("RoLoc2D error: %s",ex.what());
			return false;
		}
		tf::Transform T = m_lastOdomTf.inverse()*odomTf;
		
		m_tfBr.sendTransform(tf::StampedTransform(m_auxTf*odomTf, ros::Time::now(), m_globalFrameId, "base_link2"));
		
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
		
		// Perform filter update if required
		if(m_doUpdate)
		{
			// Estimate dt and da from odometry
			float dt, da;
			da = atan2(T.getOrigin().y(), T.getOrigin().x());
			dt = T.getOrigin().x()/cos(da);
			da = getYawFromTf(T);
			
			// Perform state prediction
			predict(dt, da);
			m_lastOdomTf = odomTf;
						
			// Try filter update 
			if(update())
			{				
				// Update global transform
				m_lastGlobalTf = tf::Transform(tf::Quaternion(0.0, 0.0, sin(m_x(2)*0.5), cos(m_x(2)*0.5)), tf::Vector3(m_x(0), m_x(1), 0))*m_lastOdomTf.inverse();
			}
		}
		
		publishRobotMarker(0, 0, 0, "TeresaBody", 0);
		publishRobotMarker(0, 0, 0, "base_link", 1);
		publishRobotMarker(0, 0, 0, "base_link2", 2);
		publishNodes("optitrack");
		
		return false;
	}
	
	// Filter predition based on odometry 
    bool predict(float dt, float da)
    {		
		m_x(0) += dt*cos(m_x(2));
		m_x(1) += dt*sin(m_x(2));
		m_x(2) += da;
		Eigen::Matrix<float, 2, 2> Q;
		Q.setIdentity();
		Q(0,0) = m_odomDDev*m_odomDDev; Q(1,1) = m_odomADev*m_odomADev;
		Eigen::Matrix<float, 3, 3> J1;
		J1.setIdentity();
		J1(0,2) = -dt*sin(m_x(2)); J1(1,2) = dt*cos(m_x(2)); 
		Eigen::Matrix<float, 3, 2> J2;
		J2(0,0) = cos(m_x(2)); J2(0,1) = 0.0;
		J2(1,0) = sin(m_x(2)); J2(1,1) = 0.0;
		J2(2,0) = 0.0;         J2(2,1) = 1.0;
		m_P = J1*m_P*J1.transpose() + J2*Q*J2.transpose();
		
		return false;
    }
    
    // Filter update based on odometry predicion and range updates
    bool update(void)
    {
		// Check that we have enought updates
		std::cout << "N measurements: " << m_data.size() << std::endl;
		if(m_data.size() == 0)
		{
			return false;
		}	
		
		// Detect outliers
		int nUpdates = 0;
		std::vector<bool> inliers(m_data.size(), false);
		float x = m_x(0), y = m_x(1), a = m_x(2);
		for(size_t i = 0; i<m_data.size(); i++)
		{
			// Get node position
			float px, py, pz;
			px = m_data[i].x;
			py = m_data[i].y;
			pz = m_data[i].z;
				
			// Get 2D projection of the 3D meassurement cosidering the sensor height
			float r = m_data[i].getMeanRange();
			r = sqrt(r*r + (pz-m_heightCorrection)*(pz-m_heightCorrection)); 
			
			// Get update equation
			float r_ = sqrt((x-px)*(x-px) + (y-py)*(y-py));
					
			// If this an inlier?
			if(fabs(r-r_) < 1.0)
			{
				nUpdates++;
				inliers[i] = true;
			}
			else
			{
				std::cout << "Outlier!" << std::endl;
			}
		}
		if(nUpdates <= 0)
			return false;
		
		// Get update equations
		int k = 0;
		Eigen::MatrixXf H, R;
		Eigen::VectorXf z, hx;
		H.setZero(nUpdates,3);
		z.setZero(nUpdates);
		hx.setZero(nUpdates);
		R.setIdentity(nUpdates, nUpdates);
		for(size_t i = 0; i<m_data.size(); i++)
		{
			if(!inliers[i])
				continue;
				
			// Get node position
			float px, py, pz;
			px = m_data[i].x;
			py = m_data[i].y;
			pz = m_data[i].z;
				
			// Get 2D projection of the 3D meassurement cosidering the sensor height
			float range = m_data[i].getMeanRange();
			z(k) = sqrt(range*range + (pz-m_heightCorrection)*(pz-m_heightCorrection)); 
			
			// Get update equation
			hx(k) = sqrt((x-px)*(x-px) + (y-py)*(y-py));
			
			// Compute measurement jacobian
			H(k,0) = -(2*px - 2*x)/(2*hx(k)); H(k,1) = -(2*py - 2*y)/(2*hx(k)); H(k,2) = 0.0;
			
			// Get the update noise matrix
			if(m_rangeDev > 0.0)
				R(k,k) = m_rangeDev*m_rangeDev;
			else
				R(k,k) = m_data[i].variance;
			k++;
		}
		std::cout << "z:" << z << std::endl;
		std::cout << "hx:" << hx << std::endl;
				
		// Compute innovation matrix
		Eigen::MatrixXf S = H*m_P*H.transpose()+R;
		
		// Compute kalman gain
		Eigen::MatrixXf K = m_P*H.transpose()*S.inverse();
		
		// Compute innovation vector
		Eigen::VectorXf in = z - hx;
		
		// Compute new state vector
		m_x = m_x + K*in;
		
		// Compute new covariance matrix
		Eigen::Matrix<float, 3, 3> I, aux;
		I.setIdentity();
		aux = (I-K*H)*m_P;
		m_P = aux;
		
		// Clean range measurement vector
		m_data.clear();
		
		std::cout << "x:\n" << m_x.transpose() << std::endl;
		std::cout << "P:\n" << m_P << std::endl;
		
		return true;
	}
	
	void publishRobotMarker(float x, float y, float yaw, std::string frame_id, int robot_id)
	{
		// The teresa robot is compossed by 4 markers
		visualization_msgs::MarkerArray robot;
		visualization_msgs::Marker marker;
		
		// Get robot color
		float r, g, b;
		switch(robot_id)
		{
			case 0:
				r = 1.0;
				g = 0.0;
				b = 0.0;
				break;
			case 1:
				r = 0.0;
				g = 1.0;
				b = 0.0;
				break;
			case 2:
				r = 0.0;
				g = 0.0;
				b = 1.0;
				break;
			default:
				r = 1.0;
				g = 1.0;
				b = 0.0;
		}
		
		// Robot direction
		marker.header.frame_id = frame_id;
		marker.header.stamp = ros::Time::now();
		marker.ns = "roloc_2d";
		marker.id = 10*robot_id;
		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = x;
		marker.pose.position.y = y;
		marker.pose.position.z = 0.15;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = sin(yaw*0.5);
		marker.pose.orientation.w = cos(yaw*0.5);
		marker.scale.x = 0.65;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.color.a = 1.0; 
		marker.color.r = r;
		marker.color.g = g;
		marker.color.b = b;
		robot.markers.push_back(marker);
		
		// Wheel frame
		marker.header.frame_id = frame_id;
		marker.header.stamp = ros::Time::now();
		marker.ns = "roloc_2d";
		marker.id = 10*robot_id+1;
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = x;
		marker.pose.position.y = y;
		marker.pose.position.z = 0.15;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = sin(yaw*0.5);
		marker.pose.orientation.w = cos(yaw*0.5);
		marker.scale.x = 0.6;
		marker.scale.y = 0.5;
		marker.scale.z = 0.3;
		marker.color.a = 1.0; 
		marker.color.r = r;
		marker.color.g = g;
		marker.color.b = b;
		robot.markers.push_back(marker);
		
		// Computer frame
		marker.header.frame_id = frame_id;
		marker.header.stamp = ros::Time::now();
		marker.ns = "roloc_2d";
		marker.id = 10*robot_id+2;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = x;
		marker.pose.position.y = y;
		marker.pose.position.z = 0.3+0.20;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = sin(yaw*0.5);
		marker.pose.orientation.w = cos(yaw*0.5);
		marker.scale.x = 0.25;
		marker.scale.y = 0.15;
		marker.scale.z = 0.40;
		marker.color.a = 1.0; 
		marker.color.r = r;
		marker.color.g = g;
		marker.color.b = b;
		robot.markers.push_back(marker);
		
		// Stalk
		marker.header.frame_id = frame_id;
		marker.header.stamp = ros::Time::now();
		marker.ns = "roloc_2d";
		marker.id = 10*robot_id+3;
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = x;
		marker.pose.position.y = y;
		marker.pose.position.z = 0.3+0.40+0.35;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = sin(yaw*0.5);
		marker.pose.orientation.w = cos(yaw*0.5);
		marker.scale.x = 0.05;
		marker.scale.y = 0.05;
		marker.scale.z = 0.70;
		marker.color.a = 1.0; 
		marker.color.r = r;
		marker.color.g = g;
		marker.color.b = b;
		robot.markers.push_back(marker);
		
		// Screen
		marker.header.frame_id = frame_id;
		marker.header.stamp = ros::Time::now();
		marker.ns = "roloc_2d";
		marker.id = 10*robot_id+4;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = x;
		marker.pose.position.y = y;
		marker.pose.position.z =  0.3+0.40+0.35+0.2;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = sin(yaw*0.5);
		marker.pose.orientation.w = cos(yaw*0.5);
		marker.scale.x = 0.05;
		marker.scale.y = 0.25;
		marker.scale.z = 0.4;
		marker.color.a = 1.0; 
		marker.color.r = r;
		marker.color.g = g;
		marker.color.b = b;
		robot.markers.push_back(marker);
		
		//only if using a MESH_RESOURCE marker type:
		m_visPub.publish(robot);
	}
	
	void publishNodes(std::string frame_id)
	{
		// The teresa robot is compossed by 4 markers
		visualization_msgs::MarkerArray robot;
		visualization_msgs::Marker marker;
		
		// Robot direction
		marker.header.frame_id = frame_id;
		marker.header.stamp = ros::Time::now();
		marker.ns = "roloc_2d";
		marker.id = 100;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = 0.9363;
		marker.pose.position.y = -1.843324;
		marker.pose.position.z = 1.5625;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0;
		marker.pose.orientation.w = 0;
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.color.a = 1.0; 
		marker.color.r = 0;
		marker.color.g = 1;
		marker.color.b = 0;
		robot.markers.push_back(marker);
		
		marker.header.frame_id = frame_id;
		marker.header.stamp = ros::Time::now();
		marker.ns = "roloc_2d";
		marker.id = 101;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x =  -1.5429;
		marker.pose.position.y = -1.8475;
		marker.pose.position.z =  1.5559;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0;
		marker.pose.orientation.w = 0;
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.color.a = 1.0; 
		marker.color.r = 0;
		marker.color.g = 1;
		marker.color.b = 0;
		robot.markers.push_back(marker);
		
		marker.header.frame_id = frame_id;
		marker.header.stamp = ros::Time::now();
		marker.ns = "roloc_2d";
		marker.id = 102;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = 1.1792;
		marker.pose.position.y = 2.9165;
		marker.pose.position.z = 1.6565;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0;
		marker.pose.orientation.w = 0;
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.color.a = 1.0; 
		marker.color.r = 0;
		marker.color.g = 1;
		marker.color.b = 0;
		robot.markers.push_back(marker);
		
		marker.header.frame_id = frame_id;
		marker.header.stamp = ros::Time::now();
		marker.ns = "roloc_2d";
		marker.id = 103;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = -2.2047;
		marker.pose.position.y = 2.9120;
		marker.pose.position.z = 1.46947;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0;
		marker.pose.orientation.w = 0;
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.color.a = 1.0; 
		marker.color.r = 0;
		marker.color.g = 1;
		marker.color.b = 0;
		robot.markers.push_back(marker);
		
		m_visPub.publish(robot);
	}

private:

	// Periodically check motion thresholds
	void checkUpdateThresholdsTimer(const ros::TimerEvent& event)
	{
		checkUpdateThresholds();
	}
	
	// Range data callback
	void rangeDataCallback(const range_msgs::P2PRange::ConstPtr& msg)
	{	
		if(msg->source_id == 0xdeca010010000a5f && (msg->header.stamp-m_t1).toSec() < 3)
		{
			return;
		}
		else
			m_t1 = msg->header.stamp;
		if(msg->source_id == 0xdeca0100100007a0 && (msg->header.stamp-m_t2).toSec() < 3)
		{
			return;
		}
		else
			m_t2 = msg->header.stamp;
		if(msg->source_id == 0xdeca010010000a60 && (msg->header.stamp-m_t3).toSec() < 3)
		{
			return;
		}
		else
			m_t3 = msg->header.stamp;
		if(msg->source_id == 0xdeca010010000a5e && (msg->header.stamp-m_t4).toSec() < 3)
		{
			return;
		}
		else
			m_t4 = msg->header.stamp;
		
		/* Check frame of the anchor positio 
		if(msg->position.header.frame_id != globalFrameId)
		{
			ROS_WARN("Ignoring range measurement from anchor %lx with position in frame \"%s\"; anchor pose must be specified in global frame \"%s\"",
			msg->source_id,
			msg->position.header.frame_id.c_str(),
			m_globalFrameId.c_str());
			return;	
		}*/
		
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
			if(msg->source_id == 0xdeca010010000a5f)
			{
				anchor.x =  0.9363;
				anchor.y = -1.843324;
				anchor.z =  1.5625;
			}
			if(msg->source_id == 0xdeca0100100007a0)
			{
				anchor.x = -1.5429;
				anchor.y = -1.8475;
				anchor.z =  1.5559;
			}
			if(msg->source_id == 0xdeca010010000a60)
			{
				anchor.x = 1.1792;
				anchor.y = 2.9165;
				anchor.z = 1.6565;
			}
			if(msg->source_id == 0xdeca010010000a5e)
			{
				anchor.x = -2.2047;
				anchor.y =  2.9120;
				anchor.z =  1.46947;
			}
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
		// Init covariance matrix
		m_P.setIdentity();
		m_P(0,0) = m_initXDev*m_initXDev;
		m_P(1,1) = m_initYDev*m_initYDev;
		m_P(2,2) = m_initADev*m_initADev;
		
		// Init state vector
		m_x(0) = x;
		m_x(1) = y; 
		m_x(2) = a;
		
		// Init odom TF
		try
		{
			m_tfListener.waitForTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), ros::Duration(2.0));
			m_tfListener.lookupTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), m_lastOdomTf);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("RoLoc2D error: %s",ex.what());
			return false;
		}
		
		// Init global TF
		m_lastGlobalTf = tf::Transform(tf::Quaternion(0.0, 0.0, sin(a*0.5), cos(a*0.5)), tf::Vector3(x, y, 0))*m_lastOdomTf.inverse();
		m_auxTf = m_lastGlobalTf;

		// Set init flag
		m_isInit = true;
		std::cout  << "[roloc_2d] Initialized at (" << x << ", " << y << ") with yaw = " << a << "rad" << std::endl;
		
		return true;
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
	bool m_isInit, m_tryInit;
	
	// Last odom transform
	tf::StampedTransform m_lastOdomTf;
	
	// Last global TF estimated
	tf::Transform m_lastGlobalTf, m_auxTf;
	
	// Range measurements
	std::vector<rangeData> m_data;
	
	// SLAM Covariance matrix
    Eigen::Matrix<float, 3, 3> m_P;	
    
    // SLAM state vector x = [x,y,angle]'
	Eigen::Matrix<float, 3, 1> m_x;
	
	// Subscriber data and timer
	ros::NodeHandle m_nh;
	ros::Subscriber m_rangeSub, m_initialPoseSub;
	ros::Timer m_updateTimer;
	tf::TransformBroadcaster m_tfBr;
	tf::TransformListener m_tfListener;
	ros::Publisher m_visPub;
	
	// Temporal
	ros::Time m_t1, m_t2, m_t3, m_t4;
};

#endif


