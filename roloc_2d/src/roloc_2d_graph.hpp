#ifndef __ROLOC_2D_GRAPH_HPP__
#define __ROLOC_2D_GRAPH_HPP__

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <splm/splm.h>
#include <unistd.h>

class RoLoc2DGraph
{
public:

	// Graph vertexes are 2D poses
	struct Vertex
	{
		double x;	// Robot X position in meters
		double y;	// Robot Y position in meters
		double a;	// Robot orientation in rad
		
		// Vestex laser-scan
	    std::vector<float> scanX;
	    std::vector<float> scanY;
		
		Vertex(void)
		{
			x = 0;
			y = 0;
			a = 0;
		}
		
		Vertex(float _x, float _y, float _a)
		{
			x = _x;
			y = _y;
			a = _a;
		}

		Vertex(float _x, float _y, float _a, std::vector<float> &_scanX, std::vector<float> &_scanY)
		{
			x = _x;
			y = _y;
			a = _a;
			scanX = _scanX;
			scanY = _scanY;
		}
		
		Vertex(const Vertex &d)
		{
			x = d.x;
			y = d.y;
			a = d.a;
			scanX = d.scanX;
			scanY = d.scanY;
		}
		
		Vertex& operator=(const Vertex& d)
		{
			if(this != &d)
			{
				x = d.x;
				y = d.y;
				a = d.a;
				scanX = d.scanX;
				scanY = d.scanY;
			}
			return *this;
		}
	};
	
	// Graph edges are 2D robot odometry or ranging to anchors
	struct Edge
	{
		int type;	// -1: unknown, 0: robot 2d odom, 1: range to anchor
		
		// Base vertex
		int fromVertex;
		
		// 2D odometry transform from fromVertex to toVertex
		int toVertex;	// Destinatio vertex 
		double t;		// Meassured translation in meters
		double a; 		// Meassured rotation in rad
		
		// Range distance from fromVertex to the anchor position
		double r;	// Meassured range in meters
		double ax;	// Anchor X position in meters
		double ay;	// Anchor Y position in meters

		Edge(void)
		{
			type = -1;
			fromVertex = -1;
			toVertex = -1;
			t = 0;
			a = 0; 	
			r = -1;
			ax = 0;	
			ay = 0;	
		}
		
		Edge(int _fromVertex, int _toVertex, double _t, double _a)
		{
			type = 0;
			fromVertex = _fromVertex;
			toVertex = _toVertex;
			t = _t;
			a = _a; 
			
			r = -1;
			ax = 0;	
			ay = 0;	
		}
		
		Edge(int _fromVertex, double _anchorX, double _anchorY, double _range)
		{
			type = 1;
			fromVertex = _fromVertex;
			r = _range;
			ax = _anchorX;
			ay = _anchorY;
			
			toVertex = -1;
			t = 0;
			a = 0; 
		}
		
		Edge(const Edge &d)
		{
			type = d.type;
			fromVertex = d.fromVertex;
			toVertex = d.toVertex;
			t = d.t;
			a = d.a; 	
			r = d.r;
			ax = d.ax;	
			ay = d.ay;	
		}
		
		Edge& operator=(const Edge& d)
		{
			if(this != &d)
			{
				type = d.type;
				fromVertex = d.fromVertex;
				toVertex = d.toVertex;
				t = d.t;
				a = d.a; 	
				r = d.r;
				ax = d.ax;	
				ay = d.ay;
			}
			return *this;
		}
	};
	
	
	// Class constructor
    RoLoc2DGraph(std::string nodeName)
    {
		// Load parameters
		ros::NodeHandle lnh("~");
        if(!lnh.getParam("range_weight", m_rangeW))
            m_rangeW = 0.8;
		if(!lnh.getParam("odom_weight", m_odomW))
			m_odomW = 1.0;
		if(!lnh.getParam("max_poses", m_maxPoses))
			m_maxPoses = -1;
		if(!lnh.getParam("publish_graph", m_publishGraph))
			m_publishGraph = 0;
		if(!lnh.getParam("global_frame_id", m_publishGraphFrameId))
			m_publishGraphFrameId = "map";
		
		// Visualization of graph	
		if(m_publishGraph > 0)
			m_visPub = lnh.advertise<visualization_msgs::MarkerArray>("/roloc_graph_node/graph", 0);
    }

    // Class destructor
    ~RoLoc2DGraph(void)
    {
	}
	
	// Set inial 2D position
	void setInitPos(double x, double y, double a)
	{
		m_vertexes.push_back(Vertex(x, y, a));
	}
	
	// Add odometry constraint over the latest vertex
	void addOdomContrains(double dt, double da)
	{
		float x, y, a;
		
		// Last vertex index
		int i = m_vertexes.size()-1;
		
		// Compute odometry based on lastest extimation
		x = m_vertexes[i].x + dt*cos(m_vertexes[i].a);
		y = m_vertexes[i].y + dt*sin(m_vertexes[i].a);
		a = m_vertexes[i].a + da;
		
		// Add the new vertex
		m_vertexes.push_back(Vertex(x, y, a));
		
		// Add the associated edge
		m_edges.push_back(Edge(i, i+1, dt, da));		
	}
	
	// Add odometry constraint with laser-scan over the latest vertex
	void addOdomContrains(double dt, double da, std::vector<float> X, std::vector<float> Y)
	{
		float x, y, a;
		
		// Last vertex index
		int i = m_vertexes.size()-1;
		
		// Compute odometry based on lastest extimation
		x = m_vertexes[i].x + dt*cos(m_vertexes[i].a);
		y = m_vertexes[i].y + dt*sin(m_vertexes[i].a);
		a = m_vertexes[i].a + da;
		
		// Add the new vertex
		m_vertexes.push_back(Vertex(x, y, a, X, Y));
		
		// Add the associated edge
		m_edges.push_back(Edge(i, i+1, dt, da));		
	}
	
	// Add range constraints to the latest vertex
	void addRangeConstraints(double anchorX, double anchorY, double range)
	{
		// Last estimation index
		int i = m_vertexes.size()-1;
		
		// Add the associated edge
		m_edges.push_back(Edge(i, anchorX, anchorY, range));
	}
	
	// Optimize graph
	bool optimize(void)
	{
		// Check graph consistency
		//if(m_vertexes.size() <= 1 || m_edges.size() < m_vertexes.size()-1)
		//	return false;
		
		// Compute number of parameters to estimate
		int m;
		if(m_vertexes.size() > m_maxPoses && m_maxPoses > 0)
		{
			m = 3*m_maxPoses;
			m_initVertex = m_vertexes.size() - m_maxPoses;	
		}
		else
		{
			m = 3*m_vertexes.size();
			m_initVertex = 0;
		}
		//std::cout << "M: " << m << std::endl;
		
		// Fill up initial parameters vector
		int j = 0;
		double *p = new double[m];
		for(unsigned int i=m_initVertex; i<m_vertexes.size(); i++)
		{
			p[j++] = m_vertexes[i].x;
			p[j++] = m_vertexes[i].y;
			p[j++] = m_vertexes[i].a;
		}
		//std::cout << "J: " << j << std::endl;
		
		// Compute number of meassurement equations:
		// - 3x for every odom constraint
		// - 1x for every anchor ranging
		// Also compute the total number of non-ceros of the jacobian:
		// - 8x for every odom constraint
		// - 2x for every anchor ranging
		int n = 0, nnz = 0;
		for(unsigned int i=0; i<m_edges.size(); i++)
		{
			if(m_edges[i].type == 0)
			{
				if(m_edges[i].fromVertex >= m_initVertex && m_edges[i].toVertex >= m_initVertex)
				{
					n += 3;
					nnz += 8;
				}
			}
			else if(m_edges[i].type == 1)
			{
				if(m_edges[i].fromVertex > m_initVertex)
				{
					n += 1;
					nnz += 2;
				}
			}
		}
		//std::cout << "N: " << n << std::endl;
		//std::cout << "NNZ: " << nnz << std::endl;
		
		// Setup optimizer
		double opts[SPLM_OPTS_SZ], info[SPLM_INFO_SZ];
		opts[0]=SPLM_INIT_MU; opts[1]=SPLM_STOP_THRESH; opts[2]=SPLM_STOP_THRESH;
		opts[3]=SPLM_STOP_THRESH;
		opts[4]=SPLM_DIFF_DELTA; // relevant only if finite difference approximation to Jacobian is used
		opts[5]=SPLM_CHOLMOD; // use CHOLMOD
		
		// Launch optimization
		int res;
		res = sparselm_dercrs(RoLoc2DGraph::evalGraph, RoLoc2DGraph::evalGraphJacob, p, NULL, m, 3, n, nnz, -1, 10000, opts, info, this); 
		if(res < 0)
			return false;
		std::cout << "SPLM Solved: " << res << " iterations" << std::endl;
			
		// Store solution into vertexes
		j = 0;
		for(unsigned int i=m_initVertex; i<m_vertexes.size(); i++)
		{
			m_vertexes[i].x = p[j++];
			m_vertexes[i].y = p[j++];
			m_vertexes[i].a = p[j++];
		}
		
		// Free memory
		delete []p;
		
		return true;
	}
	
	void publishGraph(std::string frame_id, int startVertex = 0)
	{
		// The teresa robot is compossed by 4 markers
		visualization_msgs::MarkerArray graph;
		visualization_msgs::Marker marker;
		
		// Robot path
		marker.header.frame_id = frame_id;
		marker.header.stamp = ros::Time::now();
		marker.ns = "roloc_2d_graph";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::LINE_STRIP;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = 0.0;
		marker.pose.position.y = 0.0;
		marker.pose.position.z = 0.0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0;
		marker.pose.orientation.w = 1;
		marker.scale.x = 0.05;
		marker.scale.y = 0.0;
		marker.scale.z = 0.0;
		marker.color.a = 0.5; 
		marker.color.r = 1;
		marker.color.g = 1;
		marker.color.b = 1;
		marker.points.clear();
		for(int i=startVertex; i<m_vertexes.size(); i++)
		{
			geometry_msgs::Point p;
			p.x = m_vertexes[i].x;
			p.y = m_vertexes[i].y;
			p.z = 0;
			marker.points.push_back(p);
		}
		graph.markers.push_back(marker);
		
		// Robot poses with orietation
		marker.id = 1;
		marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = 0.0;
		marker.pose.position.y = 0.0;
		marker.pose.position.z = 0.0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 1.0;
		marker.scale.y = 1.0;
		marker.scale.z = 1.0;
		marker.color.a = 1.0; 
		marker.color.r = 0;
		marker.color.g = 1;
		marker.color.b = 0;
		marker.points.clear();
		geometry_msgs::Point p1, p2, p3;
		p1.x = 0.0, p1.y = 0.25, p1.z = 0.0;
		p2.x = 0.5, p2.y = 0.0,  p2.z = 0.0;
		p3.x = 0.0, p3.y =-0.25, p3.z = 0.0;
		for(int i=startVertex; i<m_vertexes.size(); i++)
		{
			double tx, ty, a;
			geometry_msgs::Point p;
			tx = m_vertexes[i].x;
			ty = m_vertexes[i].y;
			a =  m_vertexes[i].a;
			
			p.x = p1.x*cos(a) - p1.y*sin(a) + tx;
			p.y = p1.x*sin(a) + p1.y*cos(a) + ty;
			p.z = p1.z;
			marker.points.push_back(p);
			p.x = p2.x*cos(a) - p2.y*sin(a) + tx;
			p.y = p2.x*sin(a) + p2.y*cos(a) + ty;
			p.z = p2.z;
			marker.points.push_back(p);
			p.x = p3.x*cos(a) - p3.y*sin(a) + tx;
			p.y = p3.x*sin(a) + p3.y*cos(a) + ty;
			p.z = p3.z;
			marker.points.push_back(p);
		}
		graph.markers.push_back(marker);
		
		// Robot scans
		marker.id = 2;
		marker.type = visualization_msgs::Marker::POINTS;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = 0.0;
		marker.pose.position.y = 0.0;
		marker.pose.position.z = 0.0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.05;
		marker.scale.y = 0.05;
		marker.scale.z = 0.0;
		marker.color.a = 1.0; 
		marker.color.r = 1;
		marker.color.g = 1;
		marker.color.b = 1;
		marker.points.clear();
		for(int i=startVertex; i<m_vertexes.size(); i++)
		{
			std::vector<float> &scanX = m_vertexes[i].scanX;
			std::vector<float> &scanY = m_vertexes[i].scanY;
			if(scanX.size() == 0)
				continue;
			
			// Catch robot position and orientation
			double tx, ty, a;
			tx = m_vertexes[i].x;
			ty = m_vertexes[i].y;
			a =  m_vertexes[i].a;
			
			// Transform every scan point into the robot system reference
			for(int j=0; j<scanX.size(); j++)
			{
				geometry_msgs::Point p;
				p.x = scanX[j]*cos(a) - scanY[j]*sin(a) + tx;
				p.y = scanX[j]*sin(a) + scanY[j]*cos(a) + ty;
				p.z = 0;
				marker.points.push_back(p);
			}
		}
		graph.markers.push_back(marker);
		
		// Publish markers
		m_visPub.publish(graph);
	}
	
	void clearGraph(void)
	{
		m_vertexes.clear();
	}

	// Graph vertexes
	std::vector<Vertex> m_vertexes;
	int m_initVertex;
	
	// Graph edges
	std::vector<Edge> m_edges;
	
private:

	static void evalGraph(double *p, double *hx, int m, int n, void *data)
	{
		RoLoc2DGraph *pClass = (RoLoc2DGraph *)data; 
		std::vector<Edge> &m_edges = pClass->m_edges;
		std::vector<Vertex> &m_vertexes = pClass->m_vertexes;
		double rW = pClass->m_rangeW, oW = pClass->m_odomW; 
		int m_initVertex = pClass->m_initVertex;
		int m_publishGraph = pClass->m_publishGraph;
		std::string m_publishGraphFrameId = pClass->m_publishGraphFrameId;
		int j;
		
		// Show current solution
		if(m_publishGraph > 0)
		{
			j = 0;
			for(unsigned int i=m_initVertex; i<m_vertexes.size(); i++)
			{
				m_vertexes[i].x = p[j++];
				m_vertexes[i].y = p[j++];
				m_vertexes[i].a = p[j++];
			}
			if(m_publishGraph == 1)
				pClass->publishGraph(m_publishGraphFrameId, m_initVertex);
			else
				pClass->publishGraph(m_publishGraphFrameId);
			ros::spinOnce();
		}
		
		// Evaluate edges
		j = 0;
		for(unsigned int i=0; i<m_edges.size(); i++)
		{	
			double x1, y1, a1, x2, y2, a2, ax, ay, t, a, r;		
			int v;
			
			// Check the edge affects vertexes in use in this optimization
			if(m_edges[i].type == 0 && (m_edges[i].fromVertex < m_initVertex || m_edges[i].toVertex < m_initVertex))
				continue;
			if(m_edges[i].type == 1 && m_edges[i].fromVertex <= m_initVertex)
				continue;
			
			// Get vertex position and values into parameters vector
			v = 3*(m_edges[i].fromVertex-m_initVertex);
			x1 = p[v++];
			y1 = p[v++];
			a1 = p[v];	
				
			if(m_edges[i].type == 0)
			{
				// Get vertex position into parameters vector and constraints 
				v = 3*(m_edges[i].toVertex-m_initVertex);
				x2 = p[v++];
				y2 = p[v++];
				a2 = p[v];
				t = m_edges[i].t;
				a = m_edges[i].a;
				
				hx[j++] = (x2 - x1 - t*cos(a1)) * oW;
				hx[j++] = (y2 - y1 - t*sin(a1)) * oW;
				hx[j++] = (a2 - a1 - a) * oW;
			}
			else if(m_edges[i].type == 1)
			{
				// Get constraints from edge
				ax = m_edges[i].ax;
				ay = m_edges[i].ay;
				r = m_edges[i].r;
				
				hx[j++] = (r - sqrt((x1-ax)*(x1-ax) + (y1-ay)*(y1-ay))) * rW;
			}
		}
	}
	
	static void evalGraphJacob(double *p, struct splm_crsm *jac, int m, int n, void *data)
	{
		RoLoc2DGraph *pClass = (RoLoc2DGraph *)data;
		std::vector<Edge> &m_edges = pClass->m_edges;
		std::vector<Vertex> &m_vertexes = pClass->m_vertexes;
		int m_initVertex = pClass->m_initVertex;
		
		// Evaluate edge jacobians
		int k = 0, l = 0;
		for(unsigned int i=0; i<m_edges.size(); i++)
		{	
			double x1, y1, a1, x2, y2, a2, ax, ay, t, a, r;		
			int v1, v2;
			
			// Check the edge affects vertexes in use in this optimization
			if(m_edges[i].type == 0 && (m_edges[i].fromVertex < m_initVertex || m_edges[i].toVertex < m_initVertex))
				continue;
			if(m_edges[i].type == 1 && m_edges[i].fromVertex <= m_initVertex)
				continue;

			// Get vertex position and values into parameters vector
			v1 = 3*(m_edges[i].fromVertex-m_initVertex);
			x1 = p[v1];
			y1 = p[v1+1];
			a1 = p[v1+2];
			
			if(m_edges[i].type == 0)
			{
				// Get vertex position into parameters vector and constraints 
				v2 = 3*(m_edges[i].toVertex-m_initVertex);
				x2 = p[v2];
				y2 = p[v2+1];
				a2 = p[v2+2];
				t = m_edges[i].t;
				a = m_edges[i].a;
				
				jac->rowptr[k++] = l;
				jac->val[l] = -1.0;
				jac->colidx[l++] = v1;
				jac->val[l] =  t*sin(a1);
				jac->colidx[l++] = v1+2;
				jac->val[l] =  1.0;
				jac->colidx[l++] = v2; 
				
				jac->rowptr[k++] = l;
				jac->val[l] = -1.0;
				jac->colidx[l++] = v1+1;
				jac->val[l] = -t*cos(a1);
				jac->colidx[l++] = v1+2;
				jac->val[l] =  1.0;
				jac->colidx[l++] = v2+1;
				
				jac->rowptr[k++] = l;
				jac->val[l] = -1.0;
				jac->colidx[l++] = v1+2;
				jac->val[l] =  1.0;
				jac->colidx[l++] = v2+2; 
				
				//hx[j++] = x2 - x1 - t*cos(a1);     -1  0 +t*sin(a1) 1 0 0 
				//hx[j++] = y2 - y1 - t*sin(a1);      0 -1 -t*cos(a1) 0 1 0
				//hx[j++] = a2 - a1 - a; 			  0  0 -1         0 0 1
			}
			else if(m_edges[i].type == 1)
			{
				// Get constraints from edge
				ax = m_edges[i].ax;
				ay = m_edges[i].ay;
				r = 2*sqrt((x1-ax)*(x1-ax) + (y1-ay)*(y1-ay));
				
				jac->rowptr[k++] = l;
				jac->val[l] = (2*ax-2*x1)/r;
				jac->colidx[l++] = v1;
				jac->val[l] = (2*ay-2*y1)/r;
				jac->colidx[l++] = v1+1;
				
				//hx[j++] = r - sqrt((x1-ax)*(x1-ax) + (y1-ay)*(y1-ay));       
			}
		}
		
		// By convention
		jac->rowptr[k]=l;
	}
   
	// Parameters
	double m_rangeW, m_odomW;
	int m_maxPoses, m_publishGraph; 
	std::string m_publishGraphFrameId;
	
	// Node handler
	ros::NodeHandle m_nh;	
	ros::Publisher m_visPub;
};

#endif


