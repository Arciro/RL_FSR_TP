#ifndef RRT_PLANNER_ASTAR_H
#define RRT_PLANNER_ASTAR_H

#include "ros/ros.h"
#include <ros/package.h>
#include "boost/thread.hpp"

#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h" //to command a velocity to the robot
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h" //to receive the odometry for current position
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "tf/tf.h"
/*
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"*/

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <string>
#include <vector>
#include <random> 


#define MAX_NUM_ITERATIONS 10


using namespace std;
//using namespace cv;

typedef struct{
	double x;
	double y;
	
}Point2D;


struct Nodo{
	int id;
	Point2D position;
	vector<int> adj_list;
	
	//fields necessary for A*
	bool visited;
	Nodo* parent;
	
	float f_Ni;
	float g_Ni;
	float h_Ni;
};


typedef vector<Nodo> Tree;


class RRT_Astar
{
	private:
		ros::NodeHandle nh;
		
		ros::Subscriber odom_sub;
		ros::Subscriber map_sub;
		ros::Subscriber barcode_sub;

		ros::Publisher array_marker_pub;
		ros::Publisher marker_pub;
		ros::Publisher path_pub;
		
		string room;
		
		nav_msgs::OccupancyGrid::Ptr map;
		Point2D map_origin;
		double map_resolution;
		int map_height;
		int map_width;
		Point2D current_position; //current position of the robot to give us the starting node of the rrt
		double current_theta;
		Point2D destination;
		Point2D final_position;
		
		int counter; //to count iterations of RRT
		bool first_odom; //flag to advertise us that odom is been received
		bool map_received;
		bool qrcode_read;
		bool trees_generated;
		bool RRTend;
		float delta;

		Tree trajectory;
		//Mat img;
	
	public:
		RRT_Astar(); //constructor
		void barcode_callback(const std_msgs::String::Ptr&);
		void odometry_callback(nav_msgs::Odometry);
		void map_callback(const nav_msgs::OccupancyGrid::Ptr&);
		void planner();
		void run();
		
		//RRT methods
		void rrt(Tree&, Tree&);
		void generate_tree(Tree&);
		void find_qnear(Point2D, Nodo&, Tree);
		void calculate_qnew(Nodo&, Nodo, Point2D, float);
		void visualization_arch_node(Nodo, Nodo, float, float, float);
		void joining_bridge(Tree&, Tree&);
		double get_distance(Point2D, Point2D);
		bool collision_check(Nodo, Nodo, double);
		void display_node(Nodo);
		int toIndex(Point2D);
		
		//A* methods
		void Astar_search(Tree&);
		void find_extract_Nbest(Nodo&, Tree&);
		float fCost(Nodo);
		float gCost(const Nodo&);
		float hCost(Nodo);
		bool in(Nodo, Tree);
		void visualization_minimum_path(Nodo*, float, float, float);
};

#endif
