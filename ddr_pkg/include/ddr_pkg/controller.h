#ifndef TRACKING_REGULATION_CONTROLLER_H
#define TRACKING_REGULATION_CONTROLLER_H

#include "ros/ros.h"
#include "boost/thread.hpp"

#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "tf/tf.h"

#include "ddr_pkg/ctrl_to_plan.h"
#include "ddr_pkg/plan_to_ctrl.h"

#include <iostream>
#include <cmath>
#include <vector>


using namespace std;


class TrackReg
{
	private:
		ros::NodeHandle nh;
		ros::Subscriber odom_sub;
		ros::Subscriber path_sub;
		ros::Publisher vel_pub;
		ros::Publisher wR_pub;
		ros::Publisher wL_pub;
		ros::ServiceClient client;
		ros::ServiceServer server;
		
		double theta;
		
		bool first_odom;
		bool path_received;
		
		double t; //variabile temporale
		double time_traj;
		
		//parameters of I/O feedback linearization
		double b;
		double track_k1;
		double track_k2;
		double y1;
		double y2;
		
		//parameters of posture regulation
		double reg_k1;
		double reg_k2;
		
		double pW; //wheel radidus
		double d; //separation distance between wheels
		
		vector<geometry_msgs::Point> wp_list;
		int wp_index;
	
	public:
		TrackReg();
		void odometry_callback(nav_msgs::Odometry);
		//void path_callback(nav_msgs::Path);
		bool path_callback(ddr_pkg::plan_to_ctrl::Request&, ddr_pkg::plan_to_ctrl::Response&);
	
		void tvp(double, double, double, double&, double&); //trapezoidal velocity profile
		
		void ctrl_loop();
		void run();
	
};

#endif
