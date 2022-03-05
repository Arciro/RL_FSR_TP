#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "ros/ros.h"
#include "boost/thread.hpp"

#include "tf/tf.h"
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/JointState.h" 
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"

#include <iostream>
#include <cmath> 

using namespace std;

class Odom{

	private:
		ros::NodeHandle nh;
		ros::Publisher odom_pub;
		ros::Subscriber wheels_sub;
		ros::Subscriber imu_sub;
		
		double pW, d;
		double q0L, q0R;
		double qL, qR;
		double wL, wR;
		double thetak;
		
		bool first_wheel;

	public:
		Odom();
		void run();
		void range_kutta();
		void wheels_callback(const sensor_msgs::JointState::ConstPtr&);
		void imu_callback(const sensor_msgs::Imu::ConstPtr&);
		
};

#endif
