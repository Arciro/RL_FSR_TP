#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include "tf/tf.h"

using namespace std;


void odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
	double x = odom->pose.pose.orientation.x;
	double y = odom->pose.pose.orientation.y;
	double z = odom->pose.pose.orientation.z;
	double w = odom->pose.pose.orientation.w;
	tf::Quaternion q(x, y, z, w);

	double roll, pitch, theta;
	tf::Matrix3x3(q).getRPY(roll, pitch, theta);
	
	cout<<"Theta: "<<theta<<endl;
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "nodo_prova");
	ros::NodeHandle nh;
	ros::Subscriber sub;
	sub = nh.subscribe("/ddr/odom", 10, odom_callback);

	ros::spin();
	
	return 0;
}
