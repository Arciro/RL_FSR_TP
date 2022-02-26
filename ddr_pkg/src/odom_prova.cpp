#include "ros/ros.h"

#include "tf/tf.h"
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/JointState.h" 
#include "nav_msgs/Odometry.h"

#include <iostream>
#include <cmath> 

using namespace std;

double q[2];

void wheels_callback(const sensor_msgs::JointState::ConstPtr& wheels_msg)
{
	for (int i=0; i<2; i++)
		q[i] = wheels_msg->position[i];
	
/*	wL = wheels_msg->velocity[0];
	//cout<<"\n wL: "<<wL;
	wR = wheels_msg->velocity[1];
	//cout<<"\n wR: "<<wR;*/
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odom_node");
	ros::NodeHandle nh;
	ros::Publisher odom_pub;
	ros::Subscriber wheels_sub;
		
	double pW;
	double d;
	double wR;
	double wL;
	
	if (!nh.getParam("wheel_radius", pW))
		pW = 0.032; 
	
	if (!nh.getParam("wheel_separation", d))
		d = 0.045;
	
	odom_pub = nh.advertise<nav_msgs::Odometry>("/ddr/odom", 0);
	wheels_sub = nh.subscribe("/ddr/joint_states", 0, wheels_callback);
	
	double freq = 1000;
	double x0 = 0;
	double y0 = 0;
	double theta0 = 0;
	
	double xk = x0;
	double yk = y0;
	double thetak = theta0;
	
	double q_prec[2];
	for(int i=0; i<2; i++)
		q_prec[i] = 0;
	
	ros::Time current_time, last_time;
	
	ros::Rate r(freq);
	while(ros::ok())
	{
		tf::TransformBroadcaster odom_broadcaster;
		current_time = ros::Time::now();
		double Ts = (current_time - last_time).toSec();
		if(Ts == 0)
			Ts = 1/freq;
			
		cout<<" Ts: "<<Ts<<endl;
		//calculate linear and angular velocity from right and left wheels' velocities
		wL = (q[0] - q_prec[0])/Ts;
		wR = (q[1] - q_prec[1])/Ts;
		double vk = (pW/2.0)*(wR+wL);
		double wk = (pW/d)*(wR-wL);
		for(int j=0; j<2; j++)
			q_prec[j] = q[j];
		
		//range-kutta
		xk = xk + vk*Ts*cos(thetak + (wk*Ts)/2.0);
		yk = yk + vk*Ts*sin(thetak + (wk*Ts)/2.0);
		thetak = thetak + wk*Ts; //diviso 6?
		
		if(thetak >= M_PI)
			thetak = thetak - 2*M_PI;
			
		else if(thetak <= -M_PI)
			thetak = thetak + 2*M_PI;
		
		cout<<"\n Theta: "<<thetak<<endl;
		
		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion quat_msg;
		quat_msg = tf::createQuaternionMsgFromYaw(thetak);
		
		//first, we'll publish the transform over tf
    	geometry_msgs::TransformStamped odom_trans;
    	odom_trans.header.stamp = current_time;
    	odom_trans.header.frame_id = "odom";
    	odom_trans.child_frame_id = "base_footprint";

    	odom_trans.transform.translation.x = xk;
    	odom_trans.transform.translation.y = yk;
    	odom_trans.transform.translation.z = 0.0;
    	odom_trans.transform.rotation = quat_msg;

    	//send the transform
    	odom_broadcaster.sendTransform(odom_trans);
		
		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom_msg;
		odom_msg.header.stamp = current_time;
		odom_msg.header.frame_id = "odom";
		odom_msg.child_frame_id = "base_footprint";
		
		//set position and orientation
		odom_msg.pose.pose.position.x = xk;
		odom_msg.pose.pose.position.y = yk;
		odom_msg.pose.pose.position.z = 0;
		odom_msg.pose.pose.orientation = quat_msg; 
		
		//set the velocity
		odom_msg.twist.twist.linear.x = vk;
		odom_msg.twist.twist.linear.y = 0;
		odom_msg.twist.twist.linear.z = 0;
		odom_msg.twist.twist.angular.x = 0;
		odom_msg.twist.twist.angular.y = 0;
		odom_msg.twist.twist.angular.z = wk;
		
		//publish the message
		odom_pub.publish(odom_msg);
		last_time = current_time;
		r.sleep();
		ros::spinOnce();
	}
	
	
	return 0;
}
