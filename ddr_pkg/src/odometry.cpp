#include "ddr_pkg/odometry.h"

Odom::Odom()
{
	if (!nh.getParam("wheel_radius", pW))
		pW = 0.032; 
	
	if (!nh.getParam("wheel_separation", d))
		d = 0.045;
	
	odom_pub = nh.advertise<nav_msgs::Odometry>("/ddr/odom", 1);
	wheels_sub = nh.subscribe("/ddr/joint_states", 1, &Odom::wheels_callback, this);
}


void Odom::wheels_callback(const sensor_msgs::JointState wheels_msg)
{
	wL = wheels_msg.velocity[0];
	wR = wheels_msg.velocity[1];
}


void Odom::range_kutta()
{
	double freq = 100;
	double Ts = 1/freq;
	
	double x0 = 0;
	double y0 = 0;
	double theta0 = 0;
	
	double xk, xk_succ;
	double yk, yk_succ;
	double thetak, thetak_succ;
	
	xk = x0;
	yk = y0;
	thetak = theta0;
	
	ros::Rate r(freq);
	while(ros::ok())
	{
		tf::TransformBroadcaster odom_broadcaster;
		ros::Time now = ros::Time::now();
		
		//calculate linear and angular velocity from right and left wheels' velocities
		double vk = (pW/2)*(wR+wL);
		double wk = (pW/d)*(wR-wL);
		
		//range-kutta
		xk_succ = xk + vk*Ts*cos(thetak + (wk*Ts)/2);
		yk_succ = yk + vk*Ts*sin(thetak + (wk*Ts)/2);
		thetak_succ = thetak + wk*Ts;
		
		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion quat_msg;
		quat_msg = tf::createQuaternionMsgFromYaw(thetak_succ);
		
		//first, we'll publish the transform over tf
    	geometry_msgs::TransformStamped odom_trans;
    	odom_trans.header.stamp = now;
    	odom_trans.header.frame_id = "odom";
    	odom_trans.child_frame_id = "base_footprint";

    	odom_trans.transform.translation.x = xk_succ;
    	odom_trans.transform.translation.y = yk_succ;
    	odom_trans.transform.translation.z = 0.0;
    	odom_trans.transform.rotation = quat_msg;

    	//send the transform
    	odom_broadcaster.sendTransform(odom_trans);
		
		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom_msg;
		odom_msg.header.stamp = now;
		odom_msg.header.frame_id = "odom";
		odom_msg.child_frame_id = "base_footprint";
		
		//set position and orientation
		odom_msg.pose.pose.position.x = xk_succ;
		odom_msg.pose.pose.position.y = yk_succ;
		odom_msg.pose.pose.position.z = 0;
		odom_msg.pose.pose.orientation = quat_msg; 
		
		//set the velocity
		odom_msg.twist.twist.linear.x = vk;
		odom_msg.twist.twist.linear.y = 0;
		odom_msg.twist.twist.linear.z = 0;
		odom_msg.twist.twist.angular.x = 0;
		odom_msg.twist.twist.angular.y = 0;
		odom_msg.twist.twist.angular.z = wk;
		
		xk = xk_succ;
		yk = yk_succ;
		thetak = thetak_succ;
		
		//publish the message
		odom_pub.publish(odom_msg);
		r.sleep();
	}

}


void Odom::run()
{
	boost::thread odometry_thread(&Odom::range_kutta, this);
	
	ros::spin();
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "odom_node");

	Odom odometria;
	odometria.run();

	return 0;
}
