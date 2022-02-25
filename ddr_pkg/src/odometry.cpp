#include "ddr_pkg/odometry.h"

Odom::Odom()
{
	if (!nh.getParam("wheel_radius", pW))
		pW = 0.032; 
	
	if (!nh.getParam("wheel_separation", d))
		d = 0.045;
	
	odom_pub = nh.advertise<nav_msgs::Odometry>("/ddr/odom", 0);
	wheels_sub = nh.subscribe("/ddr/joint_states", 0, &Odom::wheels_callback, this);
}


void Odom::wheels_callback(const sensor_msgs::JointState::ConstPtr& wheels_msg)
{
	for (int i=0; i<2; i++)
		q[i] = wheels_msg->position[i];
	
/*	wL = wheels_msg->velocity[0];
	//cout<<"\n wL: "<<wL;
	wR = wheels_msg->velocity[1];
	//cout<<"\n wR: "<<wR;*/
}


void Odom::range_kutta()
{
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
		thetak = thetak + wk*0.001; //diviso 6?
		
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