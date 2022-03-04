#include "ddr_pkg/odometry.h"

Odom::Odom()
{
	if (!nh.getParam("wheel_radius", pW))
		pW = 0.032; 
	
	if (!nh.getParam("wheel_separation", d))
		d = 0.142;
		
	first_wheel = false;
	
	odom_pub = nh.advertise<nav_msgs::Odometry>("/ddr/odom", 0);
	wheels_sub = nh.subscribe("/ddr/joint_states", 0, &Odom::wheels_callback, this);
}


void Odom::wheels_callback(const sensor_msgs::JointState::ConstPtr& wheels_msg)
{
	qL = wheels_msg->position[0];
	qR = wheels_msg->position[1];
	
	if(!first_wheel)
	{
		q0L = wheels_msg->position[0];
		q0R = wheels_msg->position[1];
	}
	
	wL = wheels_msg->velocity[0];
	wR = wheels_msg->velocity[1];
	
	first_wheel = true;
}


void Odom::range_kutta()
{
	while(!first_wheel)	
		sleep(1);	
	
	double freq = 100;
	double x0, y0, theta0;
	x0 = y0 = theta0 = 0;
	
	double xk = x0;
	double yk = y0;
	double thetak = theta0;
	
	double delta_phi_L, delta_phi_R;
	
	ros::Time current_time;
	
	ros::Rate r(freq);
	while(ros::ok())
	{
		static tf::TransformBroadcaster odom_broadcaster;
		current_time = ros::Time::now();

		//calculate linear and angular velocity from right and left wheels' velocities
		delta_phi_L = qL - q0L;
		delta_phi_R = qR - q0R;
		q0L = qL;
		q0R = qR;

		double vk = (pW/2.0)*(wR+wL);
		double wk = (pW/d)*(wR-wL);
		double delta_s = (pW/2.0)*(delta_phi_R + delta_phi_L);
		double delta_theta = (pW/d)*(delta_phi_R - delta_phi_L);
		
		//exact integration
		if(delta_theta != 0)
		{
			xk = xk + (delta_s/delta_theta)*(sin(thetak + delta_theta) - sin(thetak));
			yk = yk - (delta_s/delta_theta)*(cos(thetak + delta_theta) - cos(thetak));
		}

		//range-kutta
		else
		{
			xk = xk + delta_s*cos(thetak + (delta_theta)/2.0);
			yk = yk + delta_s*sin(thetak + (delta_theta)/2.0);
		}
		
		thetak = thetak + delta_theta;
		
		if(thetak >= M_PI)
			thetak = thetak - 2*M_PI;
			
		else if(thetak <= -M_PI)
			thetak = thetak + 2*M_PI;
		
		//cout<<" Theta: "<<thetak<<endl;
		
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
