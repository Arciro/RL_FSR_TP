#include <ros/ros.h>
#include "boost/thread.hpp"
#include "std_msgs/Float64.h"
#include <sensor_msgs/Joy.h>
#include <iostream>

class TeleopJoypad
{
	public:
  		TeleopJoypad();
  		void joypadCallback(const sensor_msgs::Joy::ConstPtr& joy);
  		void teleop();
		void run();
  
  	private:
  		ros::NodeHandle nh;
  		ros::Subscriber joy_sub;
  		ros::Publisher wR_pub;
		ros::Publisher wL_pub;

  		int linear_, angular_;
  		double l_scale_, a_scale_;
  		double d;
  		double pW;
  		double v;
  		double w;
};


TeleopJoypad::TeleopJoypad()
{
	nh.param("axis_linear", linear_, linear_);
	nh.param("axis_angular", angular_, angular_);
	nh.param("scale_angular", a_scale_, a_scale_);
	nh.param("scale_linear", l_scale_, l_scale_);
	
	if (!nh.getParam("wheel_radius", pW))
		pW = 0.032; 
	
	if (!nh.getParam("wheel_separation", d))
		d = 0.145;

	wR_pub = nh.advertise<std_msgs::Float64>("/ddr/rightWheel_velocity_controller/command", 0);
	wL_pub = nh.advertise<std_msgs::Float64>("/ddr/leftWheel_velocity_controller/command", 0);

	joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopJoypad::joypadCallback, this);

}

void TeleopJoypad::joypadCallback(const sensor_msgs::Joy::ConstPtr& joy)
{	
	v = l_scale_*joy->axes[linear_];
	w = a_scale_*joy->axes[angular_];
	/*
	std_msgs::Float64 wR; //angular velocity of right wheel
	std_msgs::Float64 wL; //angular velocity of left wheel
	wR.data = (2*v + d*w)/(2*pW);
	wL.data = (2*v - d*w)/(2*pW);
		
	wR_pub.publish(wR);
	wL_pub.publish(wL);
	twist.angular.z = 
	twist.linear.x = 
	twist_pub.publish(twist);*/
}


void TeleopJoypad::teleop()
{
	ros::Rate r(100);

	while(ros::ok())
	{
		std_msgs::Float64 wR; //angular velocity of right wheel
		std_msgs::Float64 wL; //angular velocity of left wheel
		wR.data = (2*v + d*w)/(2*pW);
		wL.data = (2*v - d*w)/(2*pW);
		
		wR_pub.publish(wR);
		wL_pub.publish(wL);
		r.sleep();
	}

}

 
void TeleopJoypad::run()
{
	boost::thread teleop_thread(&TeleopJoypad::teleop, this);
	
	ros::spin();

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ddr_joypad_node");
  TeleopJoypad teleop_joypad_ddr;
  teleop_joypad_ddr.run();

  return 0; //ros::spin();
}
