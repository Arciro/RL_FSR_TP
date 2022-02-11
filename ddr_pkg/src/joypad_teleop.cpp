#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <iostream>

class TeleopJoypad
{
	public:
  		TeleopJoypad();
  		void joypadCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  	private:
  		ros::NodeHandle nh_;

  		int linear_, angular_;
  		double l_scale_, a_scale_;
  		ros::Publisher twist_pub;
  		ros::Subscriber joy_sub;
  
};


TeleopJoypad::TeleopJoypad()
{

	nh_.param("axis_linear", linear_, linear_);
	nh_.param("axis_angular", angular_, angular_);
	nh_.param("scale_angular", a_scale_, a_scale_);
	nh_.param("scale_linear", l_scale_, l_scale_);


	twist_pub = nh_.advertise<geometry_msgs::Twist>("ddr/cmd_vel", 1);

	joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoypad::joypadCallback, this);

}

void TeleopJoypad::joypadCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  twist.angular.z = a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];
  twist_pub.publish(twist);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ddr_joypad_node");
  TeleopJoypad teleop_joypad_ddr;

  ros::spin();
}
