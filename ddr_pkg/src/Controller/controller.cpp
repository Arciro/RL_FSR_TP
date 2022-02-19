#include "ddr_pkg/controller.h"


TrackReg::TrackReg()
{
	first_odom = false; 
	path_received = false;
	
	if (!nh.getParam("wheel_radius", pW))
		pW = 0.032; 
	
	if (!nh.getParam("wheel_separation", d))
		d = 0.045;
	
	if (!nh.getParam("track_k1", track_k1))
		track_k1 = 0.1;
		
	if (!nh.getParam("track_k2", track_k2))
		track_k2 = 0.1;
	
	if (!nh.getParam("b", b))
		b = 0.1;
		
	if (!nh.getParam("reg_k1", reg_k1))
		reg_k1 = 0.1;
		
	if (!nh.getParam("reg_k2", reg_k2))
		reg_k2 = 0.1;
	
	t = 0.0;
	time_traj = 3.0;
	
	odom_sub = nh.subscribe("/ddr/odom", 1, &TrackReg::odometry_callback, this);
	path_sub = nh.subscribe("/path", 1, &TrackReg::path_callback, this);
	vel_pub = nh.advertise<geometry_msgs::Twist>("/ddr/cmd_vel", 1);
}



void TrackReg::odometry_callback(nav_msgs::Odometry odom)
{	
	geometry_msgs::Pose2D posa;
	
	posa.x = odom.pose.pose.position.x;
	posa.y = odom.pose.pose.position.y;
	
	double x = odom.pose.pose.orientation.x;
	double y = odom.pose.pose.orientation.y;
	double z = odom.pose.pose.orientation.z;
	double w = odom.pose.pose.orientation.w;
	tf::Quaternion q(x, y, z, w);

	double roll, pitch;
	tf::Matrix3x3(q).getRPY(roll, pitch, posa.theta);
	theta = posa.theta;
	
	y1 = posa.x + b*cos(theta);
	y2 = posa.y + b*sin(theta);
	
	first_odom = true;
}



void TrackReg::path_callback(nav_msgs::Path path_msg)
{
	wp_list.clear();
	
	for(int i=0; i<path_msg.poses.size(); i++)
		wp_list.push_back(path_msg.poses[i].pose.position);
	
	path_received = true;
}



void TrackReg::ctrl_loop()
{
	while(!first_odom && !path_received)	
		sleep(1);

	ros::Rate r(100);
	
	bool move = false;
	bool finish = false;
	
	int wp_index = 0;
	
	while(ros::ok())
	{
		double xi, xf, yi, yf;
		double s, s_dot;
	
		double y1d, y2d;
		double dot_y1d, dot_y2d;
		double u1, u2;
	
		double gamma;
		
		geometry_msgs::Twist cmd;
		double v;
		double w;
		
		double norma, err;
		
		if((wp_index < wp_list.size()-1) && !finish)
		{
			xi = wp_list[wp_index].x;
			yi = wp_list[wp_index].y;
			xf = wp_list[wp_index+1].x;
			yf = wp_list[wp_index+1].y;
		
			norma = sqrt(pow(xf-xi, 2) + pow(yf-yi, 2));
			err = sqrt( pow(xf-y1, 2) + pow(yf-y2, 2) );
			cout<<"\n ERR: "<<fabs(err)<<endl;
			cout<<" INDEX: "<<wp_index<<endl;
			cout<<" T: "<<t<<endl;
			cout<<" DIM WP LIST: "<<wp_list.size()<<endl;
		
			if(fabs(err)<0.05 && t>=time_traj)
			{
				move = false;
				wp_index++;
				if(wp_index == wp_list.size()-1)
					finish = true;
			}
		
			if(!move && (wp_index+1 != wp_list.size()))
			{
				cout<<" REGULATION"<<endl;
				gamma = atan2(wp_list[wp_index+1].y - wp_list[wp_index].y, wp_list[wp_index+1].x - wp_list[wp_index].x) - theta;
				//---check the rotation with smaller motion    
   			if (fabs(gamma) > M_PI) 
  				{
					if(gamma > 0.0) 
						gamma = gamma - 2*M_PI;
      			else 
      				gamma = gamma + 2*M_PI;
				}	

				v = 0.0;
      		w = reg_k2*gamma + reg_k1*sin(gamma)*cos(gamma);
      	
      		if(fabs(gamma) < 0.2)
      			move = true;
			
				t = 0.0;
			}
		
			else
			{
				cout<<" TRACKING"<<endl;
				tvp(0, norma, time_traj, s, s_dot);
		
				y1d = xi + s*(xf-xi)/norma;
				y2d = yi + s*(yf-yi)/norma;
			
				dot_y1d = s_dot*(xf-xi)/norma;
				dot_y2d = s_dot*(yf-yi)/norma;
		
				u1 = dot_y1d + track_k1*(y1d-y1);
				u2 = dot_y2d + track_k2*(y2d-y2);

				v = cos(theta)*u1 + sin(theta)*u2;
				w = (-sin(theta)*u1 + cos(theta)*u2)/b;
			
				t += 0.01;
			}
			
		}
		
		else
		{
			cout<<" SI RUOTA!!!!"<<endl;
			v = 0.0;
			w = M_PI/2;
			//t += 0.01;
			//cout<<" t: "<<t<<endl;
			/*
			if(t >= 8)
			{
				cout<<" SI RICOMINCIA DA ZERO"<<endl;
				v = w = 0.0;
				if(path_received && t>=10)
				{
					wp_index = 0;
					t = 0.0;
				}
			}*/
		}
		
		wR = (2*v + d*w)/(2*pW);
		wL = (2*v - d*w)/(2*pW);
		
		cmd.linear.x = v;
		cmd.angular.z = w;
		vel_pub.publish(cmd);
		r.sleep();
	}
}


void TrackReg::run()
{
	boost::thread controller_thread(&TrackReg::ctrl_loop, this);

	ros::spin();
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "controller_node");
	
	TrackReg ctrl;
	ctrl.run();
	
	return 0;
}

