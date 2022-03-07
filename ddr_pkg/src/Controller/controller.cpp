#include "ddr_pkg/controller.h"


TrackReg::TrackReg()
{
	first_odom = false; 
	path_received = false;
	
	if (!nh.getParam("wheel_radius", pW))
		pW = 0.032; 
	
	if (!nh.getParam("wheel_separation", d))
		d = 0.142;
	
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
	time_traj = 5.0;
	
	odom_sub = nh.subscribe("/ddr/odom", 0, &TrackReg::odometry_callback, this);
	//path_sub = nh.subscribe("/path", 1, &TrackReg::path_callback, this);
	vel_pub = nh.advertise<geometry_msgs::Twist>("/ddr/cmd_vel", 1);
	
	wR_pub = nh.advertise<std_msgs::Float64>("/ddr/rightWheel_velocity_controller/command", 0);
	wL_pub = nh.advertise<std_msgs::Float64>("/ddr/leftWheel_velocity_controller/command", 0);
	
	client = nh.serviceClient<ddr_pkg::ctrl_to_plan>("ctrl_finished_topic");
	server = nh.advertiseService("plan_finished_topic", &TrackReg::path_callback, this);
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


/*
void TrackReg::path_callback(nav_msgs::Path path_msg)
{
	wp_list.clear();
	
	for(int i=0; i<path_msg.poses.size(); i++)
		wp_list.push_back(path_msg.poses[i].pose.position);
	
	path_received = true;
}
*/

bool TrackReg::path_callback(ddr_pkg::plan_to_ctrl::Request &req, ddr_pkg::plan_to_ctrl::Response &res)
{
	wp_list.clear();
	//cout<<"\n NELLA CALBACK CI ENTRI IMMEDIATAMENTE?"<<endl;
	for(int i=0; i<req.path.poses.size(); i++)
		wp_list.push_back(req.path.poses[i].pose.position);
	
	res.ctrl_run = "Il planner ha pianificato il percorso, ora il controller può partire";
	cout<<"\n "<<res.ctrl_run<<endl;
	path_received = true;
	wp_index = 0;
	
	return true;
}


void TrackReg::ctrl_loop()
{
	while((!first_odom) || (!path_received))	
		sleep(1);
	//cout<<"\n CICLO INIZIA SUBITO?"<<endl;
	ros::Rate r(100);
	
	bool move = false;
	bool msg2plan;
	
	while(ros::ok())
	{
		double xi, xf, yi, yf;
		double s, s_dot;
	
		double y1d, y2d;
		double dot_y1d, dot_y2d;
		double u1, u2;
	
		double gamma;
		
		double v, w;		
		double norma, err;
		
		if((wp_index < wp_list.size()-1) && path_received)
		{
			msg2plan = false;
			xi = wp_list[wp_index].x;
			yi = wp_list[wp_index].y;
			xf = wp_list[wp_index+1].x;
			yf = wp_list[wp_index+1].y;
		
			norma = sqrt(pow(xf-xi, 2) + pow(yf-yi, 2));
			err = sqrt( pow(xf-y1, 2) + pow(yf-y2, 2));
	/*		cout<<"\n ERR: "<<fabs(err)<<endl;
			cout<<" INDEX: "<<wp_index<<endl;
			cout<<" T: "<<t<<endl;
			cout<<" DIM WP LIST: "<<wp_list.size()<<endl;*/
		
			if(fabs(err)<0.05 && t>=time_traj)
			{
				move = false;
				wp_index++;
				if(wp_index == wp_list.size()-1)
					path_received = false;
			}
		
			if(!move && (wp_index+1 != wp_list.size()))
			{
			//	cout<<" REGULATION"<<endl;
				gamma = atan2(wp_list[wp_index+1].y - wp_list[wp_index].y, wp_list[wp_index+1].x - wp_list[wp_index].x) - theta;
				    
   			if (fabs(gamma) > M_PI) 
  				{
					if(gamma > 0.0) 
						gamma = gamma - 2*M_PI;
      			else 
      				gamma = gamma + 2*M_PI;
				}	

				v = 0.0;
      		w = reg_k2*gamma + reg_k1*sin(gamma)*cos(gamma);
      	
      		if(fabs(gamma) < 0.15)
      			move = true;
			
				t = 0.0;
			}
		
			else
			{
			//	cout<<" TRACKING"<<endl;
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
			//cout<<" FINAL REGULATION"<<endl;
			gamma = atan2(0, 1) - theta;
				    
   		if (fabs(gamma) > M_PI) 
  			{
				if(gamma > 0.0) 
					gamma = gamma - 2*M_PI;
      		else 
      			gamma = gamma + 2*M_PI;
			}	

			v = 0.0;
      	w = reg_k2*gamma + reg_k1*sin(gamma)*cos(gamma);
      	
      	double fin_err = sqrt(pow(wp_list[wp_list.size()-1].y - y2, 2) + pow(wp_list[wp_list.size()-1].x - y1, 2));
      	if((fabs(theta) < 0.05) && (fabs(fin_err) < 0.05))
      	{
      		if(!msg2plan)
      		{
      			//cout<<"\n MA QUI QUANTE VOLTE CI ENTRI?"<<endl;
      			msg2plan = true;
      			ddr_pkg::ctrl_to_plan srv;
      			srv.request.goal_achieved = true;
      			if(client.call(srv))
      				cout<<"\n Il controller è terminato, ora tocca al planner."<<endl;
      				
      			else
						ROS_ERROR("Fallimento nel chiamare il service");
      		}
      	}
			
			t = 0.0;
		}

		std_msgs::Float64 wR; //angular velocity of right wheel
		std_msgs::Float64 wL; //angular velocity of left wheel
		
		wR.data = (2*v + d*w)/(2*pW);
		wL.data = (2*v - d*w)/(2*pW);
		
		wR_pub.publish(wR);
		wL_pub.publish(wL);
		
		/*
		geometry_msgs::Twist cmd;
		cmd.linear.x = v;
		cmd.angular.z = w;
		vel_pub.publish(cmd);*/
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

