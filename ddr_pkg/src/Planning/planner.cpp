#include "ddr_pkg/planner.h"

RRT_Astar::RRT_Astar()
{
	counter = 0;
	if (!nh.getParam("delta", delta))
		delta = 0.8;
	
	current_position.x = 0.0;
	current_position.y = 0.0;
	destination.x = 2.0;
	destination.y = -3.0;
	RRTend = false;
	Astar_end = false;
	
	first_odom = false;
	map_received = false;
	trees_generated = false;
	qrcode_read = false;
	ctrl_end = false;
	
	map_height = 608;
	map_width = 512;
	map_resolution = 0.05;
	map_origin.x = -12.2;
	map_origin.y = -15.4;
	
/*	string path_package = ros::package::getPath("ddr_pkg");
	string image_path = (path_package + "/maps/scenario.png");
	img = imread(image_path, IMREAD_COLOR);*/
	
	odom_sub = nh.subscribe("/ddr/odom", 0, &RRT_Astar::odometry_callback, this);
	map_sub = nh.subscribe("/map", 1, &RRT_Astar::map_callback, this);
	barcode_sub = nh.subscribe("/barcode", 1, &RRT_Astar::barcode_callback, this);
	
	//path_pub = nh.advertise<nav_msgs::Path>("/path", 1);
	array_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
	marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	
	server = nh.advertiseService("ctrl_finished_topic", &RRT_Astar::ctrl2plan_callback, this);
	client = nh.serviceClient<ddr_pkg::plan_to_ctrl>("plan_finished_topic");
}


void RRT_Astar::odometry_callback(nav_msgs::Odometry odom)
{	
	current_position.x = odom.pose.pose.position.x;
	current_position.y = odom.pose.pose.position.y;

	first_odom = true;
}


void RRT_Astar::map_callback(const nav_msgs::OccupancyGrid::Ptr & map_msg)
{
	map = map_msg;
	map_origin.x = map->info.origin.position.x;
	map_origin.y = map->info.origin.position.y;
	
	map_height = map->info.height;
	map_width = map->info.width;
	map_resolution = map->info.resolution;
	
	map_received = true;
}



void RRT_Astar::barcode_callback(const std_msgs::String::Ptr& qrcode)
{
	room = qrcode->data.c_str();
	
	if(room == "r1")
	{
		destination.x = 1.1;
		destination.y = 3.5;
	//	cout<<" ROOM 1"<<endl;
	}
	
	else if(room == "r2")
	{
		destination.x = -1.0;
		destination.y = 4.0;
		cout<<" ROOM 2"<<endl;
	}

	qrcode_read = true;
}


bool RRT_Astar::ctrl2plan_callback(ddr_pkg::ctrl_to_plan::Request &req, ddr_pkg::ctrl_to_plan::Response &res)
{
	res.plan_run = "Dato che il controller ha finito, il planner puo' ripartire";
	if(req.goal_achieved)
	{
		cout<<"\n "<<res.plan_run<<endl;
		ctrl_end = true;
	}
	
	return true;
}



void RRT_Astar::planner()
{
	while(!first_odom && !map_received)	
		sleep(1);

	ros::Rate r(100);
	
	Tree Ts, Tg;
	
	Nodo qs; //start configuration
	Nodo qg; //goal configuration
	
	final_position = destination;
	
	while(ros::ok())
	{		
		if(qrcode_read)
		{
			qrcode_read = false;
			if(ctrl_end)
			{
				cout<<" Obiettivo raggiunto"<<endl;
				ctrl_end = false;
				final_position = destination;
				cout<<" Punto finale: "<<final_position.x<<" "<<final_position.y<<endl;
				counter = 0;
			}
		}
		
		if(counter == 0)
		{
			Ts.clear();
			qs.id = counter;
			qs.visited = false;
			qs.position = current_position;
			qs.parent = 0;
			Ts.push_back(qs);
			//display_node(qs);
			
			Tg.clear();
			qg.id = 200 + counter;
			qg.visited = false;
			qg.position = final_position;
			qg.parent = 0;
			Tg.push_back(qg);
			//display_node(qg);
		}

		rrt(Ts, Tg);
		
		if(trees_generated)
			joining_bridge(Ts, Tg);

		if(RRTend)
		{
			int dim_Ts = Ts.size();
			int dim_Tg = Tg.size();
			Tree graph;
		
			for(int i=0; i<dim_Ts; i++)
				graph.push_back(Ts[i]);
		
			for(int j=0; j<dim_Tg; j++)
				graph.push_back(Tg[j]);
			
			Astar_search(graph);
			RRTend = false;
			Astar_end = true;
		}
		
		/*
		imshow("Display window", img);
		waitKey(0);*/

		if(Astar_end)
		{
			ddr_pkg::plan_to_ctrl srv;
			for(int k=1; k<=trajectory.size(); k++)
			{
				geometry_msgs::PoseStamped wp;

				wp.pose.position.x = trajectory[trajectory.size()-k].position.x;
				wp.pose.position.y = trajectory[trajectory.size()-k].position.y;
				wp.pose.position.z = 0;

				srv.request.path.poses.push_back(wp);
			}
			
			if(client.call(srv))
				cout<<"\n Il planner sta mandando il percorso al controller"<<endl;
				
			else
				ROS_ERROR("Fallimento nel chiamare il service");
			
			Astar_end = false;	
		}
	/*	nav_msgs::Path path_graph;
		
		for(int j=1; j<=trajectory.size(); j++)
		{
			geometry_msgs::PoseStamped wp_pub;
			
			wp_pub.pose.position.x = trajectory[trajectory.size()-j].position.x;
			wp_pub.pose.position.y = trajectory[trajectory.size()-j].position.y;
			wp_pub.pose.position.z = 0;
			
			path_graph.poses.push_back(wp_pub);
		}

		
		path_pub.publish(path_graph);*/

		r.sleep();
	}
	
}//fine funzione planner



void RRT_Astar::run()
{
	boost::thread planner_thread(&RRT_Astar::planner, this);

	ros::spin();
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "planner_node");
	
	RRT_Astar plan;
	plan.run();
	
	return 0;
}

