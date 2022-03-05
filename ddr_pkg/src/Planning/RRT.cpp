#include "ddr_pkg/planner.h"


/* *** Funzione che implementa il bi-directional Rapidly-exploring Random Tree *** */
void RRT_Astar::rrt(Tree& Tstart, Tree& Tgoal)
{
	ros::Rate r(10);
	//GENERATION TS
	while(counter < MAX_NUM_ITERATIONS)
	{
		generate_tree(Tstart);
		r.sleep();
	}
	
	//GENERATION TG 
	while(counter < MAX_NUM_ITERATIONS*2 ) //qui avremo che counter è uguale a MAX_NUM_ITERATIONS
	{			
		if(counter == MAX_NUM_ITERATIONS*2 - 1)
			trees_generated = true;
				
		generate_tree(Tgoal);
		r.sleep();
	}

}



//funzione che genera l'albero
void RRT_Astar::generate_tree(Tree& albero)
{
	random_device rd;
	uniform_int_distribution<int> dist(-100, 100);

	Point2D q_rand;
	Nodo q_near, q_new;

	counter++;
	
	do{
		q_rand.x = dist(rd);
		q_rand.y = dist(rd);
		cout<<"<--- q_rand --->"<<endl;
		cout<<" x: "<<q_rand.x<<endl;
		cout<<" y: "<<q_rand.y<<endl<<endl;

		find_qnear(q_rand, q_near, albero);
		cout<<"<--- q_near --->"<<endl;
		cout<<" ID: "<<q_near.id<<endl;
		cout<<" x: "<<q_near.position.x<<endl;
		cout<<" y: "<<q_near.position.y<<endl<<endl;

		if(counter > MAX_NUM_ITERATIONS)
			q_new.id = 200 - MAX_NUM_ITERATIONS + counter; 
		
		else
			q_new.id = counter;

		calculate_qnew(q_new, q_near, q_rand, delta);
	}while(!collision_check(q_near, q_new, delta));
	
	//display_node(q_new);
	
	q_new.visited = false;
	q_new.adj_list.push_back(q_near.id);
	cout<<"<--- q_new --->"<<endl;
	cout<<" ID: "<<q_new.id<<endl;
	cout<<" x: "<<q_new.position.x<<endl;
	cout<<" y: "<<q_new.position.y<<endl<<endl;
	albero.push_back(q_new);
				
	for(int i=0; i<albero.size(); i++)
		if(albero[i].id == q_near.id)
			albero[i].adj_list.push_back(q_new.id);

	if(counter > MAX_NUM_ITERATIONS)
		visualization_arch_node(q_near, q_new, 0.0, 0.0, 1.0);
		
	else
		visualization_arch_node(q_near, q_new, 1.0, 0.0, 0.0);
}



void RRT_Astar::find_qnear(Point2D p, Nodo& qn, Tree t)
{
	double min_dist;
	double current_dist;
	int index = 0;
	
	for(int i=0; i<t.size(); i++)
	{
		if(i == 0)
			min_dist = get_distance(p, t[i].position); 
		
		current_dist = get_distance(p, t[i].position);
		if(current_dist < min_dist)
		{
			min_dist = current_dist;
			index = i;
		}
			
	}//fine for
	
	qn.id = t[index].id;
	qn.position.x = t[index].position.x;
	qn.position.y = t[index].position.y;
}


void RRT_Astar::calculate_qnew(Nodo& new_q, Nodo near_q, Point2D qr, float d)
{	
	//declarations of 2 points
	double x2 = qr.x;
	double y2 = qr.y;
	double x1 = near_q.position.x;
	double y1 = near_q.position.y;

	
	double dist;
	dist = get_distance(qr, near_q.position);
	
	if(dist > d)
	{
		Point2D versore;
		versore.y = (y2-y1)/dist;
		versore.x = (x2-x1)/dist;
	
		new_q.position.x = (versore.x * d) + near_q.position.x;
		new_q.position.y = (versore.y * d) + near_q.position.y;
	}
	
	else
		new_q.position = qr; 
}




void RRT_Astar::joining_bridge(Tree& tree_start, Tree& tree_goal)
{
	ros::Rate r(10);
	int row = 0;
	int column = 0;
	bool change_tree = false;
	
	do{
		int dim_Ts = tree_start.size();
		int dim_Tg = tree_goal.size();
		double dist_mat[dim_Ts][dim_Tg];
		double min_mat, curr_mat;

		for(int i=0; i<tree_start.size(); i++)
			for(int j=0; j<tree_goal.size(); j++)
				dist_mat[i][j] = get_distance(tree_start[i].position, tree_goal[j].position);
		
		for(int i=0; i<tree_start.size(); i++)
		{
			for(int j=0; j<tree_goal.size(); j++)
			{
				if(i==0 && j==0)
					min_mat = get_distance(tree_start[i].position, tree_goal[j].position);
				
				curr_mat = get_distance(tree_start[i].position, tree_goal[j].position);
				if(curr_mat < min_mat)
				{
					min_mat = curr_mat;
					row = i;
					column = j;
				}//fine if
			}//fine for innestato
		}//fine for
	
		tree_start[row].adj_list.push_back(tree_goal[column].id);
		tree_goal[column].adj_list.push_back(tree_start[row].id);
	
		if(!collision_check(tree_start[row], tree_goal[column], get_distance(tree_start[row].position, tree_goal[column].position)))
		{
			tree_start[row].adj_list.pop_back();
			tree_goal[column].adj_list.pop_back();
			for(int i=0; i<5; i++)
			{
				if(change_tree)
				{
					generate_tree(tree_goal);
					if(i==4)
						change_tree = false;
				}
				
				else
				{
					generate_tree(tree_start);
					if(i==4)
						change_tree = true;
				}
				r.sleep();	
			}
		}
		
	}while(!collision_check(tree_start[row], tree_goal[column], get_distance(tree_start[row].position, tree_goal[column].position)));
		
	trees_generated = false;
	RRTend = true;

	visualization_msgs::Marker bridge; 
	geometry_msgs::Point p1, p2;
	
	uint32_t bridge_shape = visualization_msgs::Marker::ARROW;
	bridge.header.frame_id = "map";
	bridge.header.stamp = ros::Time::now();

  	bridge.ns = "join_bridge";
   bridge.id = 2000;

   bridge.type = bridge_shape;
   bridge.action = visualization_msgs::Marker::ADD;
   
   p1.x = tree_start[row].position.x;
   p1.y = tree_start[row].position.y;
   p1.z = 0;
   
   p2.x = tree_goal[column].position.x;
   p2.y = tree_goal[column].position.y;
   p2.z = 0;
   
   bridge.points.push_back(p1);
   bridge.points.push_back(p2);

	bridge.scale.x = 0.05;
  	bridge.scale.y = 0.1;
   bridge.scale.z = 0.1;
   
   bridge.pose.orientation.x = 0.0;
   bridge.pose.orientation.y = 0.0;
   bridge.pose.orientation.z = 0.0;
   bridge.pose.orientation.w = 1.0;
   
   bridge.color.r = 0.0;
	bridge.color.g = 1.0;
	bridge.color.b = 0.0;
	bridge.color.a = 1.0;
	
	bridge.lifetime = ros::Duration();
	
	marker_pub.publish(bridge);
}


// semplice funzione che calcola la distanza tra 2 punti
double RRT_Astar::get_distance(Point2D a, Point2D b)
{
	double d;
	d = sqrt(pow(b.x - a.x, 2) + pow(b.y-a.y, 2));
	return d;
}


bool RRT_Astar::collision_check(Nodo node_near, Nodo node_new, double dist)
{	
	float passo = 20;
	float discretization = dist/passo; //andiamo a campionare il percorso con 5 punti 

	if(map->data[toIndex(node_new.position)] == 100) //se la cella è vuota allora NON c'è collisione
		return false;
		
	else
	{
		for(int i=1; i<passo; i++)
		{
			Nodo temp; //nodo temporaneo di cui vogliamo controllare la collisione
			
			//qui calcoliamo le coordinate del campione che si trova tra qnear e qnew
			calculate_qnew(temp, node_near, node_new.position, discretization*i);
			if(map->data[toIndex(temp.position)] == 100)
				return false;
		}
	}
		
	return true;
}


int RRT_Astar::toIndex(Point2D p)
{
	int index, alfa, beta;
	//alfa e beta sarebbero i pixel della mappa.png corrispondenti ai punti di gazebo p
	alfa = round((p.x - map_origin.x)/map_resolution);
	beta = round((p.y - map_origin.y)/map_resolution);
	//beta = map_height - beta;
	
	//questa è la formula che permette di individare l'indice del vettore data[] corrispondente a questo punto
	index = beta*map_width + alfa;
	return index;
}


