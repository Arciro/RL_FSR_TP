#include "ddr_pkg/planner.h"


void RRT_Astar::visualization_arch_node(Nodo q_vicino, Nodo q_nuovo, float red, float green, float blue)
{
	visualization_msgs::Marker nodes; //marker per i nodi
	visualization_msgs::Marker archs; //marker per gli archi
	visualization_msgs::Marker id_marker; //marker testuali per contrassegnare i nodi
	visualization_msgs::MarkerArray rviz_marker;
	
	geometry_msgs::Point p1, p2;
	
	uint32_t node_shape = visualization_msgs::Marker::SPHERE;
	uint32_t arch_shape = visualization_msgs::Marker::ARROW;
   uint32_t text = visualization_msgs::Marker::TEXT_VIEW_FACING;
   
   nodes.header.frame_id = archs.header.frame_id = id_marker.header.frame_id = "map";
   nodes.header.stamp = archs.header.stamp = id_marker.header.stamp = ros::Time::now();

   nodes.ns = archs.ns = id_marker.ns = "graph";
   nodes.id = q_nuovo.id;
   id_marker.id = 800+counter;
   
   if(counter > MAX_NUM_ITERATIONS)
   	archs.id = 600 + counter; 
   else
   	archs.id = 400 + counter;
   
   nodes.type = node_shape;
   archs.type = arch_shape;
   id_marker.type = text;
   nodes.action = archs.action = id_marker.action = visualization_msgs::Marker::ADD;

   nodes.pose.position.x = id_marker.pose.position.x = q_nuovo.position.x;
   nodes.pose.position.y = id_marker.pose.position.y = q_nuovo.position.y;
   nodes.pose.position.z = 0;
   id_marker.pose.position.z = 0.1;
   nodes.pose.orientation.x = 0.0;
   nodes.pose.orientation.y = 0.0;
   nodes.pose.orientation.z = 0.0;
   nodes.pose.orientation.w = 1.0;
   
   p1.x = q_vicino.position.x;
   p1.y = q_vicino.position.y;
   p1.z = 0;
   
   p2.x = q_nuovo.position.x;
   p2.y = q_nuovo.position.y;
   p2.z = 0;
   
   archs.points.push_back(p1);
   archs.points.push_back(p2);

   archs.scale.x = 0.05;
   archs.scale.y = 0.1;
   archs.scale.z = 0.1;

   nodes.scale.x = 0.15;
   nodes.scale.y = 0.15;
   nodes.scale.z = 0.15;
   
   id_marker.scale.z = 0.08;

   nodes.color.r = archs.color.r = red;
   nodes.color.g = archs.color.g = green;
   nodes.color.b = archs.color.b = blue;
   nodes.color.a = archs.color.a = id_marker.color.a = 1.0;
	id_marker.color.r = 0.0;
	id_marker.color.g = 1.0;
	id_marker.color.b = 0.0; 
	
	id_marker.text = to_string(nodes.id);
	
   nodes.lifetime = archs.lifetime = id_marker.lifetime = ros::Duration(20.0);
	
	rviz_marker.markers.push_back(nodes);
   rviz_marker.markers.push_back(archs);
   rviz_marker.markers.push_back(id_marker);
	
	array_marker_pub.publish(rviz_marker);
}




void RRT_Astar::visualization_minimum_path(Nodo* n, float red, float green, float blue)
{
	visualization_msgs::Marker nodes; //marker per i nodi
	
	uint32_t node_shape = visualization_msgs::Marker::SPHERE;
   
   nodes.header.frame_id = "map";
   nodes.header.stamp = ros::Time::now();

   nodes.ns = "graph";
   nodes.id = n->id + 1000;
   
   nodes.type = node_shape;

   nodes.action = visualization_msgs::Marker::ADD;

   nodes.pose.position.x = n->position.x;
   nodes.pose.position.y = n->position.y;
   nodes.pose.position.z = 0.2;
   nodes.pose.orientation.x = 0.0;
   nodes.pose.orientation.y = 0.0;
   nodes.pose.orientation.z = 0.0;
   nodes.pose.orientation.w = 1.0;

   nodes.scale.x = 0.15;
   nodes.scale.y = 0.15;
   nodes.scale.z = 0.15;

   nodes.color.r = red;
   nodes.color.g = green;
   nodes.color.b = blue;
   nodes.color.a = 1.0;
	
   nodes.lifetime = ros::Duration(20.0);

	marker_pub.publish(nodes);
}



/* funzione per visualizzare piccoli cerchi sull'immagine della mappa con OpenCV
void RRT_Astar::display_node(Nodo n)
{
	int pixel_x, pixel_y;
	pixel_x = round((n.position.x - map_origin.x)/map_resolution);
	pixel_y = round((n.position.y - map_origin.y)/map_resolution);
	pixel_y = map_height - pixel_y; 
	
	circle(img, Point(pixel_x, pixel_y), 3, Scalar(0,69,255), FILLED);
}*/

