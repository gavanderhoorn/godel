/*
	Copyright Feb 10, 2014 Southwest Research Institute

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
*/

#include <godel_surface_detection/interactive/interactive_surface_server.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <sstream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

namespace godel_surface_detection {
namespace interactive {

InteractiveSurfaceServer::InteractiveSurfaceServer() :
		marker_name_(defaults::MARKER_SERVER_NAME),
		marker_description_(defaults::MARKER_DESCRIPTION),
		arrow_distance_(defaults::ARROW_DISTANCE),
		arrow_head_diameter_(defaults::ARROW_HEAD_DIAMETER),
		arrow_head_length_(defaults::ARROW_HEAD_LENGTH),
		arrow_length_(defaults::ARROW_LENGTH),
		arrow_shaft_diameter_(defaults::ARROW_SHAFT_DIAMETER)
{
	// TODO Auto-generated constructor stub

}

InteractiveSurfaceServer::~InteractiveSurfaceServer() {
	// TODO Auto-generated destructor stub
}


bool InteractiveSurfaceServer::init()
{

	srand(time(NULL));
	return load_parameters();
}

void InteractiveSurfaceServer::run()
{
	marker_server_ptr_ = interactive_markers::InteractiveMarkerServerPtr(
			new interactive_markers::InteractiveMarkerServer(defaults::MARKER_SERVER_NAME,"",false));

	// create callbacks
	button_callback_ =  interactive_markers::InteractiveMarkerServer::FeedbackCallback(
			boost::bind(&InteractiveSurfaceServer::button_marker_callback,this,_1));

	menu_callback_ = interactive_markers::InteractiveMarkerServer::FeedbackCallback(
				boost::bind(&InteractiveSurfaceServer::menu_marker_callback,this,_1));

	// setup menu handler
	select_entry_id_ =  menu_handler_.insert("Select",menu_callback_);
	unselect_entry_id_ =  menu_handler_.insert("Unselect",menu_callback_);
	interactive_markers::MenuHandler::EntryHandle submenu_handle = menu_handler_.insert("More Options");
	select_all_entry_id_ = menu_handler_.insert(submenu_handle,"Select All",menu_callback_);
	clear_all_entry_id_ =  menu_handler_.insert(submenu_handle,"Clear Selections",menu_callback_);
	hide_entry_id_ = menu_handler_.insert(submenu_handle,"Hide",menu_callback_);
	show_all_entry_id_ = menu_handler_.insert(submenu_handle,"Show All",menu_callback_);

	marker_server_ptr_->applyChanges();
}

bool InteractiveSurfaceServer::load_parameters()
{
	ros::NodeHandle nh("~");
	std::string ns = params::PARAMETER_NS + "/";
	bool succeeded = true;//nh.getParam(ns +params::FRAME_ID,frame_id_);
	return succeeded;
}

void InteractiveSurfaceServer::set_selection_flag(std::string marker_name,bool selected)
{

	visualization_msgs::InteractiveMarker int_marker;
	if(surface_selection_map_.count(marker_name)>0 && marker_server_ptr_->get(marker_name,int_marker))
	{
		surface_selection_map_[marker_name] = selected;
		int_marker.controls[1].markers[0].color.a = selected ? 1 : 0;
		//int_marker.controls[1].always_visible = selected;
		marker_server_ptr_->insert(int_marker);
	}
}

void InteractiveSurfaceServer::toggle_selection_flag(std::string marker_name)
{
	visualization_msgs::InteractiveMarker int_marker;
	bool selected;
	if(surface_selection_map_.count(marker_name)>0 && marker_server_ptr_->get(marker_name,int_marker))
	{
		selected = surface_selection_map_[marker_name];
		int_marker.controls[1].markers[0].color.a = !selected ? 1 : 0;
		surface_selection_map_[marker_name] = !selected;
		//int_marker.controls[1].always_visible = selected;
		marker_server_ptr_->insert(int_marker);
	}
}

void InteractiveSurfaceServer::button_marker_callback(
		const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
	switch(feedback->event_type)
	{
	case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
		ROS_INFO_STREAM("marker "<<feedback->marker_name <<" button control was clicked");
		toggle_selection_flag(feedback->marker_name);
		marker_server_ptr_->applyChanges();
		break;

	case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
		break;
	}
}

void InteractiveSurfaceServer::menu_marker_callback(
		const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{

	typedef std::map<std::string,bool>::iterator SelectionIterator;

	switch(feedback->event_type)
	{
	case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
//		ROS_INFO_STREAM("marker "<<feedback->marker_name <<" button control was clicked");
		toggle_selection_flag(feedback->marker_name);
		marker_server_ptr_->applyChanges();
		break;

	case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
//		ROS_INFO_STREAM("marker: "<<feedback->marker_name <<", entry_id: "<< feedback->menu_entry_id
//				<<", menu control was clicked");

		if(feedback->menu_entry_id == select_entry_id_ )
		{
			set_selection_flag(feedback->marker_name,true);
			marker_server_ptr_->applyChanges();
			return;
		}
		else if(feedback->menu_entry_id == unselect_entry_id_)
		{
			set_selection_flag(feedback->marker_name,false);
			marker_server_ptr_->applyChanges();
			return;

		}
		else if(feedback->menu_entry_id == clear_all_entry_id_)
		{

			for(SelectionIterator i = surface_selection_map_.begin();i!= surface_selection_map_.end();i++)
			{
				set_selection_flag(i->first,false);
			}

			marker_server_ptr_->applyChanges();
			return;

		}
		else if(feedback->menu_entry_id == select_all_entry_id_)
		{
			for(SelectionIterator i = surface_selection_map_.begin();i!= surface_selection_map_.end();i++)
			{
				set_selection_flag(i->first,true);
			}

			marker_server_ptr_->applyChanges();
			return;

		}
		else if(feedback->menu_entry_id == hide_entry_id_)
		{
			ROS_WARN_STREAM("'Hide' menu option has not been implemented yet");

			marker_server_ptr_->applyChanges();
			return;
		}
		else if(feedback->menu_entry_id == show_all_entry_id_)
		{
			ROS_WARN_STREAM("'Show All' menu option has not been implemented yet");

			marker_server_ptr_->applyChanges();
			return;
		}

		break;
	}
}

void InteractiveSurfaceServer::create_arrow_marker(
		const visualization_msgs::Marker& surface_marker,visualization_msgs::Marker& arrow_marker)
{
	// create temporary point cloud
	pcl::PointCloud<pcl::PointXYZ> surface;
	surface.width = surface_marker.points.size();
	surface.height = 1;
	surface.points.resize(surface_marker.points.size());
	for(int i = 0; i < surface_marker.points.size();i++)
	{
		surface.points[i].x = surface_marker.points[i].x;
		surface.points[i].y = surface_marker.points[i].y;
		surface.points[i].z = surface_marker.points[i].z;
	}

	// finding bouding box bounds
	pcl::PointXYZ min,max;
	pcl::getMinMax3D(surface,min,max);

	// create arrow
	arrow_marker.type = arrow_marker.ARROW;
	arrow_marker.scale.x = arrow_shaft_diameter_;
	arrow_marker.scale.y = arrow_head_diameter_;
	arrow_marker.scale.z = arrow_head_length_;
	arrow_marker.color.r = 0;
	arrow_marker.color.g = arrow_marker.color.b = arrow_marker.color.a = 1;
	arrow_marker.points.resize(2);

	// start point
	arrow_marker.points[1].x = 0.5f*(min.x + max.x);
	arrow_marker.points[1].y = 0.5f*(min.y + max.y);
	arrow_marker.points[1].z = max.z + arrow_distance_;

	// end point
	arrow_marker.points[0].x = arrow_marker.points[1].x;
	arrow_marker.points[0].y = arrow_marker.points[1].y;
	arrow_marker.points[0].z = arrow_marker.points[1].z + arrow_length_;

}

void InteractiveSurfaceServer::add_surface(const visualization_msgs::Marker& marker,
		const geometry_msgs::Pose &pose)
{
	// create marker
	std::stringstream ss;
	ss<<marker_name_<<"_" <<surface_selection_map_.size() +1;
	visualization_msgs::InteractiveMarker int_marker;
	int_marker.name = ss.str();
	int_marker.pose = pose;



	// create button control
	visualization_msgs::InteractiveMarkerControl button_control;
	button_control.interaction_mode = button_control.BUTTON;
	button_control.markers.push_back(marker);
	button_control.name = "button_" + int_marker.name;
	button_control.always_visible = true;

	// create seletected arrow marker
	visualization_msgs::Marker arrow_marker;
	create_arrow_marker(marker,arrow_marker);
	arrow_marker.color.a = 0; // unselected
	visualization_msgs::InteractiveMarkerControl selected_arrow;
	selected_arrow.interaction_mode = selected_arrow.FIXED;
	selected_arrow.markers.push_back(arrow_marker);
	selected_arrow.name = "selected_"+int_marker.name;
	selected_arrow.always_visible = true;

	// fill interactive marker
	int_marker.controls.push_back(button_control);
	int_marker.controls.push_back(selected_arrow);
	int_marker.scale = 1;
	int_marker.header.frame_id = marker.header.frame_id;
	int_marker.description = marker_description_;

	// add marker to server
	marker_server_ptr_->insert(int_marker,button_callback_);
	menu_handler_.apply(*marker_server_ptr_,int_marker.name);

	// save name
	surface_selection_map_.insert(std::make_pair(int_marker.name,false));

	// apply changes
	marker_server_ptr_->applyChanges();
}

void InteractiveSurfaceServer::add_surface(const visualization_msgs::Marker& marker)
{
	geometry_msgs::Pose pose;
	tf::poseTFToMsg(tf::Transform::getIdentity(),pose);
	add_surface(marker,pose);
}

void InteractiveSurfaceServer::add_random_surface_marker()
{
	// create pose
	geometry_msgs::Pose pose;
	double xy_step = 0.2f;
	double z_step = 0.1f;
	double pmin = -2;
	double pmax = 2;
	tfScalar x= (pmax-pmin)*(static_cast<double>(rand())/static_cast<double>(RAND_MAX)) + pmin;
	tfScalar y= (pmax-pmin)*(static_cast<double>(rand())/static_cast<double>(RAND_MAX)) + pmin;;
	tfScalar z= (pmax-pmin)*(static_cast<double>(rand())/static_cast<double>(RAND_MAX)) + pmin;;
	tf::Transform t = tf::Transform(tf::Quaternion::getIdentity(),tf::Vector3(x,y,z));
	tf::poseTFToMsg(t,pose);

	// create triangle list marker
	visualization_msgs::Marker marker;
	create_polygon_marker(marker,rand() % defaults::MAX_TRIANGLES + 1);

	add_surface(marker,pose);
}


void InteractiveSurfaceServer::create_polygon_marker(
		visualization_msgs::Marker& marker,int triangles)
{

	ROS_INFO_STREAM("Creating polygon of "<<triangles<<" triangles");

	// setting marker properties
	marker.type = marker.TRIANGLE_LIST;
	marker.scale.x = marker.scale.y = marker.scale.z = 1;
	marker.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0;
	marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z =0;
	marker.pose.orientation.w = 1;
	marker.color.r = 0.5f;
	marker.color.g = 0.5f;
	marker.color.b = 0.8f;
	marker.color.a = 0.4f;

	// defining triangle vertices
	tf::Vector3 p0(-0.05f,0,0);
	tf::Vector3 p1(0.05f,0,0);
	tf::Vector3 p2;

	// rotation transform
	tf::Transform rot(tf::Quaternion(tf::Vector3(0,0,1),M_PI/3.0f),tf::Vector3(0,0,0));

	for(int i = 0; i < triangles;i++)
	{
		// computing last triangle point
		p2 = rot*(p1-p0) + p0;

		// creating temp vector
		std::vector<tf::Vector3> points;
		points.push_back(p0);points.push_back(p1);points.push_back(p2);

		// adding points to marker
		for(unsigned int j = 0 ; j< points.size();j++)
		{
			geometry_msgs::Point p;
			p.x = points[j].getX();p.y = points[j].getY();p.z = points[j].getZ();
			marker.points.push_back(p);
		}

		// assigning new points for next triangle
		if(rand()%2 == 0)
		{
			p1 = p2;
		}
		else
		{
			p0 = p2;
		}
	}
}



} /* namespace interactive */
} /* namespace godel_surface_detection */
