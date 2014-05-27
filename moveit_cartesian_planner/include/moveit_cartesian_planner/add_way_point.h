#ifndef add_way_point__H
#define add_way_point__H

#include <rviz/tool.h>

//#include <rqt_gui_cpp/plugin.h>
#include <QWidget>
#include <stdio.h>
#include <iostream>
#include <string.h>

#include <rviz/panel.h>
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <math.h>
#include <tf/LinearMath/Vector3.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <ros/ros.h>

//#include <rqt_gui_cpp/plugin.h>
//#include <../src/path_planner.ui>

/*namespace Ogre
{
class SceneNode;
class Vector3;
}*/

namespace rviz
{
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
}

using namespace visualization_msgs;
namespace moveit_cartesian_planner
{
	//namespace visualization_msgs{
class AddWayPoint: public rviz::Tool
{
Q_OBJECT
public:
	AddWayPoint(QWidget* parent = 0);
	~AddWayPoint();

	virtual void onInitialize();

	virtual void activate();
	virtual void deactivate();

	virtual int processMouseEvents(rviz::ViewportMouseEvent& event);
	//create controls for each different marker. Here we have control for the SphereMarkers(the cartesian way points)
	//only to move in certain direction
	InteractiveMarkerControl& makeSphereControl(InteractiveMarker &msg );

	//the box control can be used as a pointer to a certain 3D location and when double clicked it will add a sphere to that location
	InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg_box );

	//fucntion for all the interactive marker interactions
	virtual void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

	virtual void load(const rviz::Config& config);
	virtual void save(rviz::Config config) const;

	void makeSphere(const tf::Vector3& point_pos,int count);
	void makeBox();

//public Q_SLOTS:
//	setSpherePosition(std::vector<tf::Vector3 > positions);
private:
	//std::vector<Ogre::SceneNode*> sphere_nodes_;
	//Ogre::SceneNode* moving_sphere_node_;
	std::string sphere_resource_;
	rviz::VectorProperty* current_sphere_property_;
	//boost::shared_ptr<rviz::Arrow> path_sphere_;

	boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
	std::vector<tf::Vector3 > positions;
	int count;
	//std::vector<tf::Vector3 > point_new_pos;
	//std::vector<Ogre::Vector3 > position;
/*
	Ui::PlannerWidget ui_;
    QWidget* widget_;*/

};
//} //end of namcespace for visualization messages
} //end of namespace moveit_cartesian_planner

#endif //PLANT_FLAG_TOOL_H


