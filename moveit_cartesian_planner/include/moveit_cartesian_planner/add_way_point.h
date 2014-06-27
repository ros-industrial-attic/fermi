#ifndef add_way_point_H_
#define add_way_point_H_

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include <algorithm>
#include <vector>
#include <iterator>

#include <rviz/panel.h>
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Scalar.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>


#include <rviz/properties/bool_property.h>
#include <rviz/properties/string_property.h>
#include <moveit_cartesian_planner/widgets/path_planning_widget.h>

#include <moveit_cartesian_planner/generate_cartesian_path.h>

#include <QWidget>
#include <QCursor>
#include <QObject>
#include <QKeyEvent>
#include <QHBoxLayout>

#endif


namespace rviz
{
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;

class StringProperty;
class BoolProperty;
}

using namespace visualization_msgs;
namespace moveit_cartesian_planner
{
class AddWayPoint: public rviz::Panel
{
Q_OBJECT
public:
	AddWayPoint(QWidget* parent = 0); 
	virtual ~AddWayPoint();

	virtual void onInitialize();

	//fucntion for all the interactive marker interactions
	virtual void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

	virtual void makeArrow(const tf::Transform& point_pos,int count_arrow);//
	virtual void makeBox();

	virtual void msgCallback(const boost::shared_ptr<const geometry_msgs::PointStamped>& point_ptr);

	//function for returning the count of the current points in the rviz enviroment!!
	virtual int getCount();

private:
	//create controls for each different marker. Here we have control for the defaulot starting control ArrowMarkers(the cartesian way points)
	InteractiveMarkerControl& makeArrowControl_default(InteractiveMarker &msg );

	InteractiveMarkerControl& makeArrowControl_details(InteractiveMarker &msg );

	//the box control can be used as a pointer to a certain 3D location and when clicked it will add a arrow to that location
	InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg_box );

	virtual void changeMarkerControlAndPose(std::string marker_name,bool set_control);


	boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
	interactive_markers::MenuHandler menu_handler;

	std::vector<tf::Transform> waypoints_pos;
	tf::Transform box_pos;

	int count;

    message_filters::Subscriber<geometry_msgs::PointStamped> point_sub_;
    tf::TransformListener tf_;
    tf::MessageFilter<geometry_msgs::PointStamped> * tf_filter_;
    ros::NodeHandle n_;
    std::string target_frame_;
    

protected Q_SLOTS:
	// rviz::Panel virtual functions
	virtual void load(const rviz::Config& config);
	virtual void save(rviz::Config config) const;
public Q_SLOTS:
	virtual void point_deleted(std::string marker_name); 
	void addPointFromUI( const tf::Transform point_pos);
	void point_pose_updated(const tf::Transform& point_pos, const char* marker_name);
	void parse_waypoints();
	void saveWayPointsToFile();
	void clearAllPoints_RViz();
Q_SIGNALS:
	void initRviz();
	void point_deleted_from_Rviz(int marker_name_nr); 
	void addPointFrom_RViz(const tf::Transform& point_pos, const int count);
	void point_pose_updated_RViz(const tf::Transform& point_pos, const char* marker_name);
	void cartesian_waypoints(const std::vector<tf::Transform > point_pos);
	void way_points_signal(std::vector<geometry_msgs::Pose> waypoints);


protected:
    QWidget *widget_;
    QObject *path_generate;

};
} //end of namespace moveit_cartesian_planner

#endif //add_way_point_H_


