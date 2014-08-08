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
#include <rviz/default_plugin/interactive_markers/interactive_marker.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Scalar.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/ColorRGBA.h>
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
#include <QTimer>
#include <QtConcurrentRun>
#include <QFuture>

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
	virtual void makeInteractiveMarker();

	virtual void msgCallback(const boost::shared_ptr<const geometry_msgs::PointStamped>& point_ptr);

	// //function for returning the count of the current points in the rviz enviroment!!
	// virtual int getCount();

private:
	//function for creating a way-point marker
	Marker makeWayPoint( InteractiveMarker &msg );
	//function to create the InteractionArrow Marker
	Marker makeInterArrow( InteractiveMarker &msg );
	//create controls for each different marker. Here we have control for the defaulot starting control ArrowMarkers(the cartesian way points)
	InteractiveMarkerControl& makeArrowControlDefault(InteractiveMarker &msg );

	InteractiveMarkerControl& makeArrowControlDetails(InteractiveMarker &msg );

	//the box control can be used as a pointer to a certain 3D location and when clicked it will add a arrow to that location
	InteractiveMarkerControl& makeInteractiveMarkerControl( InteractiveMarker &msg_box );

	virtual void changeMarkerControlAndPose(std::string marker_name,bool set_control);


    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
	interactive_markers::MenuHandler menu_handler;

	//InteractiveMarker inter_arrow_marker_;

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
	virtual void pointDeleted(std::string marker_name); 
	void addPointFromUI( const tf::Transform point_pos);
	void pointPoseUpdated(const tf::Transform& point_pos, const char* marker_name);
	void parseWayPoints();
	void saveWayPointsToFile();
	void clearAllPointsRViz();
	void wayPointOutOfIK_slot(int point_number,int out);
	void getRobotModelFrame_slot(const std::string robot_model_frame,const tf::Transform end_effector);

Q_SIGNALS:
	void initRviz();
	void pointDeleteRviz(int marker_name_nr); 
	void addPointRViz(const tf::Transform& point_pos, const int count);
	void pointPoseUpdatedRViz(const tf::Transform& point_pos, const char* marker_name);
	void wayPoints_signal(std::vector<geometry_msgs::Pose> waypoints);
	void onUpdatePosCheckIkValidity(const geometry_msgs::Pose& waypoint,const int point_number);


protected:
    QWidget *widget_;
    QObject *path_generate;
private:
	//define constants for color, arrow size, etc.
	std_msgs::ColorRGBA WAY_POINT_COLOR;
	std_msgs::ColorRGBA WAY_POINT_COLOR_OUTSIDE_IK;
	std_msgs::ColorRGBA ARROW_INTER_COLOR;

	geometry_msgs::Vector3 WAY_POINT_SCALE_CONTROL;
	geometry_msgs::Vector3 ARROW_INTER_SCALE_CONTROL;

	float INTERACTIVE_MARKER_SCALE;
	float ARROW_INTERACTIVE_SCALE;
};
} //end of namespace moveit_cartesian_planner

#endif //add_way_point_H_


