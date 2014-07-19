// MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>


#include <pluginlib/class_loader.h>

#include <QObject>


#ifndef GENERATE_CARTESIAN_PATH_H_
#define GENERATE_CARTESIAN_PATH_H_

class GenerateCartesianPath: public QObject
{
Q_OBJECT

public:

public:
	GenerateCartesianPath(QObject* parent = 0);
	virtual ~GenerateCartesianPath();
	void init();
public Q_SLOTS:
	void moveToPose(std::vector<geometry_msgs::Pose> waypoints);
	void checkWayPointValidity(const geometry_msgs::Pose& waypoints,const int marker_name);
	void initRviz_done();
Q_SIGNALS:
	void wayPointOutOfIK(int point_number, int out_of_range); 
	void getRobotModelFrame_signal(const std::string robot_model_frame,const tf::Transform end_effector);
protected:
	robot_model_loader::RobotModelLoader robot_model_loader;
	moveit::core::RobotModelPtr kinematic_model;
	moveit::core::RobotStatePtr kinematic_state;
	const moveit::core::JointModelGroup* joint_model_group;
};

#endif // GENERATE_CARTESIAN_PATH_H_
