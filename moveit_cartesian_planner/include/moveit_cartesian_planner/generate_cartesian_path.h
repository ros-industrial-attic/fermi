#include <ros/ros.h>
#include <boost/function.hpp>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseArray.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <pluginlib/class_loader.h>

#include <QObject>


#ifndef GENERATE_CARTESIAN_PATH_H_
#define GENERATE_CARTESIAN_PATH_H_

class GenerateCartesianPath: public QObject
{
Q_OBJECT
public:
	static const double PLANNING_TIME;
	static const double WAIT_MSG_DURATION;
	static const double EEF_STEP;
	static const double MIN_TRAJECTORY_TIME_STEP;
	static const double MIN_JOINT_VELOCITY;
public:
	GenerateCartesianPath(QObject* parent = 0);
	virtual ~GenerateCartesianPath();
	void init();
public Q_SLOTS:
	void move_to_pose(std::vector<geometry_msgs::Pose> waypoints);
protected:

protected:

public:

};

#endif // GENERATE_CARTESIAN_PATH_H_
