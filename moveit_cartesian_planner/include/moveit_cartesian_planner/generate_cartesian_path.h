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

#include <QWidget>
#include <QObject>


#ifndef GENERATE_CARTESIAN_PATH_H_
#define GENERATE_CARTESIAN_PATH_H_

typedef boost::shared_ptr<move_group_interface::MoveGroup> MoveGroupPtr;
typedef boost::shared_ptr<tf::TransformListener> TransformListenerPtr;

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
	//static void apply_simple_trajectory_filter(moveit_msgs::RobotTrajectory& trajectory);
	//static void apply_trajectory_parabolic_time_parameterization(robot_trajectory::RobotTrajectory& rt,
	//		moveit_msgs::RobotTrajectory &traj,unsigned int max_iterations=200,double max_time_change_per_it=.6);
protected:
		// generates circular trajectory above target object
	//bool create_scan_trajectory(std::vector<geometry_msgs::Pose> &scan_poses,moveit_msgs::RobotTrajectory& scan_traj);
protected:
    // moveit
	MoveGroupPtr move_group_ptr_;
	TransformListenerPtr tf_listener_ptr_;
	std::vector<geometry_msgs::Pose> scan_traj_poses_;
public:
	//remove this later its bad to have global variables this is just for initial test!!!
	// std::string group_name_;
	// std::string world_frame_;
	// std::string tcp_frame_;
	// std::string home_position_;
	// int num_scan_points_;
	// double reachable_scan_points_ratio_;
	// std::string scan_topic_;
	// bool stop_on_planning_error_;
};

#endif // GENERATE_CARTESIAN_PATH_H_
