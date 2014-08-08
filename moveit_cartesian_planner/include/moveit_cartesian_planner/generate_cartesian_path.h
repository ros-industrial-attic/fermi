// MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>


#include <pluginlib/class_loader.h>

#include <QObject>
#include <QTimer>
#include <QtConcurrentRun>
#include <QFuture>


#ifndef GENERATE_CARTESIAN_PATH_H_
#define GENERATE_CARTESIAN_PATH_H_

typedef boost::shared_ptr<move_group_interface::MoveGroup> MoveGroupPtr;

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
	void checkWayPointValidityHandler(const geometry_msgs::Pose& waypoint,const int point_number);
	void initRviz_done();
	void cartesianPathHandler(std::vector<geometry_msgs::Pose> waypoints);
	void setCartParams(double plan_time_,double cart_step_size_, double cart_jump_thresh_, bool moveit_replan_,bool avoid_collisions_);
Q_SIGNALS:
	void wayPointOutOfIK(int point_number, int out_of_range); 
	void getRobotModelFrame_signal(const std::string robot_model_frame,const tf::Transform end_effector);
	void cartesianPathExecuteStarted();
	void cartesianPathExecuteFinished();
protected:

	moveit::core::RobotStatePtr kinematic_state;
	const moveit::core::JointModelGroup* joint_model_group;

    MoveGroupPtr moveit_group_;
// MoveIt and Cartesian path parameters set by the user from the QT UI
   double PLAN_TIME_;
   double CART_STEP_SIZE_;
   double CART_JUMP_THRESH_;
   bool MOVEIT_REPLAN_;
   bool AVOID_COLLISIONS_;

};

#endif // GENERATE_CARTESIAN_PATH_H_
