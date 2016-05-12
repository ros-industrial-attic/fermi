// MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>


#include <pluginlib/class_loader.h>
#include <std_msgs/String.h>

#include <QObject>
#include <QTimer>
#include <QtConcurrentRun>
#include <QFuture>


#ifndef GENERATE_CARTESIAN_PATH_H_
#define GENERATE_CARTESIAN_PATH_H_

typedef boost::shared_ptr<move_group_interface::MoveGroup> MoveGroupPtr;
typedef boost::shared_ptr<robot_model_loader::RobotModelLoader> RobotModelLoaderPtr;

/*!
 *  \brief     Class for setting up the MoveIt enviroment.
 *  \details   The GenerateCartesianPath Class handles all the interactions with the MoveIt enviroment.
 	 		   This Class inherits from the QObject superclass.
 	 		   The concept of this class is to initialize all the necessary parameters for generating Cartesian Path from Way-Points.
 	 		   It gets the necessary Way-Points and handles them in the Cartesian Path Planner of the MoveIt. Furthermore it checks if a Way-Point is inside the IK solution for the loaded robot and changes it state correspondingly.
 *  \author    Risto Kojcev
 */

class GenerateCartesianPath: public QObject
{
Q_OBJECT

public:

public:
	//! Constructor for the MoveIt Cartesian Path Planner.
	GenerateCartesianPath(QObject* parent = 0);
	//! Virtual Destructor for the cartesian Path planner.
	virtual ~GenerateCartesianPath();
	//! Initialization of the necessary MoveIt parameters.
	void init();
	//get the current planning group name
	void getPlanGroupName(const std_msgs::StringConstPtr& msg);
public Q_SLOTS:
	//! Get the Way-Points from the RViz enviroment and use them to generate Cartesian Path.
  void moveToPose(std::vector<geometry_msgs::Pose> waypoints);
  //! Checks if the Way-Point is in the valid IK solution for the Robot.
	void checkWayPointValidity(const geometry_msgs::Pose& waypoints,const int marker_name);
	//! Slot for letting the Cartesian Path planning class that the RViz has finished with its initialization.
	void initRvizDone();
	//! Function for setting time consuming Cartesian Path Execution function to a separate thread.
	void cartesianPathHandler(std::vector<geometry_msgs::Pose> waypoints);
	//! Get the User entered MoveIt and Cartesian Path parameters and pass them to the corresponding private variables.
	void setCartParams(double plan_time_,double cart_step_size_, double cart_jump_thresh_, bool moveit_replan_,bool avoid_collisions_);

	void getSelectedGroupIndex(int index);

	//! Move to starting position of the robot. As loaded by default
	void moveToHome();
Q_SIGNALS:
	//! Let the RViz that a Way-Point is outside the IK solution.
	void wayPointOutOfIK(int point_number, int out_of_range);
	//! Send the pose of the currently loaded Robot Frame to the AddWayPoint and PathPlanningWidget classes.
	void getRobotModelFrame_signal(const std::string robot_model_frame,const tf::Transform end_effector);
	//! Let the RQT Widget know that a Cartesian Path Execution has started.
	void cartesianPathExecuteStarted();
	//! Let the RQT Widget know that Cartesian Path Execution has finished.
	void cartesianPathExecuteFinished();
	//! Send the percantage of successful completion of the Cartesian Path.
	void cartesianPathCompleted(double fraction);
	//! Send the planning groups to the GUI
	void sendCartPlanGroup(std::vector< std::string > group_names);
protected:
    //! MoveIt protected variables.
	moveit::core::RobotStatePtr kinematic_state_;
	const moveit::core::JointModelGroup* joint_model_group_;
  	MoveGroupPtr moveit_group_;
	robot_model::RobotModelConstPtr kmodel_;
	RobotModelLoaderPtr robot_model_loader;

	tf::Transform end_effector;


	std::vector< std::string > group_names;
	int selected_plan_group;
	std::string target_frame_;
	std::vector< const moveit::core::JointModelGroup * >  end_eff_joint_groups;

    //! MoveIt and Cartesian path parameters set by the user from the QT UI
    //! Parameter for setting the planning time of the MoveIt.
    double PLAN_TIME_;
    //! Parameter for setting the Cartesian Path step size.
    double CART_STEP_SIZE_;
    //! Parameter for setting the Jump Threshold of the Cartesian Path.
    double CART_JUMP_THRESH_;
    //! Allow MoveIt to replan.
    bool MOVEIT_REPLAN_;
    //! Generate Cartesian Path that avoids collisions.
    bool AVOID_COLLISIONS_;

};

#endif // GENERATE_CARTESIAN_PATH_H_