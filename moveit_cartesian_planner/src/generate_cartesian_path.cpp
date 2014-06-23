#include <boost/assign/list_of.hpp>
#include <boost/assert.hpp>
#include <math.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_cartesian_planner/generate_cartesian_path.h>

const double GenerateCartesianPath::PLANNING_TIME = 60.0f;
const double GenerateCartesianPath::WAIT_MSG_DURATION = 5.0f;
const double GenerateCartesianPath::MIN_TRAJECTORY_TIME_STEP = 0.8f; // seconds
const double GenerateCartesianPath::EEF_STEP = 0.04f; // 5cm
const double GenerateCartesianPath::MIN_JOINT_VELOCITY = 0.01f ; // rad/sect


GenerateCartesianPath::GenerateCartesianPath(QObject *parent)
{
    //to add initializations
}
GenerateCartesianPath::~GenerateCartesianPath()
{
   //to add destructor
   //clear points for example

}

void GenerateCartesianPath::init()
{

   //to add initializations
}

void GenerateCartesianPath::move_to_pose(std::vector<geometry_msgs::Pose> waypoints)
{
  //not necessary!!!
  // ros::AsyncSpinner spinner(1);
  // spinner.start();

  move_group_interface::MoveGroup group("manipulator");
  group.setNamedTarget("straight up");
  move_group_interface::MoveGroup::Plan plan;

    moveit_msgs::RobotTrajectory trajectory_;
    group.setPlanningTime(10.0);
    double fraction = group.computeCartesianPath(waypoints,0.01,0.0,trajectory_,false);
    robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "manipulator");
    rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory_);

    // Thrid create a IterativeParabolicTimeParameterization object
  	trajectory_processing::IterativeParabolicTimeParameterization iptp;
  	bool success = iptp.computeTimeStamps(rt);
  	ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

  	// Get RobotTrajectory_msg from RobotTrajectory
  	rt.getRobotTrajectoryMsg(trajectory_);
  	// Finally plan and execute the trajectory
	plan.trajectory_ = trajectory_;
	ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",fraction * 100.0);    
	sleep(5.0);
	group.execute(plan);
 
  sleep(1.0);
  group.stop();
  sleep(1.0);  // wait for stop command to be received
}