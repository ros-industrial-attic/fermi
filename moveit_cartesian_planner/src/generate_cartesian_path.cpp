#include <boost/function.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/assert.hpp>
#include <math.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_cartesian_planner/generate_cartesian_path.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <ros/ros.h>
// #include <tf/transform_listener.h>
// #include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseArray.h>


GenerateCartesianPath::GenerateCartesianPath(QObject *parent)
{
    //to add initializations
    init();


}
GenerateCartesianPath::~GenerateCartesianPath()
{
   //to add destructor
   //clear points for example

}

void GenerateCartesianPath::init()
{
  robot_model_loader = robot_model_loader::RobotModelLoader("robot_description"); 
  kinematic_model = moveit::core::RobotModelPtr( robot_model_loader.getModel());
  std::string robot_modelFrame;
  robot_modelFrame.assign(kinematic_model->getModelFrame().c_str());
  ROS_INFO_STREAM("Model frame: " << kinematic_model->getModelFrame().c_str());

  kinematic_state = moveit::core::RobotStatePtr(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues(); 
  joint_model_group = kinematic_model->getJointModelGroup("manipulator");

}

void GenerateCartesianPath::moveToPose(std::vector<geometry_msgs::Pose> waypoints)
{


  move_group_interface::MoveGroup group("manipulator");
  move_group_interface::MoveGroup::Plan plan;

    moveit_msgs::RobotTrajectory trajectory_;
    group.setPlanningTime(10.0);//user selectable??? why not!!!
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
    group.allowReplanning (true);
  	group.execute(plan);
}

void GenerateCartesianPath::checkWayPointValidity(const geometry_msgs::Pose& waypoint,const int point_number)
{

       bool found_ik = kinematic_state->setFromIK(joint_model_group, waypoint, 1, 0.001);

         if(found_ik)
        {

           Q_EMIT wayPointOutOfIK(point_number,0); 
        }
        else
        {
          ROS_INFO("Did not find IK solution for waypoint %d",point_number);
          Q_EMIT wayPointOutOfIK(point_number,1); 
        }
}

void GenerateCartesianPath::initRviz_done()
{
  ROS_INFO("RViz is done now we need to emit the signal");
  Q_EMIT getRobotModelFrame_signal(kinematic_model->getModelFrame().c_str());
}