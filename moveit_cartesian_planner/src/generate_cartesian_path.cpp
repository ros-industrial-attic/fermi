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
#include <geometry_msgs/PoseArray.h>


GenerateCartesianPath::GenerateCartesianPath(QObject *parent)
{
    //to add initializations
    init();


}
GenerateCartesianPath::~GenerateCartesianPath()
{
  moveit_group_.reset();
  kinematic_state.reset();
}

void GenerateCartesianPath::init()
{

  //kinematic_model = moveit::core::RobotModelPtr( robot_model_loader::RobotModelLoader("robot_description").getModel());
  //robot_model_loader = robot_model_loader::RobotModelLoader("robot_description"); 
  moveit_group_.reset(new move_group_interface::MoveGroup("manipulator"));

  moveit_group_->setPlanningTime(10.0);//user selectable??? why not!!
  moveit_group_->allowReplanning (true);
  //kinematic_model = moveit::core::RobotModelPtr( moveit_group_->getCurrentState()->getRobotModel());

  //ROS_INFO_STREAM("Model frame: " << kinematic_model->getModelFrame().c_str());

  kinematic_state = moveit::core::RobotStatePtr(moveit_group_->getCurrentState());
  kinematic_state->setToDefaultValues(); 
  

  const robot_model::RobotModelConstPtr &kmodel = kinematic_state->getRobotModel();
  joint_model_group = kmodel->getJointModelGroup("manipulator");


}

void GenerateCartesianPath::moveToPose(std::vector<geometry_msgs::Pose> waypoints)
{
    // move_group_interface::MoveGroup group("manipulator");
    move_group_interface::MoveGroup::Plan plan;
    //moveit_group_->setStartState(*kinematic_state);

    // //we need to change all the positions and orientations vectors to geometry_msgs::Pose, in the next days work on this
  // for(int i=0;i<waypoints.size();i++)
  // {

  //    ROS_INFO_STREAM( "In planner positions:"<<waypoints[i].position.x<<";"<< waypoints[i].position.y<<"; " << waypoints[i].position.z);
  //    ROS_INFO_STREAM( "In planner orientations:"<<waypoints[i].orientation.x<<";"<<waypoints[i].orientation.y<<";"<<waypoints[i].orientation.z<<";"<<waypoints[i].orientation.w);
  // }

    moveit_msgs::RobotTrajectory trajectory_;
    double fraction = moveit_group_->computeCartesianPath(waypoints,0.01,0.0,trajectory_,false);
    robot_trajectory::RobotTrajectory rt(kinematic_state->getRobotModel(), "manipulator");

    rt.setRobotTrajectoryMsg(*kinematic_state, trajectory_);

    ROS_INFO_STREAM("Pose reference frame: " << moveit_group_->getPoseReferenceFrame ());

    // Thrid create a IterativeParabolicTimeParameterization object
  	trajectory_processing::IterativeParabolicTimeParameterization iptp;
  	bool success = iptp.computeTimeStamps(rt);
  	ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");


    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory_);
    // Finally plan and execute the trajectory
  	plan.trajectory_ = trajectory_;
  	ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",fraction * 100.0);  
 

  	moveit_group_->execute(plan);

    kinematic_state = moveit_group_->getCurrentState();
}

void GenerateCartesianPath::checkWayPointValidity(const geometry_msgs::Pose& waypoint,const int point_number)
{

       bool found_ik = kinematic_state->setFromIK(joint_model_group, waypoint, 2, 0.005);

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

  //const std::vector< std::string > robot_link_names  = kinematic_model->getLinkModelNames();
  //const int nr_dofs = kinematic_state->getVariableCount();

  // std::vector<double> joint_values;
  // kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

  const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform(moveit_group_->getEndEffectorLink());
  tf::Transform end_effector;
  tf::transformEigenToTF(end_effector_state, end_effector);

  Q_EMIT getRobotModelFrame_signal(moveit_group_->getPoseReferenceFrame(),end_effector);
}