#include <boost/function.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/assert.hpp>
#include <math.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_cartesian_plan_plugin/generate_cartesian_path.h>

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
  /*! The destructor resets the moveit_group_ and the kinematic_state of the robot.
  */
  moveit_group_.reset();
  kinematic_state.reset();
  robot_model_loader.reset();
  kmodel.reset();
}

void GenerateCartesianPath::init()
{
  /*! Initialize the MoveIt parameters:
        - MoveIt group
        - Kinematic State is the current kinematic congiguration of the Robot
        - Robot model which handles getting the Robot Model
        - Joint Model group which are necessary for checking if Way-Point is outside the IK Solution
        .
  */
  selected_plan_group = 0;
  robot_model_loader = RobotModelLoaderPtr(new robot_model_loader::RobotModelLoader("robot_description"));
  kmodel = robot_model_loader->getModel();
  //kmodel = robot_model::RobotModelConstPtr(new moveit::core::RobotModel(robot_model_loader->getModel()));

  //group_names = kmodel->getJointModelGroupNames();

  end_eff_joint_groups = kmodel->getEndEffectors();
  

  ROS_INFO_STREAM("size of the end effectors is: "<<end_eff_joint_groups.size());

  if (end_eff_joint_groups.empty())
  {
    group_names = kmodel->getJointModelGroupNames();
  }
  else
  {
  for(int i=0;i<end_eff_joint_groups.size();i++)
  {
    const std::pair< std::string, std::string > & parent_group_name = end_eff_joint_groups.at(i)->getEndEffectorParentGroup();
    group_names.push_back(parent_group_name.first);
  }
}

  ROS_INFO_STREAM("Group name:"<< group_names[selected_plan_group]);

  moveit_group_ = MoveGroupPtr(new move_group_interface::MoveGroup(group_names[selected_plan_group]));
  kinematic_state = moveit::core::RobotStatePtr(new robot_state::RobotState(kmodel));
  kinematic_state->setToDefaultValues();



  joint_model_group = kmodel->getJointModelGroup(group_names[selected_plan_group]);

  //ROS_INFO_STREAM("Joint model group " << end_eff_joint_groups[0]->getName());

  // if(joint_model_group->getLinkModelNames().empty())
  // {
  //   ROS_INFO_STREAM("Joint model group is empty");
  //   kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back());
  // }

}
void GenerateCartesianPath::setCartParams(double plan_time_,double cart_step_size_, double cart_jump_thresh_, bool moveit_replan_,bool avoid_collisions_)
{
  /*! Set the necessary parameters for the MoveIt and the Cartesian Path Planning.
      These parameters correspond to the ones that the user has entered or the default ones before the execution of the Cartesian Path Planner.
  */
  ROS_INFO_STREAM("MoveIt and Cartesian Path parameters from UI:\n MoveIt Plan Time:"<<plan_time_
                  <<"\n Cartesian Path Step Size:"<<cart_step_size_
                  <<"\n Jump Threshold:"<<cart_jump_thresh_
                  <<"\n Replanning:"<<moveit_replan_
                  <<"\n Avoid Collisions:"<<avoid_collisions_);

  PLAN_TIME_        = plan_time_;
  MOVEIT_REPLAN_    = moveit_replan_;
  CART_STEP_SIZE_   = cart_step_size_;
  CART_JUMP_THRESH_ = cart_jump_thresh_;
  AVOID_COLLISIONS_ = avoid_collisions_;
}

void GenerateCartesianPath::moveToPose(std::vector<geometry_msgs::Pose> waypoints)
{
    /*!

    */
    Q_EMIT cartesianPathExecuteStarted();

    moveit_group_->setPlanningTime(PLAN_TIME_);
    moveit_group_->allowReplanning (MOVEIT_REPLAN_);

    move_group_interface::MoveGroup::Plan plan;

    moveit_msgs::RobotTrajectory trajectory_;
    double fraction = moveit_group_->computeCartesianPath(waypoints,CART_STEP_SIZE_,CART_JUMP_THRESH_,trajectory_,AVOID_COLLISIONS_);
    robot_trajectory::RobotTrajectory rt(kmodel, group_names[selected_plan_group]);

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
    Q_EMIT cartesianPathCompleted(fraction);

  	moveit_group_->execute(plan);

    kinematic_state = moveit_group_->getCurrentState();

    Q_EMIT cartesianPathExecuteFinished();

}

void GenerateCartesianPath::cartesianPathHandler(std::vector<geometry_msgs::Pose> waypoints)
{
  /*! Since the execution of the Cartesian path is time consuming and can lead to locking up of the Plugin and the RViz enviroment the function for executing the Cartesian Path Plan has been placed in a separtate thread.
      This prevents the RViz and the Plugin to lock.
  */
  ROS_INFO("Starting concurrent process for Cartesian Path");
  QFuture<void> future = QtConcurrent::run(this, &GenerateCartesianPath::moveToPose, waypoints);
}



void GenerateCartesianPath::checkWayPointValidity(const geometry_msgs::Pose& waypoint,const int point_number)
{
      /*! This function is called every time the user updates the pose of the Way-Point and checks if the Way-Point is within the valid IK solution for the Robot.
          In the case when a point is outside the valid IK solution this function send a signal to the RViz enviroment to update the color of the Way-Point.
      */
       bool found_ik = kinematic_state->setFromIK(joint_model_group, waypoint, 3, 0.006);

         if(found_ik)
        {

           Q_EMIT wayPointOutOfIK(point_number,0);
        }
        else
        {
          // ROS_INFO("Did not find IK solution for waypoint %d",point_number);
          Q_EMIT wayPointOutOfIK(point_number,1);
        }
}
// //doesnt make sense to put it in concurent process, the update is not realistic
// void GenerateCartesianPath::checkWayPointValidityHandler(const geometry_msgs::Pose& waypoint,const int point_number)
// {
//     ROS_INFO("Concurrent process for the Point out of IK range");
//     QFuture<void> future = QtConcurrent::run(this, &GenerateCartesianPath::checkWayPointValidity, waypoint,point_number);

// }

void GenerateCartesianPath::initRviz_done()
{
  /*! Once the initialization of the RViz is has finished, this function sends the pose of the robot end-effector and the name of the base frame to the RViz enviroment.
      The RViz enviroment sets the User Interactive Marker pose and Add New Way-Point RQT Layout default values based on the end-effector starting position.
      The transformation frame of the InteractiveMarker is set based on the robot PoseReferenceFrame.
  */
  ROS_INFO("RViz is done now we need to emit the signal");

  if(moveit_group_->getEndEffectorLink().empty())
  {
    ROS_INFO("End effector link is empty");
    const std::vector< std::string > &  joint_names = joint_model_group->getLinkModelNames();
    for(int i=0;i<joint_names.size();i++)
    {
      ROS_INFO_STREAM("Link " << i << " name: "<< joint_names.at(i));
    }
    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform(joint_names.at(0));
    //tf::Transform end_effector;
    tf::transformEigenToTF(end_effector_state, end_effector);
    Q_EMIT getRobotModelFrame_signal(moveit_group_->getPoseReferenceFrame(),end_effector);
  }
  else
  {

    ROS_INFO("End effector link is not empty");
    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform(moveit_group_->getEndEffectorLink());
    //tf::Transform end_effector;
    tf::transformEigenToTF(end_effector_state, end_effector);

    Q_EMIT getRobotModelFrame_signal(moveit_group_->getPoseReferenceFrame(),end_effector);
  }

    Q_EMIT sendCartPlanGroup(group_names);


}
void GenerateCartesianPath::moveToHome()
{

  geometry_msgs::Pose home_pose;
  tf::poseTFToMsg(end_effector,home_pose);

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(home_pose);

  cartesianPathHandler(waypoints);

}

void GenerateCartesianPath::getSelectedGroupIndex(int index)
{
  selected_plan_group = index;

  ROS_INFO_STREAM("selected name is:"<<group_names[selected_plan_group]);
  moveit_group_.reset();
  kinematic_state.reset();
  moveit_group_ = MoveGroupPtr(new move_group_interface::MoveGroup(group_names[selected_plan_group]));

  kinematic_state = moveit::core::RobotStatePtr(new robot_state::RobotState(kmodel));
  kinematic_state->setToDefaultValues();

  joint_model_group = kmodel->getJointModelGroup(group_names[selected_plan_group]);

  if(moveit_group_->getEndEffectorLink().empty())
  {
    ROS_INFO("End effector link is empty");
    const std::vector< std::string > &  joint_names = joint_model_group->getLinkModelNames();
    for(int i=0;i<joint_names.size();i++)
    {
      ROS_INFO_STREAM("Link " << i << " name: "<< joint_names.at(i));
    }
    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform(joint_names.at(0));
    //tf::Transform end_effector;
    tf::transformEigenToTF(end_effector_state, end_effector);
    Q_EMIT getRobotModelFrame_signal(moveit_group_->getPoseReferenceFrame(),end_effector);
  }
  else
  {

    ROS_INFO("End effector link is not empty");
    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform(moveit_group_->getEndEffectorLink());
    //tf::Transform end_effector;
    tf::transformEigenToTF(end_effector_state, end_effector);

    Q_EMIT getRobotModelFrame_signal(moveit_group_->getPoseReferenceFrame(),end_effector);
  }

}
