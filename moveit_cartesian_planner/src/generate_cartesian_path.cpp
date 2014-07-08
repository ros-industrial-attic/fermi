#include <boost/assign/list_of.hpp>
#include <boost/assert.hpp>
#include <math.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_cartesian_planner/generate_cartesian_path.h>

// const double GenerateCartesianPath::PLANNING_TIME = 60.0f;
// const double GenerateCartesianPath::WAIT_MSG_DURATION = 5.0f;
// const double GenerateCartesianPath::MIN_TRAJECTORY_TIME_STEP = 0.8f; // seconds
// const double GenerateCartesianPath::EEF_STEP = 0.04f; // 5cm
// const double GenerateCartesianPath::MIN_JOINT_VELOCITY = 0.01f ; // rad/sect


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
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  kinematic_state = moveit::core::RobotStatePtr(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues(); 
  joint_model_group = kinematic_model->getJointModelGroup("manipulator");
}

void GenerateCartesianPath::move_to_pose(std::vector<geometry_msgs::Pose> waypoints)
{
  // not necessary!!!
  // ros::AsyncSpinner spinner(1);
  // spinner.start();

  move_group_interface::MoveGroup group("manipulator");
  //group.setNamedTarget("straight up");
  move_group_interface::MoveGroup::Plan plan;

    moveit_msgs::RobotTrajectory trajectory_;
    group.setPlanningTime(30.0);//user selectable??? why not!!!
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
  // geometry_msgs::Pose waypoint_gmsg;
  // tf::poseTFToMsg(waypoint,waypoint_gmsg);

  // if(strcmp(marker_name,"add_point_button")!=0)
  // {
      //int point_number = atoi(marker_name);

      // ROS_INFO("Model frame in the update out of reach function: %s", kinematic_model->getModelFrame().c_str());
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
//   }
// else
// {
//   ROS_INFO("I dont check for the interaction marker");
// }

}