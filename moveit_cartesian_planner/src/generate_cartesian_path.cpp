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
	// group_name_ = "manipulator";
	// home_position_ = "home";
	// world_frame_ = "base_link";
	// tcp_frame_ = "link_6";
}
GenerateCartesianPath::~GenerateCartesianPath()
{}

void GenerateCartesianPath::init()
{

	// move_group_ptr_ = MoveGroupPtr(new move_group_interface::MoveGroup(group_name_));
	// move_group_ptr_->setEndEffectorLink(tcp_frame_);
	// move_group_ptr_->setPoseReferenceFrame(world_frame_);
	//move_group_ptr_->setPlanningTime(PLANNING_TIME);
	//tf_listener_ptr_ = TransformListenerPtr(new tf::TransformListener());
}

void GenerateCartesianPath::move_to_pose(std::vector<geometry_msgs::Pose> waypoints)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();
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

// void GenerateCartesianPath::apply_simple_trajectory_filter(	moveit_msgs::RobotTrajectory& traj)
// {
// 	std::vector<trajectory_msgs::JointTrajectoryPoint> &current_points = traj.joint_trajectory.points;
// 	std::vector<trajectory_msgs::JointTrajectoryPoint> points;
// 	double dt;

// 	// setting first point timestamp to zero
// 	current_points.front().time_from_start = ros::Duration(0);

// 	// removing redundant joint points
// 	points.push_back(current_points.front());
// 	trajectory_msgs::JointTrajectoryPoint last_point = current_points.front();
// 	for(unsigned int i = 1;i < current_points.size(); i++)
// 	{
// 		trajectory_msgs::JointTrajectoryPoint &next_point = current_points[i];

// 		// time elapsed between p1 and p2
// 		dt = (next_point.time_from_start - last_point.time_from_start).toSec();
// 		if(dt<MIN_TRAJECTORY_TIME_STEP)
// 		{
// 			dt = MIN_TRAJECTORY_TIME_STEP;
// 			next_point.time_from_start = last_point.time_from_start + ros::Duration(dt);
// 		}

// 		// computing velocity for each joint
// 		int zero_vel_counter = 0;
// 		next_point.velocities.resize(next_point.positions.size());
// 		for(unsigned int j = 0; j < next_point.positions.size(); j++)
// 		{
// 			next_point.velocities[j] = (next_point.positions[j] - last_point.positions[j])/dt;
// 			if(next_point.velocities[j] < MIN_JOINT_VELOCITY)
// 			{
// 				zero_vel_counter++;
// 			}
// 		}


// 		if(zero_vel_counter < next_point.positions.size())
// 		{

// 			points.push_back(next_point);
// 			last_point = next_point;
// 		}
// 	}

// 	// filling first and last points with 0 velocities
// 	points.front().velocities.assign(traj.joint_trajectory.joint_names.size(),0);
// 	points.back().velocities.assign(traj.joint_trajectory.joint_names.size(),0);

// 	// checking time stamp for last joint point
// 	trajectory_msgs::JointTrajectoryPoint &p_last = points.back();
// 	trajectory_msgs::JointTrajectoryPoint &p_before_last = *(points.end()-2);
// 	if((p_last.time_from_start - p_before_last.time_from_start).toSec() < MIN_TRAJECTORY_TIME_STEP)
// 	{
// 		p_last.time_from_start = p_before_last.time_from_start + ros::Duration(MIN_TRAJECTORY_TIME_STEP);
// 	}

// 	traj.joint_trajectory.points.assign(points.begin(),points.end());

// 	ROS_INFO_STREAM("Filtered trajectory: "<<traj);
// }