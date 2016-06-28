#include <boost/function.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/assert.hpp>
#include <math.h>
#include <moveit_cartesian_plan_plugin/set_cart_ft_control.hpp>


SetCartesianFTControl::SetCartesianFTControl(QObject *parent)
{
    //to add initializations
    init();

}
SetCartesianFTControl::~SetCartesianFTControl()
{
  /*! The destructor resets the moveit_group_ and the kinematic_state of the robot.
  */
}

void SetCartesianFTControl::init()
{
  /*! Initialize the MoveIt parameters:
        - MoveIt group
        - Kinematic State is the current kinematic congiguration of the Robot
        - Robot model which handles getting the Robot Model
        - Joint Model group which are necessary for checking if Way-Point is outside the IK Solution
        .
  */
    cartesian_ft_params_pub = n_.advertise<cartesian_impedance_msgs::SetCartesianForceCtrl>("/set_cartesian_ft_params", 1000);
}

void SetCartesianFTControl::sendCartFTParams(cartesian_impedance_msgs::SetCartesianForceCtrlPtr cart_ft_params)
{
  ROS_INFO("Publishing FT Ctrl Set %s",cart_ft_params->DOF.c_str());
  cartesian_ft_params_pub.publish(cart_ft_params);

}
