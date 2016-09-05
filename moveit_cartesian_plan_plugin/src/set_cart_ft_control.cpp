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
  /*! The destructor for the cartesian Force Control. Currently empty
  */
}

void SetCartesianFTControl::init()
{
  /*! Initialize Cartesian Force Control publisher
        .
  */
    cartesian_ft_params_pub = n_.advertise<cartesian_impedance_msgs::SetCartesianForceCtrl>("/set_cartesian_ft_params", 1000);
}

void SetCartesianFTControl::sendCartFTParams(cartesian_impedance_msgs::SetCartesianForceCtrlPtr cart_ft_params)
{
  /*! Publish the Cartesian Force Control Parameters set from the UI
        .
  */
  cartesian_ft_params_pub.publish(cart_ft_params);

}
