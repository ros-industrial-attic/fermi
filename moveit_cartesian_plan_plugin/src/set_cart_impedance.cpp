#include <boost/function.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/assert.hpp>
#include <math.h>
#include <moveit_cartesian_plan_plugin/set_cart_impedance.hpp>


SetCartesianImpedance::SetCartesianImpedance(QObject *parent)
{
    //to add initializations
    init();

}
SetCartesianImpedance::~SetCartesianImpedance()
{
  /*! The destructor for the cartesian Impedance Control. Currently empty...
  */
}

void SetCartesianImpedance::init()
{
  /*! Initialize Cartesian Impedance Control publisher
        .
  */
    cartesian_impedance_params_pub = n_.advertise<cartesian_impedance_msgs::SetCartesianImpedance>("/set_cartesian_impedance_params", 1000);
}

void SetCartesianImpedance::sendCartImpedanceParams(cartesian_impedance_msgs::SetCartesianImpedancePtr cart_impedance_params)
{
  /*! Publish the Cartesian Impedance Control Parameters that are set from the UI
        .
  */
  cartesian_impedance_params_pub.publish(cart_impedance_params);

}
