#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <cartesian_impedance_msgs/SetCartesianForceCtrl.h>

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>


#include <pluginlib/class_loader.h>
#include <std_msgs/String.h>

#include <QObject>
#include <QTimer>
#include <QtConcurrentRun>
#include <QFuture>


#ifndef SET_CARTESIAN_FT_CONTROL_H_
#define SET_CARTESIAN_FT_CONTROL_H_

/*!
 *  \brief     Class for setting up the Cartesian Force msgs and UI parameters.
 *  \details   The SetCartesianFTControl Class handles all the interactions with the Qt UI.
 	 		   This Class inherits from the QObject superclass.
 	 		   The concept of this class is to initialize all the necessary parameters for setting Cartesian Force parameters set from the UI..
 	 		   It gets the necessary parameters for Cartesian Force and sends them via particular topic to the robot driver.
 *  \author    Risto Kojcev
 */

class SetCartesianFTControl: public QObject
{
Q_OBJECT

public:
	//! Constructor for the Cartesian Force Control Parameters.
	SetCartesianFTControl(QObject* parent = 0);
	//! Virtual Destructor for the Cartesian Force Control Parameters.
	virtual ~SetCartesianFTControl();
	//! Initialization of the necessary Cartesian Force Control Parameters.
	void init();
public Q_SLOTS:
	//! Get the Way-Points from the RViz enviroment and use them to generate Cartesian Path.
  void sendCartFTParams(cartesian_impedance_msgs::SetCartesianForceCtrlPtr cart_ft_params);

Q_SIGNALS:

private:
	ros::NodeHandle n_;
	ros::Publisher cartesian_ft_params_pub;

};

#endif // GENERATE_CARTESIAN_PATH_H_
