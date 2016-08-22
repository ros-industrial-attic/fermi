#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <cartesian_impedance_msgs/SetCartesianImpedance.h>

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>


#include <pluginlib/class_loader.h>
#include <std_msgs/String.h>

#include <QObject>
#include <QTimer>
#include <QtConcurrentRun>
#include <QFuture>


#ifndef SET_CARTESIAN_IMPEDANCE_H_
#define SET_CARTESIAN_IMPEDANCE_H_

/*!
 *  \brief     Class for setting up the MoveIt enviroment.
 *  \details   The GenerateCartesianPath Class handles all the interactions with the MoveIt enviroment.
 	 		   This Class inherits from the QObject superclass.
 	 		   The concept of this class is to initialize all the necessary parameters for generating Cartesian Path from Way-Points.
 	 		   It gets the necessary Way-Points and handles them in the Cartesian Path Planner of the MoveIt. Furthermore it checks if a Way-Point is inside the IK solution for the loaded robot and changes it state correspondingly.
 *  \author    Risto Kojcev
 */

class SetCartesianImpedance: public QObject
{
Q_OBJECT

public:
	//! Constructor for the Cartesian Impedance Parameters.
	SetCartesianImpedance(QObject* parent = 0);
	//! Virtual Destructor for the Cartesian Impedance Parameters.
	virtual ~SetCartesianImpedance();
	//! Initialization of the necessary Cartesian Impedance Parameters.
	void init();
public Q_SLOTS:
	//! Get the Way-Points from the RViz enviroment and use them to generate Cartesian Path.
  void sendCartImpedanceParams(cartesian_impedance_msgs::SetCartesianImpedancePtr cart_impedance_params);

Q_SIGNALS:
	// //! Let the RViz that a Way-Point is outside the IK solution.
	// void getCartImpedanceParams(cartesian_impedance_msgs::SetCartesianImpedance cart_impedance_params);

private:
	ros::NodeHandle n_;
	ros::Publisher cartesian_impedance_params_pub;

};

#endif // GENERATE_CARTESIAN_PATH_H_
