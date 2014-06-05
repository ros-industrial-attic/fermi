#ifndef path_planning_widget_H_
#define path_planning_widget_H_

#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <ui_path_planning_widget.h>

#include <QWidget>
#include <QTimer>
#include <QtConcurrentRun>
#include <QMainWindow>
#include <QTreeView>

namespace moveit_cartesian_planner
{
	namespace widgets {

		class PathPlanningWidget: public QWidget
		{
		Q_OBJECT
		public:
			PathPlanningWidget(std::string ns="");
			virtual ~PathPlanningWidget();
		    std::string get_name()
			{
				return "RobotPathPlanner";
			}
		protected:
			void init();
			std::string param_ns_;
			Ui::PathPlanningWidget ui_;
		protected Q_SLOTS:
		    void point_added_from_UI(const tf::Vector3& point_pos,const tf::Quaternion& point_orient,int& count_arrow,bool marker_deleted);
			void point_delted_from_UI();
			
		};
	}


} //end of namespace moveit_cartesian_planner

#endif //path_planning_widget_H_