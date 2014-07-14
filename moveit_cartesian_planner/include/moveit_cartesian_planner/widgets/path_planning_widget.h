#ifndef path_planning_widget_H_
#define path_planning_widget_H_

#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string.h>

#include <ui_path_planning_widget.h>

#include <moveit_cartesian_planner/add_way_point.h>

#include <QWidget>
#include <QTimer>
#include <QtConcurrentRun>
#include <QMainWindow>
#include <QTreeView>
#include <QStandardItemModel>
#include <QStandardItem>
#include <QSplitter>
#include <QHeaderView>
#include <QCompleter>
#include <QIntValidator>
#include <QDataStream>
#include <QString>
#include <QFileDialog>
 #include <QMessageBox>

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
			QStandardItemModel* pointDataModel;
		private:
			QStringList pointList;
			// checks the range of the points and restricts the user entry for deleting
			// a point from the line edit only to the available points
			void pointRange();
		protected Q_SLOTS:
		    void initTreeView();
			void pointDeletedUI();
			void pointAddUI();
			void insertRow(const tf::Transform& point_pos,const int count);
			void removeRow(int marker_nr);
			void pointPosUpdated_slot( const tf::Transform& point_pos, const char* marker_name);
			void selectedPoint(const QModelIndex& current, const QModelIndex& previous);
			void treeViewDataChanged(const QModelIndex &index,const QModelIndex &index2); 
			void parseWayPointBtn_slot();
			void savePointsToFile();
			void loadPointsFromFile();
			void clearAllPoints_slot();
		Q_SIGNALS:
		    void addPoint( const tf::Transform point_pos );
		    void pointDelUI_signal( std::string marker_name);
		    void pointPosUpdated_signal( const tf::Transform& position, const char* marker_name);
		    void parseWayPointBtn_signal();
		    void saveToFileBtn_press();
		    void clearAllPoints_signal();
			
		};
	}


} //end of namespace moveit_cartesian_planner

#endif //path_planning_widget_H_