#include <moveit_cartesian_planner/widgets/path_planning_widget.h>
#include <moveit_cartesian_planner/point_tree_model.h>
#include <moveit_cartesian_planner/generate_cartesian_path.h>



#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*0.017453293)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*57.29578)
#endif

namespace moveit_cartesian_planner
{
	namespace widgets {

		PathPlanningWidget::PathPlanningWidget(std::string ns):
		param_ns_(ns)
		{
			init();
		}
		PathPlanningWidget::~PathPlanningWidget()
		{

		}
		void PathPlanningWidget::init()
		{

			// initializing gui
			ui_.setupUi(this);
      //set up the starting values for the lineEdit of the positions
      ui_.LineEditX->setText("0.0");
      ui_.LineEditY->setText("0.0");
      ui_.LineEditZ->setText("0.0");
      //set up the starting values for the lineEdit of the orientations, in Euler angles
      ui_.LineEditRx->setText("0.0");
      ui_.LineEditRy->setText("0.0");
      ui_.LineEditRz->setText("0.0");

      ui_.txtPointName->setText("0");
      QStringList headers;
      headers<<tr("Point")<<tr("Position (m)")<<tr("Orientation (deg)");
      PoinTtreeModel *model = new PoinTtreeModel(headers,"add_point_button");
      ui_.treeView->setModel(model);

			//connect(ui_.btnAddPoint,SIGNAL(clicked()),this,SLOT(point_added_from_UI()));
      connect(ui_.btnAddPoint,SIGNAL(clicked()),this,SLOT(point_added_from_UI()));
      connect(ui_.btnRemovePoint,SIGNAL(clicked()),this,SLOT(point_deleted_from_UI()));
      connect(ui_.treeView->selectionModel(),SIGNAL(currentChanged(const QModelIndex& , const QModelIndex& )),this,SLOT(selectedPoint(const QModelIndex& , const QModelIndex&)));
      connect(ui_.treeView->model(),SIGNAL(dataChanged(const QModelIndex& , const QModelIndex& )),this,SLOT(treeViewDataChanged(const QModelIndex&,const QModelIndex&)));
      connect(ui_.targetPoint,SIGNAL(clicked()),this,SLOT(parse_waypoint_btn_slot()));
		}
    void PathPlanningWidget::pointRange()
    {
      QAbstractItemModel *model=ui_.treeView->model();
      int count = model->rowCount()-1;
      ui_.txtPointName->setValidator(new QIntValidator(1,count,ui_.txtPointName));
    }

    void PathPlanningWidget::initTreeView()
    {

        QAbstractItemModel *model=ui_.treeView->model();

        model->setData(model->index(0,0,QModelIndex()),QVariant("add_point_button"),Qt::EditRole);
        model->setData(model->index(0,1,QModelIndex()),QVariant("0;0;0;"),Qt::EditRole);
        model->setData(model->index(0,2,QModelIndex()),QVariant("0;0;0;1;"),Qt::EditRole);

        //update the validator for the lineEdit Point
        pointRange();
    }
    void PathPlanningWidget::selectedPoint(const QModelIndex& current, const QModelIndex& previous)
    {
      ROS_INFO("Selected Index Changed");

       if(current.parent()==QModelIndex())
        ui_.txtPointName->setText(QString::number(current.row()));
      else if((current.parent()!=QModelIndex()) && (current.parent().parent() == QModelIndex()))
        ui_.txtPointName->setText(QString::number(current.parent().row()));
      else 
        ui_.txtPointName->setText(QString::number(current.parent().parent().row()));
    }
		void PathPlanningWidget::point_added_from_UI()
		{

        double x,y,z,rx,ry,rz;
        x = ui_.LineEditX->text().toDouble();
        y = ui_.LineEditY->text().toDouble();
        z = ui_.LineEditZ->text().toDouble();
        rx = DEG2RAD(ui_.LineEditRx->text().toDouble());
        ry = DEG2RAD(ui_.LineEditRy->text().toDouble());
        rz = DEG2RAD(ui_.LineEditRz->text().toDouble());

        // create transform
        tf::Vector3 p(x,y,z);
        tf::Quaternion q;

        tf::Matrix3x3 m;
        m.setRPY(rx,ry,rz);
        m.getRotation(q);
        //q.setRPY(rx,ry,rz);
        tf::Transform point_pos(q,p);
        Q_EMIT addPoint(point_pos);
        ROS_INFO_STREAM("Quartenion set at point add UI: "<<q.x()<<"; "<<q.y()<<"; "<<q.z()<<"; "<<q.w()<<";");

        //reset after adding a point the text fields
        //set up the starting values for the lineEdit of the positions
        ui_.LineEditX->setText("0.0");
        ui_.LineEditY->setText("0.0");
        ui_.LineEditZ->setText("0.0");
        //set up the starting values for the lineEdit of the orientations, in Euler angles
        ui_.LineEditRx->setText("0.0");
        ui_.LineEditRy->setText("0.0");
        ui_.LineEditRz->setText("0.0");

        pointRange();         
		}
    void PathPlanningWidget::point_deleted_from_UI()
    {
        std::string marker_name;
        QString qtPointNr = ui_.txtPointName->text();
        marker_name = qtPointNr.toUtf8().constData();

        int marker_nr = atoi(marker_name.c_str());

        if(strcmp(marker_name.c_str(),"0")!=0)
        {
          remove_row(marker_nr);
          pointRange();
          Q_EMIT point_del_UI_signal(marker_name.c_str());
        }
    }
    void PathPlanningWidget::insert_row(const tf::Transform& point_pos,const int count)
    {

      ROS_INFO("inserting new row in the TreeView");
      QAbstractItemModel *model = ui_.treeView->model();

      //convert the quartenion to roll pitch yaw angle
      double rx,ry,rz;
      tf::Matrix3x3 m(point_pos.getRotation());
      m.getRPY(rx, ry, rz,1);
      //ROS_INFO_STREAM("Quartenion at add_row: "<<orientation.x()<<"; "<<orientation.y()<<"; "<<orientation.z()<<"; "<<orientation.w()<<";");

       if((!model->insertRow(count,model->index(count, 0))) && count==0)
       {
         return;
       }
      //set the strings of each axis of the position
      QString pos_x = QString::number(point_pos.getOrigin().x());
      QString pos_y = QString::number(point_pos.getOrigin().y());
      QString pos_z = QString::number(point_pos.getOrigin().z());

      //repeat that with the orientation
      QString orient_x = QString::number(RAD2DEG(rx));
      QString orient_y = QString::number(RAD2DEG(ry));
      QString orient_z = QString::number(RAD2DEG(rz));

      model->setData(model->index(count,0),QVariant(count),Qt::EditRole);

      //add a child to the last inserted item. First add children in the treeview that
      //are just telling the user that if he expands them he can see details about the position and orientation of each point
      QModelIndex ind = model->index(count, 0);
      model->insertRows(0, 2, ind);
      QModelIndex chldind_pos = model->index(0, 0, ind);
      QModelIndex chldind_orient = model->index(1, 0, ind);
      model->setData(chldind_pos, QVariant("Position"), Qt::EditRole);
      model->setData(chldind_orient, QVariant("Orientation"), Qt::EditRole);
//*****************************Set the children for the position**********************************************************
      //now add information about each child separately. For the position we have coordinates for X,Y,Z axis.
      //therefore we add 3 rows of information
      model->insertRows(0, 3, chldind_pos);

      //next we set up the data for each of these columns. First the names
      model->setData(model->index(0, 0, chldind_pos), QVariant("X:"), Qt::EditRole);
      model->setData(model->index(1, 0, chldind_pos), QVariant("Y:"), Qt::EditRole);
      model->setData(model->index(2, 0, chldind_pos), QVariant("Z:"), Qt::EditRole);

      //second we add the current position information, for each position axis separately
      model->setData(model->index(0, 1, chldind_pos), QVariant(pos_x), Qt::EditRole);
      model->setData(model->index(1, 1, chldind_pos), QVariant(pos_y), Qt::EditRole);
      model->setData(model->index(2, 1, chldind_pos), QVariant(pos_z), Qt::EditRole);
//***************************************************************************************************************************

//*****************************Set the children for the orientation**********************************************************
      //now we repeat everything again,similar as the position for adding the children for the orientation
      model->insertRows(0, 3, chldind_orient);
      //next we set up the data for each of these columns. First the names
      model->setData(model->index(0, 0, chldind_orient), QVariant("Rx:"), Qt::EditRole);
      model->setData(model->index(1, 0, chldind_orient), QVariant("Ry:"), Qt::EditRole);
      model->setData(model->index(2, 0, chldind_orient), QVariant("Rz:"), Qt::EditRole);

      //second we add the current position information, for each position axis separately
      model->setData(model->index(0, 2, chldind_orient), QVariant(orient_x), Qt::EditRole);
      model->setData(model->index(1, 2, chldind_orient), QVariant(orient_y), Qt::EditRole);
      model->setData(model->index(2, 2, chldind_orient), QVariant(orient_z), Qt::EditRole);
//****************************************************************************************************************************
      pointRange();

    }
    void PathPlanningWidget::remove_row(int marker_nr)
    {
        QAbstractItemModel *model = ui_.treeView->model();

          model->removeRow(marker_nr,QModelIndex());
          ROS_INFO_STREAM("deleting point nr: "<< marker_nr);

          for(int i=marker_nr;i<=model->rowCount();++i)
          {
            model->setData(model->index((i-1),0,QModelIndex()),QVariant((i-1)),Qt::EditRole);
          }
          //check how to properly set the selection
          ui_.treeView->selectionModel()->setCurrentIndex(model->index((model->rowCount()-1),0,QModelIndex()),QItemSelectionModel::ClearAndSelect);
          ui_.txtPointName->setText(QString::number(model->rowCount()-1));
        pointRange();
    }

    void PathPlanningWidget::point_pos_updated_slot(const tf::Transform& point_pos, const char* marker_name)
    {

        QAbstractItemModel *model = ui_.treeView->model();

        ROS_INFO_STREAM("Updating marker name:"<<marker_name);

        //convert the quartenion to roll pitch yaw Euler angle
        double rx,ry,rz;
        tf::Matrix3x3 m(point_pos.getRotation());
        //ROS_INFO_STREAM("Quartenion set at update_point: "<<orientation.x()<<"; "<<orientation.y()<<"; "<<orientation.z()<<"; "<<orientation.w()<<";");
        m.getRPY(rx, ry, rz,1);

      if((strcmp(marker_name,"add_point_button") == 0) || (atoi(marker_name)==0))
      {
          QString pos_s;
          pos_s = QString::number(point_pos.getOrigin().x()) + "; " + QString::number(point_pos.getOrigin().y()) + "; " + QString::number(point_pos.getOrigin().z()) + ";";
          QString orient_s;
          orient_s = QString::number(rx) + "; " + QString::number(ry) + "; " + QString::number(rz) + ";";

          model->setData(model->index(0,0),QVariant("add_point_button"),Qt::EditRole);
          model->setData(model->index(0,1),QVariant(pos_s),Qt::EditRole);
          model->setData(model->index(0,2),QVariant(orient_s),Qt::EditRole);
      }
      else
      {

          int changed_marker = atoi(marker_name);
    //**********************update the positions and orientations of the children as well***********************************************************************************************
          QModelIndex ind = model->index(changed_marker, 0);
          QModelIndex chldind_pos = model->index(0, 0, ind);
          QModelIndex chldind_orient = model->index(1, 0, ind);

          //set the strings of each axis of the position
          QString pos_x = QString::number(point_pos.getOrigin().x());
          QString pos_y = QString::number(point_pos.getOrigin().y());
          QString pos_z = QString::number(point_pos.getOrigin().z());

          //repeat that with the orientation
          QString orient_x = QString::number(RAD2DEG(rx));
          QString orient_y = QString::number(RAD2DEG(ry));
          QString orient_z = QString::number(RAD2DEG(rz));

          //second we add the current position information, for each position axis separately
          model->setData(model->index(0, 1, chldind_pos), QVariant(pos_x), Qt::EditRole);
          model->setData(model->index(1, 1, chldind_pos), QVariant(pos_y), Qt::EditRole);
          model->setData(model->index(2, 1, chldind_pos), QVariant(pos_z), Qt::EditRole);

          //second we add the current position information, for each position axis separately
          model->setData(model->index(0, 2, chldind_orient), QVariant(orient_x), Qt::EditRole);
          model->setData(model->index(1, 2, chldind_orient), QVariant(orient_y), Qt::EditRole);
          model->setData(model->index(2, 2, chldind_orient), QVariant(orient_z), Qt::EditRole);
//*****************************************************************************************************************************************************************************************
      }
    }

    void PathPlanningWidget::treeViewDataChanged(const QModelIndex &index,const QModelIndex &index2)
    {

      qRegisterMetaType<std::string>("std::string");
      QAbstractItemModel *model = ui_.treeView->model();
      QVariant index_data;

      if ((index.parent() == QModelIndex()) && (index.row()!=0))
      {
        

      }
      else if(((index.parent().parent()) != QModelIndex()) && (index.parent().parent().row()!=0))
      {
          //ROS_INFO("I am editing a child item");
          QModelIndex main_root = index.parent().parent();
          std::stringstream s;
          s<<main_root.row();
          std::string temp_str = s.str();

          QModelIndex chldind_pos = model->index(0, 0, main_root.sibling(main_root.row(),0));
          QModelIndex chldind_orient = model->index(1, 0, main_root.sibling(main_root.row(),0));

          QVariant pos_x = model->data(model->index(0, 1,chldind_pos),Qt::EditRole);
          QVariant pos_y = model->data(model->index(1, 1,chldind_pos),Qt::EditRole);
          QVariant pos_z = model->data(model->index(2, 1,chldind_pos),Qt::EditRole);

          QVariant orient_x = model->data(model->index(0, 2,chldind_orient),Qt::EditRole);
          QVariant orient_y = model->data(model->index(1, 2,chldind_orient),Qt::EditRole);
          QVariant orient_z = model->data(model->index(2, 2,chldind_orient),Qt::EditRole);

            tf::Vector3 p(pos_x.toDouble(),pos_y.toDouble(),pos_z.toDouble());

            tf::Quaternion q;
            float rx,ry,rz;
            rx = DEG2RAD(orient_x.toDouble());
            ry = DEG2RAD(orient_y.toDouble());
            rz = DEG2RAD(orient_z.toDouble());

            tf::Matrix3x3 m;
            m.setRPY(rx,ry,rz);
            m.getRotation(q);
            //q.setRPY(DEG2RAD(orient_x.toDouble()),DEG2RAD(orient_y.toDouble()),DEG2RAD(orient_z.toDouble()));
            ROS_INFO_STREAM("Quartenion at TreeView Edit: "<<q.x()<<"; "<<q.y()<<"; "<<q.z()<<"; "<<q.w()<<";");
            tf::Transform point_pos = tf::Transform(q,p);

            Q_EMIT point_pos_updated_signal(point_pos,temp_str.c_str());
      }

    }
    void PathPlanningWidget::parse_waypoint_btn_slot()
    {
      Q_EMIT parse_waypoint_btn_signal();
    }

	}
}