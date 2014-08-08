#include <moveit_cartesian_planner/widgets/path_planning_widget.h>
#include <moveit_cartesian_planner/point_tree_model.h>
#include <moveit_cartesian_planner/generate_cartesian_path.h>

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

      ui_.txtPointName->setText("0");
      //set up the default values for the MoveIt and Cartesian Path 
      ui_.lnEdit_PlanTime->setText("5.0");
      ui_.lnEdit_StepSize->setText("0.01");
      ui_.lnEdit_JmpThresh->setText("0.0");
      

      QStringList headers;
      headers<<tr("Point")<<tr("Position (m)")<<tr("Orientation (deg)");
      PointTreeModel *model = new PointTreeModel(headers,"add_point_button");
      ui_.treeView->setModel(model);
      ui_.btn_LoadPath->setToolTip(tr("Load Way-Points from a file"));
      ui_.btn_SavePath->setToolTip(tr("Save Way-Points to a file"));
      ui_.btnAddPoint->setToolTip(tr("Add a new Way-Point"));
      ui_.btnRemovePoint->setToolTip(tr("Remove a selected Way-Point"));

			//connect(ui_.btnAddPoint,SIGNAL(clicked()),this,SLOT(pointAddUI()));
      connect(ui_.btnAddPoint,SIGNAL(clicked()),this,SLOT(pointAddUI()));
      connect(ui_.btnRemovePoint,SIGNAL(clicked()),this,SLOT(pointDeletedUI()));
      connect(ui_.treeView->selectionModel(),SIGNAL(currentChanged(const QModelIndex& , const QModelIndex& )),this,SLOT(selectedPoint(const QModelIndex& , const QModelIndex&)));
      connect(ui_.treeView->model(),SIGNAL(dataChanged(const QModelIndex& , const QModelIndex& )),this,SLOT(treeViewDataChanged(const QModelIndex&,const QModelIndex&)));
      connect(ui_.targetPoint,SIGNAL(clicked()),this,SLOT(sendCartTrajectoryParamsFromUI()));
      connect(ui_.targetPoint,SIGNAL(clicked()),this,SLOT(parseWayPointBtn_slot()));
      connect(ui_.btn_LoadPath,SIGNAL(clicked()),this,SLOT(loadPointsFromFile()));
      connect(ui_.btn_SavePath,SIGNAL(clicked()),this,SLOT(savePointsToFile()));
      connect(ui_.btn_ClearAllPoints,SIGNAL(clicked()),this,SLOT(clearAllPoints_slot()));
      
		}

    void PathPlanningWidget::sendCartTrajectoryParamsFromUI()
    {

        double plan_time_,cart_step_size_,cart_jump_thresh_;
        bool moveit_replan_,avoid_collisions_;

        plan_time_        = ui_.lnEdit_PlanTime->text().toDouble();
        cart_step_size_   = ui_.lnEdit_StepSize->text().toDouble();
        cart_jump_thresh_ = ui_.lnEdit_JmpThresh->text().toDouble();

        moveit_replan_    = ui_.chk_AllowReplanning->isChecked();
        avoid_collisions_ = ui_.chk_AvoidColl->isChecked();

        Q_EMIT cartesianPathParamsFromUI_signal(plan_time_,cart_step_size_,cart_jump_thresh_,moveit_replan_,avoid_collisions_);

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
		void PathPlanningWidget::pointAddUI()
		{

        double x,y,z,rx,ry,rz;
        x = ui_.LineEditX->text().toDouble();
        y = ui_.LineEditY->text().toDouble();
        z = ui_.LineEditZ->text().toDouble();
        rx = DEG2RAD(ui_.LineEditRx->text().toDouble());
        ry = DEG2RAD(ui_.LineEditRy->text().toDouble());
        rz = DEG2RAD(ui_.LineEditRz->text().toDouble());

        // // create transform
        tf::Transform point_pos( tf::Transform(tf::createQuaternionFromRPY(rx,ry,rz),tf::Vector3(x,y,z)));
        Q_EMIT addPoint(point_pos);
        //ROS_INFO_STREAM("Quartenion set at point add UI: "<<q.x()<<"; "<<q.y()<<"; "<<q.z()<<"; "<<q.w()<<";");

        // //reset after adding a point the text fields
        // //set up the starting values for the lineEdit of the positions
        // ui_.LineEditX->setText("0.0");
        // ui_.LineEditY->setText("0.0");
        // ui_.LineEditZ->setText("0.0");
        // //set up the starting values for the lineEdit of the orientations, in Euler angles
        // ui_.LineEditRx->setText("0.0");
        // ui_.LineEditRy->setText("0.0");
        // ui_.LineEditRz->setText("0.0");

        pointRange();         
		}
    void PathPlanningWidget::pointDeletedUI()
    {
        std::string marker_name;
        QString qtPointNr = ui_.txtPointName->text();
        marker_name = qtPointNr.toUtf8().constData();

        int marker_nr = atoi(marker_name.c_str());

        if(strcmp(marker_name.c_str(),"0")!=0)
        {
          removeRow(marker_nr);
          pointRange();
          Q_EMIT pointDelUI_signal(marker_name.c_str());
        }
    }
    void PathPlanningWidget::insertRow(const tf::Transform& point_pos,const int count)
    {

      ROS_INFO("inserting new row in the TreeView");
      QAbstractItemModel *model = ui_.treeView->model();

      //convert the quartenion to roll pitch yaw angle
      tf::Vector3 p = point_pos.getOrigin();
      tfScalar rx,ry,rz;
      point_pos.getBasis().getRPY(rx,ry,rz,1);

      if(count == 0)
      {
        model->insertRow(count,model->index(count, 0));

        model->setData(model->index(0,0,QModelIndex()),QVariant("add_point_button"),Qt::EditRole);
        pointRange();
      }
      else
      {
      //ROS_INFO_STREAM("Quartenion at add_row: "<<orientation.x()<<"; "<<orientation.y()<<"; "<<orientation.z()<<"; "<<orientation.w()<<";");

       if(!model->insertRow(count,model->index(count, 0)))  //&& count==0
       {
         return;
       }
      //set the strings of each axis of the position
      QString pos_x = QString::number(p.x());
      QString pos_y = QString::number(p.y());
      QString pos_z = QString::number(p.z());

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

    }
    void PathPlanningWidget::removeRow(int marker_nr)
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

    void PathPlanningWidget::pointPosUpdated_slot(const tf::Transform& point_pos, const char* marker_name)
    {

        QAbstractItemModel *model = ui_.treeView->model();

        ROS_INFO_STREAM("Updating marker name:"<<marker_name);

        tf::Vector3 p = point_pos.getOrigin();
        tfScalar rx,ry,rz;
        point_pos.getBasis().getRPY(rx,ry,rz,1);

        rx = RAD2DEG(rx);
        ry = RAD2DEG(ry);
        rz = RAD2DEG(rz);

      if((strcmp(marker_name,"add_point_button") == 0) || (atoi(marker_name)==0))
      {
          QString pos_s;
          pos_s = QString::number(p.x()) + "; " + QString::number(p.y()) + "; " + QString::number(p.z()) + ";";
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
          QString pos_x = QString::number(p.x());
          QString pos_y = QString::number(p.y());
          QString pos_z = QString::number(p.z());

          //repeat that with the orientation
          QString orient_x = QString::number(rx);
          QString orient_y = QString::number(ry);
          QString orient_z = QString::number(rz);

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

            tfScalar rx,ry,rz;
            rx = DEG2RAD(orient_x.toDouble());
            ry = DEG2RAD(orient_y.toDouble());
            rz = DEG2RAD(orient_z.toDouble());

            tf::Transform point_pos =  tf::Transform(tf::createQuaternionFromRPY(rx,ry,rz),p);

            Q_EMIT pointPosUpdated_signal(point_pos,temp_str.c_str());
      }

    }
    void PathPlanningWidget::parseWayPointBtn_slot()
    {
      Q_EMIT parseWayPointBtn_signal();
    }

void PathPlanningWidget::loadPointsFromFile()
{

   QString fileName = QFileDialog::getOpenFileName(this,
         tr("Open Way Points File"), "",
         tr("Way Points (*.yaml);;All Files (*)"));

      if (fileName.isEmpty())
         return;
     else {

         QFile file(fileName);

         if (!file.open(QIODevice::ReadOnly)) {
             QMessageBox::information(this, tr("Unable to open file"),
                 file.errorString());
             file.close();
             return;
         }
          //clear all the scene before loading all the new points from the file!!
          clearAllPoints_slot();

          ROS_INFO_STREAM("Opening the file: "<<fileName.toStdString());
          std::ifstream fin(fileName.toStdString().c_str());

        YAML::Node doc;
    #ifdef HAVE_NEW_YAMLCPP
        doc = YAML::Load(fin);
    #else
        YAML::Parser parser(fin);
        parser.GetNextDocument(doc);
    #endif

        for (size_t i = 0; i < doc.size(); i++) {
          std::string name;
          geometry_msgs::Pose pose;
          tf::Transform pose_tf;
         
          double x,y,z,rx, ry, rz;
          doc[i]["name"] >> name;
          doc[i]["point"][0] >> x;
          doc[i]["point"][1] >> y;
          doc[i]["point"][2] >> z;
          doc[i]["point"][3] >> rx;
          doc[i]["point"][4] >> ry;
          doc[i]["point"][5] >> rz;

          rx = DEG2RAD(rx);
          ry = DEG2RAD(ry);
          rz = DEG2RAD(rz);

          pose_tf = tf::Transform(tf::createQuaternionFromRPY(rx,ry,rz),tf::Vector3(x,y,z));


          Q_EMIT addPoint(pose_tf);
        }
      }

    }
    void PathPlanningWidget::savePointsToFile()
    {

      Q_EMIT saveToFileBtn_press();

    }
    void PathPlanningWidget::clearAllPoints_slot()
    {
      //clear the treeView
      QAbstractItemModel *model = ui_.treeView->model();
      model->removeRows(0,model->rowCount());
      ui_.txtPointName->setText("0");
      tf::Transform t;
      t.setIdentity();
      insertRow(t,0); 
      pointRange();

      Q_EMIT clearAllPoints_signal();
    }
    void PathPlanningWidget::setAddPointUIStartPos(const std::string robot_model_frame,const tf::Transform end_effector)
    {

        tf::Vector3 p = end_effector.getOrigin();
        tfScalar rx,ry,rz;
        end_effector.getBasis().getRPY(rx,ry,rz,1);

        rx = RAD2DEG(rx);
        ry = RAD2DEG(ry);
        rz = RAD2DEG(rz);

      //set up the starting values for the lineEdit of the positions
      ui_.LineEditX->setText(QString::number(p.x()));
      ui_.LineEditY->setText(QString::number(p.y()));
      ui_.LineEditZ->setText(QString::number(p.z()));
      //set up the starting values for the lineEdit of the orientations, in Euler angles
      ui_.LineEditRx->setText(QString::number(rx));
      ui_.LineEditRy->setText(QString::number(ry));
      ui_.LineEditRz->setText(QString::number(rz));


    }

    void PathPlanningWidget::cartesianPathStartedHandler()
    {

      ui_.tabWidget->setEnabled(false);
      ui_.targetPoint->setEnabled(false);
      
    }
    void PathPlanningWidget::cartesianPathFinishedHandler()
    {

      ui_.tabWidget->setEnabled(true);
      ui_.targetPoint->setEnabled(true);

    }

  }
}

