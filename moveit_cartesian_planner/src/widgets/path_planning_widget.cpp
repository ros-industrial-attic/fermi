#include <moveit_cartesian_planner/widgets/path_planning_widget.h>
#include <moveit_cartesian_planner/point_tree_model.h>

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
			ui_.txtPos->setText("0.0;0.0;0.0");
			ui_.txtOrient->setText("0.0;0.0;0.0;1.0");
      ui_.txtPointName->setText("0");
      QStringList headers;
      headers<<tr("Point")<<tr("Position")<<tr("Orientation");
      PoinTtreeModel *model = new PoinTtreeModel(headers,"add_point_button");
      ui_.treeView->setModel(model);

      // QCompleter *completer = new QCompleter(pointList);
      // completer->setCaseSensitivity(Qt::CaseInsensitive);
      // ui_.txtPointName->setCompleter(completer);

			//connect(ui_.btnAddPoint,SIGNAL(clicked()),this,SLOT(point_added_from_UI()));
      connect(ui_.btnAddPoint,SIGNAL(clicked()),this,SLOT(point_added_from_UI()));
      connect(ui_.btnRemovePoint,SIGNAL(clicked()),this,SLOT(point_deleted_from_UI()));
      connect(ui_.treeView->selectionModel(),SIGNAL(currentChanged(const QModelIndex& , const QModelIndex& )),this,SLOT(selectedPoint(const QModelIndex& , const QModelIndex&)));
      connect(ui_.treeView->model(),SIGNAL(dataChanged(const QModelIndex& , const QModelIndex& )),this,SLOT(treeViewDataChanged(const QModelIndex&,const QModelIndex&)));
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
      
        ui_.txtPointName->setText(QString::number(current.row()));
    }
		void PathPlanningWidget::point_added_from_UI()
		{

			QStringList strings_pos = ui_.txtPos->text().split( ';' );
			QStringList strings_orient = ui_.txtOrient->text().split( ';' );
			ROS_INFO_STREAM("string:");

			if( (strings_pos.size() >= 3) && (strings_orient.size() >=4))
  				{
    				bool x_ok = true;
    				float x = strings_pos[ 0 ].toFloat( &x_ok );
    				bool y_ok = true;
    				float y = strings_pos[ 1 ].toFloat( &y_ok );
    				bool z_ok = true;
    				float z = strings_pos[ 2 ].toFloat( &z_ok );

    				bool x_orient_ok = true;
    				float x_orient = strings_orient[ 0 ].toFloat( &x_orient_ok );
    				bool y_orient_ok = true;
    				float y_orient = strings_orient[ 1 ].toFloat( &y_orient_ok );
    				bool z_orient_ok = true;
    				float z_orient = strings_orient[ 2 ].toFloat( &z_orient_ok );
    				bool w_orient_ok = true;
    				float w_orient = strings_orient[ 3 ].toFloat( &w_orient_ok );
    				if( x_ok && y_ok && z_ok && x_orient_ok && y_orient_ok && z_orient_ok && w_orient_ok)
    				{

      					Q_EMIT addPoint( tf::Vector3( x, y, z ),tf::Quaternion(x_orient,y_orient,z_orient,w_orient));
    				}
    				else
    				{
    					ui_.txtPos->setText("0.0;0.0;0.0");
    				  ui_.txtOrient->setText("0.0;0.0;0.0;1.0");
    				}
  				}
  				else
  				{
  					    ui_.txtPos->setText("0.0;0.0;0.0");
    				    ui_.txtOrient->setText("0.0;0.0;0.0;1.0");
  				}
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
    void PathPlanningWidget::insert_row(const tf::Vector3& position,const tf::Quaternion& orientation,const int count)
    {

      ROS_INFO("inserting new row in the TreeView");
      //int count = ui_.treeView->model()->rowCount();
      //QModelIndex index  = ui_.treeView->selectionModel()->currentIndex();
      //QModelIndex index = count;
      QAbstractItemModel *model = ui_.treeView->model();

       if((!model->insertRow(count,model->index(count, 0))) && count==0)
       {
         return;
       }
      //model->insertRow(count,model->index(count, 0));


      QString pos_s;
      pos_s = QString::number(position.x()) + "; " + QString::number(position.y()) + "; " + QString::number(position.z()) + ";";
      QString orient_s;
      orient_s = QString::number(orientation.x()) + "; " + QString::number(orientation.y()) + "; " + QString::number(orientation.z()) + "; " + QString::number(orientation.w()) + ";";

      //set the strings of each axis of the position
      QString pos_x = QString::number(position.x());
      QString pos_y = QString::number(position.y());
      QString pos_z = QString::number(position.z());

      //repeat that with the orientation
      QString orient_x = QString::number(orientation.x());
      QString orient_y = QString::number(orientation.y());
      QString orient_z = QString::number(orientation.z());
      QString orient_w = QString::number(orientation.w());

      model->setData(model->index(count,0),QVariant(count),Qt::EditRole);
      // model->setData(model->index(count,1),QVariant(pos_s),Qt::EditRole);
      // model->setData(model->index(count,2),QVariant(orient_s),Qt::EditRole);

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
      model->setData(model->index(0, 0, chldind_pos), QVariant("X"), Qt::EditRole);
      model->setData(model->index(1, 0, chldind_pos), QVariant("Y"), Qt::EditRole);
      model->setData(model->index(2, 0, chldind_pos), QVariant("Z"), Qt::EditRole);

      //second we add the current position information, for each position axis separately
      model->setData(model->index(0, 1, chldind_pos), QVariant(pos_x), Qt::EditRole);
      model->setData(model->index(1, 1, chldind_pos), QVariant(pos_y), Qt::EditRole);
      model->setData(model->index(2, 1, chldind_pos), QVariant(pos_z), Qt::EditRole);
//***************************************************************************************************************************

//*****************************Set the children for the orientation**********************************************************
      //now we repeat everything again,similar as the position for adding the children for the orientation
      model->insertRows(0, 4, chldind_orient);
      //next we set up the data for each of these columns. First the names
      model->setData(model->index(0, 0, chldind_orient), QVariant("X"), Qt::EditRole);
      model->setData(model->index(1, 0, chldind_orient), QVariant("Y"), Qt::EditRole);
      model->setData(model->index(2, 0, chldind_orient), QVariant("Z"), Qt::EditRole);
      model->setData(model->index(3, 0, chldind_orient), QVariant("W"), Qt::EditRole);

      //second we add the current position information, for each position axis separately
      model->setData(model->index(0, 2, chldind_orient), QVariant(orient_x), Qt::EditRole);
      model->setData(model->index(1, 2, chldind_orient), QVariant(orient_y), Qt::EditRole);
      model->setData(model->index(2, 2, chldind_orient), QVariant(orient_z), Qt::EditRole);
      model->setData(model->index(3, 2, chldind_orient), QVariant(orient_w), Qt::EditRole);
//****************************************************************************************************************************
      pointRange();

    }
    void PathPlanningWidget::remove_row(int marker_nr)
    {
        QAbstractItemModel *model = ui_.treeView->model();

        //int row_to_remove = atoi(marker_name.c_str());
        // if((marker_nr != 0) ) //|| (marker_nr < model->rowCount())
        // {
          model->removeRow(marker_nr,QModelIndex());
          ROS_INFO_STREAM("deleting point nr: "<< marker_nr);

          //make a different function of this it doesnt make sense to delete the marker twice!!!!!!
          //Q_EMIT point_del_UI(marker_name);

          for(int i=marker_nr;i<=model->rowCount();++i)
          {
            model->setData(model->index((i-1),0,QModelIndex()),QVariant((i-1)),Qt::EditRole);
          }
          //check how to properly set the selection
          ui_.treeView->selectionModel()->setCurrentIndex(model->index((model->rowCount()-1),0,QModelIndex()),QItemSelectionModel::ClearAndSelect);
          ui_.txtPointName->setText(QString::number(model->rowCount()-1));
        // }
        // else
        //   return;
        pointRange();
    }

    void PathPlanningWidget::point_pos_updated_slot(const char* marker_name, const tf::Vector3& position, const tf::Quaternion& orientation)
    {

        QAbstractItemModel *model = ui_.treeView->model();

        ROS_INFO_STREAM("Updating marker name:"<<marker_name);

      if((strcmp(marker_name,"add_point_button") == 0) && (atoi(marker_name)==0))
      {
          QString pos_s;
          pos_s = QString::number(position.x()) + "; " + QString::number(position.y()) + "; " + QString::number(position.z()) + ";";
          QString orient_s;
          orient_s = QString::number(orientation.x()) + "; " + QString::number(orientation.y()) + "; " + QString::number(orientation.z()) + "; " + QString::number(orientation.w()) + ";";

          model->setData(model->index(0,0),QVariant("add_point_button"),Qt::EditRole);
          model->setData(model->index(0,1),QVariant(pos_s),Qt::EditRole);
          model->setData(model->index(0,2),QVariant(orient_s),Qt::EditRole);
      }
      else
      {

          int changed_marker = atoi(marker_name);

          // QString pos_s;
          // pos_s = QString::number(position.x()) + "; " + QString::number(position.y()) + "; " + QString::number(position.z()) + ";";
          // QString orient_s;
          // orient_s = QString::number(orientation.x()) + "; " + QString::number(orientation.y()) + "; " + QString::number(orientation.z()) + "; " + QString::number(orientation.w()) + ";";

          // model->setData(model->index(changed_marker,0),QVariant(changed_marker),Qt::EditRole);
          // model->setData(model->index(changed_marker,1),QVariant(pos_s),Qt::EditRole);
          // model->setData(model->index(changed_marker,2),QVariant(orient_s),Qt::EditRole);

    //**********************update the positions and orientations of the children as well***********************************************************************************************
          QModelIndex ind = model->index(changed_marker, 0);
          QModelIndex chldind_pos = model->index(0, 0, ind);
          QModelIndex chldind_orient = model->index(1, 0, ind);

          //set the strings of each axis of the position
          QString pos_x = QString::number(position.x());
          QString pos_y = QString::number(position.y());
          QString pos_z = QString::number(position.z());

          //repeat that with the orientation
          QString orient_x = QString::number(orientation.x());
          QString orient_y = QString::number(orientation.y());
          QString orient_z = QString::number(orientation.z());
          QString orient_w = QString::number(orientation.w());

          //second we add the current position information, for each position axis separately
          model->setData(model->index(0, 1, chldind_pos), QVariant(pos_x), Qt::EditRole);
          model->setData(model->index(1, 1, chldind_pos), QVariant(pos_y), Qt::EditRole);
          model->setData(model->index(2, 1, chldind_pos), QVariant(pos_z), Qt::EditRole);

          //second we add the current position information, for each position axis separately
          model->setData(model->index(0, 2, chldind_orient), QVariant(orient_x), Qt::EditRole);
          model->setData(model->index(1, 2, chldind_orient), QVariant(orient_y), Qt::EditRole);
          model->setData(model->index(2, 2, chldind_orient), QVariant(orient_z), Qt::EditRole);
          model->setData(model->index(3, 2, chldind_orient), QVariant(orient_w), Qt::EditRole);

//*****************************************************************************************************************************************************************************************
      }
    }

    void PathPlanningWidget::treeViewDataChanged(const QModelIndex &index,const QModelIndex &index2)
    {

      //this function has a bug!!!!

      qRegisterMetaType<std::string>("std::string");
      QAbstractItemModel *model = ui_.treeView->model();
      QVariant index_data;

      if ((index.parent() == QModelIndex()) && (index.row()!=0))
      {
        // //ROS_INFO_STREAM("Im at the root item with number:"<<index.row());
        // QVariant pos_root = model->data(model->index(index.row(), 1),Qt::EditRole);
        // QVariant orient_root = model->data(model->index(index.row(), 2),Qt::EditRole);
        // QString temp_pos_str = pos_root.toString();
        // QString temp_orient_str = orient_root.toString();

        // //check if these ones are correct!!!
        // QModelIndex chldind_pos = model->index(0, 0,index.sibling(index.row(),0));
        // QModelIndex chldind_orient = model->index(1, 0,index.sibling(index.row(),0));

        // std::string pos_data_str = temp_pos_str.toUtf8().constData();
        // std::string orient_data_str = temp_orient_str.toUtf8().constData();
        // ROS_INFO_STREAM("Data in tree view. Row Nr:"<<index.row()<<"Data Position:"<<pos_data_str.c_str()<<"Data Orientation:"<<orient_data_str.c_str());


        // QStringList strings_pos = temp_pos_str.split( ';' );
        // QStringList strings_orient = temp_orient_str.split( ';' );

        //  std::stringstream s;
        //  s<<index.row();
        //  std::string temp_str = s.str();

        // if( (strings_pos.size() >= 3) && (strings_orient.size() >=4))
        //   {
        //     bool x_ok = true;
        //     float x = strings_pos[ 0 ].toFloat( &x_ok );
        //     bool y_ok = true;
        //     float y = strings_pos[ 1 ].toFloat( &y_ok );
        //     bool z_ok = true;
        //     float z = strings_pos[ 2 ].toFloat( &z_ok );

        //     bool x_orient_ok = true;
        //     float x_orient = strings_orient[ 0 ].toFloat( &x_orient_ok );
        //     bool y_orient_ok = true;
        //     float y_orient = strings_orient[ 1 ].toFloat( &y_orient_ok );
        //     bool z_orient_ok = true;
        //     float z_orient = strings_orient[ 2 ].toFloat( &z_orient_ok );
        //     bool w_orient_ok = true;
        //     float w_orient = strings_orient[ 3 ].toFloat( &w_orient_ok );
        //     if( x_ok && y_ok && z_ok && x_orient_ok && y_orient_ok && z_orient_ok && w_orient_ok)
        //     {

        //       //second we add the current position information, for each position axis separately
        //       model->setData(model->index(0, 1, chldind_pos), QVariant(x), Qt::EditRole);
        //       model->setData(model->index(1, 1, chldind_pos), QVariant(y), Qt::EditRole);
        //       model->setData(model->index(2, 1, chldind_pos), QVariant(z), Qt::EditRole);

        //       //second we add the current position information, for each position axis separately
        //       model->setData(model->index(0, 2, chldind_orient), QVariant(x_orient), Qt::EditRole);
        //       model->setData(model->index(1, 2, chldind_orient), QVariant(y_orient), Qt::EditRole);
        //       model->setData(model->index(2, 2, chldind_orient), QVariant(z_orient), Qt::EditRole);
        //       model->setData(model->index(3, 2, chldind_orient), QVariant(w_orient), Qt::EditRole);

        //       //Q_EMIT point_pos_updated_signal( temp_str.c_str(),tf::Vector3( x, y, z ),tf::Quaternion(x_orient,y_orient,z_orient,w_orient));
        //     }
        //     else
        //     {
        //       QString pos_s;
        //       pos_s = "0.0;0.0;0.0";
        //       QString orient_s;
        //       orient_s = "0.0;0.0;0.0;1.0;";

        //       model->setData(model->index(index.row(),1,QModelIndex()),QVariant(pos_s),Qt::EditRole);
        //       model->setData(model->index(index.row(),2,QModelIndex()),QVariant(orient_s),Qt::EditRole);
        //       //Q_EMIT point_pos_updated_signal( temp_str.c_str(),tf::Vector3( 0.0, 0.0, 0.0 ),tf::Quaternion(0.0,0.0,0.0,1.0));
        //     }
        //   }


      }
      else if(((index.parent().parent()) != QModelIndex()) && (index.parent().parent().row()!=0))
      {
          ROS_INFO("I am editing a child item");
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
          QVariant orient_w = model->data(model->index(3, 2,chldind_orient),Qt::EditRole);

          //if(chldind_pos.isValid() && chldind_orient.isValid())
          //{

            // QString pos_s;
            // pos_s = pos_x.toString() + "; " + pos_y.toString() + "; " + pos_z.toString() + ";";
            // QString orient_s;
            // orient_s = orient_x.toString() + "; " + orient_y.toString() + "; " + orient_z.toString() + "; " + orient_w.toString() + ";";

            // std::string pos_data_str = pos_s.toUtf8().constData();
            // std::string orient_data_str = orient_s.toUtf8().constData();

            // ROS_INFO_STREAM("Row nr:"<<main_root.row()<<"Data Position Compiled: "<<pos_data_str.c_str()<<" Data Orientation Compiled: "<<orient_data_str.c_str());

            //model->setData(model->index(main_root.row(),1,QModelIndex()),QVariant(pos_s),Qt::EditRole);
            //model->setData(model->index(main_root.row(),2,QModelIndex()),QVariant(orient_s),Qt::EditRole);

            Q_EMIT point_pos_updated_signal( temp_str.c_str(),tf::Vector3( pos_x.toDouble(), pos_y.toDouble(), pos_z.toDouble() ),tf::Quaternion(orient_x.toDouble(),orient_y.toDouble(),orient_z.toDouble(),orient_w.toDouble()));
          //}

      }

    }

	}
}