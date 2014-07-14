#include <moveit_cartesian_planner/add_way_point.h>

#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*0.017453293)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*57.29578)
#endif

//ToDo: add constans for marker creating, control, color and other visualizations!!!!
//ToDo: way-points are redrawn all the time on update, while the interactive marker is not, resolve this issue
//so that the way-points are not redrawn all the time. The user should see all components of the scene all the time
namespace moveit_cartesian_planner
{

AddWayPoint::AddWayPoint(QWidget *parent):rviz::Panel(parent), tf_()//,  target_frame_("/base_link") //change this depending on the move it frame
{

     setObjectName("MoveItPlanner");
     point_sub_.subscribe(n_, "/clicked_point", 10);
     tf_filter_ = new tf::MessageFilter<geometry_msgs::PointStamped>(point_sub_, tf_, target_frame_, 10);
     tf_filter_->registerCallback( boost::bind(&AddWayPoint::msgCallback, this, _1) );
     ROS_INFO("Constructor created;");
}

AddWayPoint::~AddWayPoint()
{

}

void AddWayPoint::onInitialize()
{

    // creating main layout
    path_generate = new GenerateCartesianPath();
    widget_ = new widgets::PathPlanningWidget("~");
    this->parentWidget()->resize(widget_->width(),widget_->height());
    QHBoxLayout* main_layout = new QHBoxLayout(this);
    main_layout->addWidget(widget_);

    server.reset( new interactive_markers::InteractiveMarkerServer("moveit_cartesian_planner") );
    ROS_INFO("initializing..");
    menu_handler.insert( "Delete", boost::bind( &AddWayPoint::processFeedback, this, _1 ) );
    menu_handler.setCheckState(menu_handler.insert( "Fine adjustment", boost::bind( &AddWayPoint::processFeedback, this, _1 )),interactive_markers::MenuHandler::UNCHECKED);
    
    //make all the necessary connections for the QObject communications

    ROS_INFO("Initializing path planning widget");

    connect(path_generate,SIGNAL(getRobotModelFrame_signal(const std::string)),this,SLOT(getRobotModelFrame_slot(const std::string)));

    connect(widget_,SIGNAL(addPoint(tf::Transform)),this,SLOT( addPointFromUI( tf::Transform)));
    connect(widget_,SIGNAL(point_del_UI_signal(std::string)),this,SLOT(point_deleted( std::string)));
    connect(this,SIGNAL(addPointFrom_RViz(const tf::Transform&,const int)),widget_,SLOT(insert_row(const tf::Transform&,const int)));
    connect(this,SIGNAL(point_pose_updated_RViz(const tf::Transform&,const char*)),widget_,SLOT(point_pos_updated_slot(const tf::Transform&,const char*)));
    connect(widget_,SIGNAL(point_pos_updated_signal(const tf::Transform&,const char*)),this,SLOT(point_pose_updated(const tf::Transform&,const char*)));
    connect(this,SIGNAL(point_deleted_from_Rviz(int)),widget_,SLOT(remove_row(int)));


    connect(widget_,SIGNAL(parse_waypoint_btn_signal()),this,SLOT(parse_waypoints()));
    connect(this,SIGNAL(way_points_signal(std::vector<geometry_msgs::Pose>)),path_generate,SLOT(move_to_pose(std::vector<geometry_msgs::Pose>)));
    connect(widget_,SIGNAL(saveToFileBtn_press()),this,SLOT(saveWayPointsToFile()));
    connect(widget_,SIGNAL(clearAllPoints_signal()),this,SLOT(clearAllPoints_RViz()));

    

    connect(path_generate,SIGNAL(wayPointOutOfIK(int,int)),this,SLOT(wayPointOutOfIK_slot(int,int)));
    connect(this,SIGNAL(onUpdatePosCheckIkVadility(const geometry_msgs::Pose&, const int)),path_generate,SLOT(checkWayPointValidity(const geometry_msgs::Pose&, const int)));
    
    connect(this,SIGNAL(initRviz()),path_generate,SLOT(initRviz_done()));

    Q_EMIT initRviz(); 

    //initialize the waypoint count and draw the interaction marker
    count = 0;
    makeBox();
    server->applyChanges();
    
    ROS_INFO("ready.");
      
}

void AddWayPoint::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
  QString text_entry;
  ROS_INFO_STREAM("rviz: Initializing the user interaction planning panel");
  if(config.mapGetString("TextEntry",&text_entry))
  {
    //ROS_INFO_STREAM("Loaded TextEntry with value: "<<text_entry.toStdString());
  }

  ROS_INFO_STREAM("rviz Initialization Finished reading config file");
}

void AddWayPoint::save(rviz::Config config) const
{
  ROS_INFO_STREAM("Saving configuration");
  rviz::Panel::save(config);
  config.mapSetValue("TextEntry",QString::fromStdString( std::string("test_field")));
}

void AddWayPoint::msgCallback(const boost::shared_ptr<const geometry_msgs::PointStamped>& point_ptr)
{
  geometry_msgs::PointStamped point_out;
    try 
    {
      tf_.transformPoint(target_frame_, *point_ptr, point_out);
      
      printf("Clicked on position (x:%f y:%f z:%f)\n", 
             point_out.point.x,
             point_out.point.y,
             point_out.point.z);

      tf::Transform point_pos = tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(point_out.point.x, point_out.point.y, point_out.point.z));
      makeArrow(point_pos,count);
    }
    catch (tf::TransformException &ex) 
    {
      printf ("Failure %s\n", ex.what()); //Print exception which was caught
    }
    server->applyChanges();

}

int AddWayPoint::getCount()
{

  return count;
}

void AddWayPoint::addPointFromUI( const tf::Transform point_pos)
{

  ROS_INFO("Point Added");
  makeArrow(point_pos,count);
}

void AddWayPoint::processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
    {

    tf::Transform point_pos;
    tf::poseMsgToTF(feedback->pose,point_pos);

    makeArrow(point_pos,count);
    //Q_EMIT onUpdatePosCheckIkVadility(feedback->pose,feedback->marker_name.c_str());
    
    // if(strcmp(feedback->marker_name.c_str(),"add_point_button")!=0)
    // Q_EMIT onUpdatePosCheckIkVadility(feedback->pose,atoi(feedback->marker_name.c_str()));
    //server->applyChanges();
    break;
    }

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    {

      tf::Transform point_pos;
      tf::poseMsgToTF(feedback->pose,point_pos);
      point_pose_updated(point_pos, feedback->marker_name.c_str());

      Q_EMIT point_pose_updated_RViz(point_pos, feedback->marker_name.c_str());
      ROS_DEBUG_STREAM("in update function, marker name: " << feedback->marker_name.c_str());

      break;
    }
    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
    {
      //get the menu item which is pressed
      interactive_markers::MenuHandler::EntryHandle menu_item = feedback->menu_entry_id;
      interactive_markers::MenuHandler::CheckState state;

      menu_handler.getCheckState(menu_item,state);

      if(menu_item == 1)
      {
           std::string marker_name = feedback->marker_name;
           int marker_nr = atoi(marker_name.c_str());
           Q_EMIT point_deleted_from_Rviz(marker_nr);
           point_deleted(marker_name);
           break;           
      }
      else
      {
        if(state == interactive_markers::MenuHandler::UNCHECKED)
        {
          ROS_INFO("The selected marker is shown with 6DOF control");
          menu_handler.setCheckState( menu_item, interactive_markers::MenuHandler::CHECKED );
          geometry_msgs::Pose pose;
          changeMarkerControlAndPose( feedback->marker_name.c_str(),true);
          break;
        }
        else 
        {
          menu_handler.setCheckState( menu_item, interactive_markers::MenuHandler::UNCHECKED );
          ROS_INFO("The selected marker is shown as default");
          geometry_msgs::Pose pose;
          changeMarkerControlAndPose( feedback->marker_name.c_str(),false);
          break;
        }
      }
      break;

    }
  }
  server->applyChanges();
}

void AddWayPoint::point_pose_updated(const tf::Transform& point_pos, const char* marker_name)
{

  if(strcmp("add_point_button",marker_name)==0)
  {

        box_pos = point_pos;

        geometry_msgs::Pose pose;
        tf::poseTFToMsg(point_pos,pose);

        std::stringstream s;
        s << "add_point_button";
        server->setPose(s.str(),pose);
        //server->applyChanges();
        //ROS_INFO("Cube position is updated, %f:",box_pos.getOrigin().x());
  }
  else
     {
      int index = atoi(marker_name);
      
      if ( index > waypoints_pos.size() )
      {
        return;
      }
  
      waypoints_pos[index-1] = point_pos;
      geometry_msgs::Pose pose;
      tf::poseTFToMsg(point_pos,pose);
      
      std::stringstream s;
      s << index;
      server->setPose(s.str(),pose);
      server->applyChanges();
      Q_EMIT onUpdatePosCheckIkVadility(pose,index);    
    }
}

InteractiveMarkerControl& AddWayPoint::makeArrowControl_default( InteractiveMarker &msg )
{
  InteractiveMarkerControl control_menu;
  control_menu.always_visible = true;

  Marker marker;

  marker.type = Marker::ARROW;
  marker.scale.x = msg.scale*0.7;
  marker.scale.y = msg.scale*0.08;
  marker.scale.z = msg.scale*0.08;

  //make the markers with interesting color
  marker.color.r = 0.10;
  marker.color.g = 0.20;
  marker.color.b = 0.4;
  marker.color.a = 1.0;

  //make a menu control for the Arrow. This could be used for the user 
  //to delte the arrow on which the mouse pointer is on
  control_menu.interaction_mode = InteractiveMarkerControl::MENU;
  control_menu.name = "menu_select";
  msg.controls.push_back( control_menu );
  control_menu.markers.push_back( marker );

  InteractiveMarkerControl control_move3d;
  control_move3d.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE_3D;
  control_move3d.name = "move";
  control_move3d.markers.push_back( marker );
  msg.controls.push_back( control_move3d );

  server->setCallback( msg.name, boost::bind( &AddWayPoint::processFeedback, this, _1 )); 
  //server->applyChanges();

  return msg.controls.back();

}

InteractiveMarkerControl& AddWayPoint::makeArrowControl_details( InteractiveMarker &msg )
{
  InteractiveMarkerControl control_menu;
  control_menu.always_visible = true;

  Marker marker;

  marker.type = Marker::ARROW;
  marker.scale.x = msg.scale*0.7;
  marker.scale.y = msg.scale*0.08;
  marker.scale.z = msg.scale*0.08;

  //make the markers with interesting color
  marker.color.r = 0.10;
  marker.color.g = 0.20;
  marker.color.b = 0.4;
  marker.color.a = 1.0;


  //make a menu control for the Arrow. This could be used for the user 
  //to delte the arrow on which the mouse pointer is on
  control_menu.interaction_mode = InteractiveMarkerControl::MENU;
  control_menu.name = "menu_select";
  msg.controls.push_back( control_menu );
  control_menu.markers.push_back( marker );


 InteractiveMarkerControl control_view_details;
//*************rotate and move around the x-axis********************
  control_view_details.orientation.w = 1;
  control_view_details.orientation.x = 1;
  control_view_details.orientation.y = 0;
  control_view_details.orientation.z = 0;

  control_view_details.name = "rotate_x";
  control_view_details.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back( control_view_details );

  control_view_details.name = "move_x";
  control_view_details.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back( control_view_details );
//*****************************************************************

//*************rotate and move around the z-axis********************
  control_view_details.orientation.w = 1;
  control_view_details.orientation.x = 0;
  control_view_details.orientation.y = 1;
  control_view_details.orientation.z = 0;

  control_view_details.name = "rotate_z";
  control_view_details.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back( control_view_details );

  control_view_details.name = "move_z";
  control_view_details.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back( control_view_details );
//*****************************************************************


//*************rotate and move around the y-axis********************
  control_view_details.orientation.w = 1;
  control_view_details.orientation.x = 0;
  control_view_details.orientation.y = 0;
  control_view_details.orientation.z = 1;

  control_view_details.name = "rotate_y";
  control_view_details.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back( control_view_details );

  control_view_details.name = "move_y";
  control_view_details.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back( control_view_details );
//*****************************************************************
  control_view_details.markers.push_back( marker );
  msg.controls.push_back( control_view_details );

  server->setCallback( msg.name, boost::bind( &AddWayPoint::processFeedback, this, _1 )); 
  //server->applyChanges();

  return msg.controls.back();

}

void AddWayPoint::makeArrow(const tf::Transform& point_pos,int count_arrow)//
{
      //ROS_INFO_STREAM("Robot Frame in AddWayPoint Object" << GenerateCartesianPath->getRobotModelFrame());
        InteractiveMarker int_marker;

        ROS_INFO_STREAM("Markers frame is: "<< target_frame_);

        //change this later for the user to add different frame ids for the markers
        int_marker.header.frame_id = target_frame_;

         ROS_DEBUG_STREAM("Markers has frame id: "<< int_marker.header.frame_id);

        //static set of the size of the arrow. Can be changed later for estetics.
        int_marker.scale =0.4;

        //move the marker according to the clicked location, there is a problem with this implementation
        tf::poseTFToMsg(point_pos,int_marker.pose);

        //check if we have arrow at that location,if not found then add it
        std::vector<tf::Transform >::iterator it_pos  =std::find((waypoints_pos.begin()),(waypoints_pos.end()-1),point_pos);

        //print vector before each arrow create just for debuggin remove it later
        // for( int i=0;i<waypoints_pos.size();i++)
        //     ROS_INFO_STREAM( "vecotr at start: \n"<<"x:"<< waypoints_pos[i].x()<<"; " << positions[i].y()<< "; "<<positions[i].z()<<";\n");

        // //check the positions and orientations vector if they are emtpy. If they are we have our first marker. Deleted the check of orientations just experimental
        if (waypoints_pos.empty() ) 
        {

            ROS_INFO("Adding first arrow!");
            count_arrow++;
            count=count_arrow;

            waypoints_pos.push_back( point_pos );
            //orientations.push_back( point_orient );
            Q_EMIT addPointFrom_RViz(point_pos,count);
        }
/********************************************Check if we have points in the same position and orientation in the scene***************************************************************************************/   
        // check if we have points in the same position and orientation in the scene. If we do do not add one.
        // This might not be necessary doe to the fact that the user might desire the robot to hold on one position
        // for longer time or to have points with same position but different orientation. 
        else if ((it_pos == (waypoints_pos.end())) || (point_pos.getOrigin() != waypoints_pos.at(count_arrow-1).getOrigin())) // && (point_pos.getOrigin() != waypoints_pos.at(count_arrow-1).getOrigin()) //(it_pos == waypoints_pos.end()) &&
        {

            count_arrow++;
            count=count_arrow;

            waypoints_pos.push_back( point_pos );
            //orientations.push_back( point_orient );

          ROS_INFO("Adding new arrow!");
          Q_EMIT addPointFrom_RViz(point_pos,count);
        }
    else
    {
            //if we have arrow, ignore adding new one and inform the user that there is arrow (waypoint at that location)
            ROS_INFO("There is already a arrow at that location, can't add new one!!");
    }
/*******************************************************************************************************************************************************************************************************************/
        std::stringstream s;
        s << count_arrow;
        ROS_INFO("end of make arrow, count is:%d, positions count:%ld",count,waypoints_pos.size());
        int_marker.name = s.str();
        int_marker.description = s.str();

        makeArrowControl_default(int_marker);
        server->insert( int_marker);
        server->setCallback( int_marker.name, boost::bind( &AddWayPoint::processFeedback, this, _1 )); 
        menu_handler.apply(*server,int_marker.name);
        server->applyChanges();
        Q_EMIT onUpdatePosCheckIkVadility(int_marker.pose,count_arrow);          
}

void AddWayPoint::changeMarkerControlAndPose(std::string marker_name,bool set_control)
{


    InteractiveMarker int_marker;
    server->get(marker_name, int_marker);

    if(set_control==true)
    {
      int_marker.controls.clear();
      makeArrowControl_details(int_marker);
    }
    else if(!set_control)
    {
      int_marker.controls.clear();
      makeArrowControl_default(int_marker);
    } 
    
     //we can use this in another function if we want to manipulate the pose of the marker.
    // if (set_pose)
    //   int_marker.pose = pose;

    server->insert( int_marker);
    //server->applyChanges();
    menu_handler.apply(*server,int_marker.name);
    // tf::Transform marker_pose;
    // tf::poseMsgToTF(int_marker.pose,marker_pose);
    Q_EMIT onUpdatePosCheckIkVadility(int_marker.pose,atoi(marker_name.c_str()));

}

void AddWayPoint::point_deleted(std::string marker_name)
{
    for( int i=0;i<waypoints_pos.size();i++)
            ROS_INFO_STREAM( "vecotr before delete: \n"<<"x:"<< waypoints_pos[i].getOrigin().x()<<"; " << waypoints_pos[i].getOrigin().y()<< "; "<<waypoints_pos[i].getOrigin().z()<<";\n");


    //get the index of the selected marker
    int index = atoi( marker_name.c_str() );
    server->erase(marker_name.c_str());
    waypoints_pos.erase (waypoints_pos.begin()+index-1);

    for( int i=0;i<waypoints_pos.size();i++)
     ROS_INFO_STREAM( "vecotr before delete: \n"<<"x:"<< waypoints_pos[i].getOrigin().x()<<"; " << waypoints_pos[i].getOrigin().y()<< "; "<<waypoints_pos[i].getOrigin().z()<<";\n");
            //InteractiveMarker int_marker;
           for(int i=index+1; i<=count;i++)
           {
              std::stringstream s;
              s << i;
              server->erase(s.str());
              makeArrow(waypoints_pos[i-2],(i-1));
           }
           count--;
           server->applyChanges();
}

InteractiveMarkerControl& AddWayPoint::makeBoxControl( InteractiveMarker &msg )
{
  //define a marker
  Marker marker;

  marker.type = Marker::ARROW;
  marker.scale.x = msg.scale*0.9;
  marker.scale.y = msg.scale*0.1;
  marker.scale.z = msg.scale*0.1;

  //make the markers with interesting color
  marker.color.r = 0.80;
  marker.color.g = 0.20;
  marker.color.b = 0.1;
  marker.color.a = 1.0;

  // //control for button interaction
  InteractiveMarkerControl control;
  control.always_visible = true;

  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.name = "button_interaction";

  control.markers.push_back( marker );
  msg.controls.push_back( control );


  /***************************************************************************************************/
  InteractiveMarkerControl control_view_details;
//*************rotate and move around the x-axis********************
  control_view_details.orientation.w = 1;
  control_view_details.orientation.x = 1;
  control_view_details.orientation.y = 0;
  control_view_details.orientation.z = 0;

  control_view_details.name = "rotate_x";
  control_view_details.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back( control_view_details );

  control_view_details.name = "move_x";
  control_view_details.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back( control_view_details );
//*****************************************************************

//*************rotate and move around the z-axis********************
  control_view_details.orientation.w = 1;
  control_view_details.orientation.x = 0;
  control_view_details.orientation.y = 1;
  control_view_details.orientation.z = 0;

  control_view_details.name = "rotate_z";
  control_view_details.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back( control_view_details );

  control_view_details.name = "move_z";
  control_view_details.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back( control_view_details );
//*****************************************************************


//*************rotate and move around the y-axis********************
  control_view_details.orientation.w = 1;
  control_view_details.orientation.x = 0;
  control_view_details.orientation.y = 0;
  control_view_details.orientation.z = 1;

  control_view_details.name = "rotate_y";
  control_view_details.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back( control_view_details );

  control_view_details.name = "move_y";
  control_view_details.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back( control_view_details );
//*****************************************************************
  control_view_details.markers.push_back( marker );
  msg.controls.push_back( control_view_details );
  /***************************************************************************************************/

  return msg.controls.back();
}

void AddWayPoint::makeBox()
{
        InteractiveMarker int_marker;
        int_marker.header.frame_id = target_frame_;
        int_marker.scale = 0.3;
        ROS_INFO_STREAM("Marker Frame is:" << target_frame_);

        int_marker.pose.position.x = 0.0;
        int_marker.pose.position.y = 0.0;
        int_marker.pose.position.z = 0.0;

        int_marker.description = "Interaction Marker";

        box_pos = tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(0.0,0.0,0.0));

        //button like interactive marker. Detect when we have left click with the mouse and add new arrow then
        int_marker.name = "add_point_button";

        makeBoxControl(int_marker);

        server->insert( int_marker);
        //add interaction feedback to the markers
        server->setCallback( int_marker.name, boost::bind( &AddWayPoint::processFeedback, this, _1 )); 
}
void AddWayPoint::parse_waypoints()
{
  geometry_msgs::Pose target_pose;
  std::vector<geometry_msgs::Pose> waypoints;

  // //we need to change all the positions and orientations vectors to geometry_msgs::Pose, in the next days work on this
  for(int i=0;i<waypoints_pos.size();i++)
  {

    tf::poseTFToMsg (waypoints_pos[i], target_pose);
    //tf::quaternionTFToMsg(orientations[i],target_pose.orientation);

    waypoints.push_back(target_pose);
    // ROS_INFO_STREAM( "positions:"<<waypoints[i].position.x<<";"<< waypoints[i].position.y<<"; " << waypoints[i].position.z);
    // ROS_INFO_STREAM( "orientations:"<<waypoints[i].orientation.x<<";"<<waypoints[i].orientation.y<<";"<<waypoints[i].orientation.z<<";"<<waypoints[i].orientation.w);
  }

  Q_EMIT way_points_signal(waypoints);

}
void AddWayPoint::saveWayPointsToFile()
{
      QString fileName = QFileDialog::getSaveFileName(this,
         tr("Save Way Points"), ".yaml",
         tr("Way Points (*.yaml);;All Files (*)"));

      if (fileName.isEmpty())
         return;
      else {
         QFile file(fileName);
         if (!file.open(QIODevice::WriteOnly)) {
             QMessageBox::information(this, tr("Unable to open file"),
                 file.errorString());
                 file.close();
             return;
      }

    YAML::Emitter out;
    out << YAML::BeginSeq;

    for(int i=0;i<waypoints_pos.size();i++)
  {
      out << YAML::BeginMap;
      std::vector <double> points_vec;
      points_vec.push_back(waypoints_pos[i].getOrigin().x());
      points_vec.push_back(waypoints_pos[i].getOrigin().y());
      points_vec.push_back(waypoints_pos[i].getOrigin().z());

      double rx, ry, rz;

      tf::Matrix3x3 m(waypoints_pos[i].getRotation());
      m.getRPY(rx, ry, rz,1);
      points_vec.push_back(RAD2DEG(rx));
      points_vec.push_back(RAD2DEG(ry));
      points_vec.push_back(RAD2DEG(rz));

      out << YAML::Key << "name";
      out << YAML::Value << (i+1);
      out << YAML::Key << "point";
      out << YAML::Value << YAML::Flow << points_vec;
      out << YAML::EndMap;
  }


    out << YAML::EndSeq;

      std::ofstream myfile;
      myfile.open (fileName.toStdString().c_str());
      myfile << out.c_str();
      myfile.close();
     }
}

void AddWayPoint::clearAllPoints_RViz()
{
  waypoints_pos.clear();
  server->clear();
  //delete the waypoints_pos vector
  count = 0;
  makeBox();
  server->applyChanges();
}
void AddWayPoint::wayPointOutOfIK_slot(int point_number,int out)
{
  //  ros::AsyncSpinner spinner(1);
  // spinner.start();

  InteractiveMarker int_marker;
  visualization_msgs::Marker point_marker;
  std::stringstream marker_name;
  marker_name<<point_number;
  server->get(marker_name.str(), int_marker);

  //InteractiveMarkerControl control;
  int control_size = int_marker.controls.size();
  ROS_INFO_STREAM("size of controls for marker: "<<control_size);
  // control = int_marker.controls.at(control_size-1);
  // point_marker = control.markers.at(0);

if(control_size == 0)
{
  return;
}
else
{
  control_size = control_size -1;
}

  if(out == 1)
  {
    //point_number = point_number + 1;
   ROS_INFO_STREAM("point which is out of reach is"<<point_number);
  // ROS_INFO_STREAM("the name of the marker is:" << int_marker.name.c_str());

    //make the marker outside the IK solution with yellow color
    int_marker.controls.at(control_size).markers.at(0).color.r = 1.0;
    int_marker.controls.at(control_size).markers.at(0).color.g = 1.0;
    int_marker.controls.at(control_size).markers.at(0).color.b = 0.0;
    int_marker.controls.at(control_size).markers.at(0).color.a = 1.0;
}
else
{

    int_marker.controls.at(control_size).markers.at(0).color.r = 0.2;
    int_marker.controls.at(control_size).markers.at(0).color.g = 0.1;
    int_marker.controls.at(control_size).markers.at(0).color.b = 0.4;
    int_marker.controls.at(control_size).markers.at(0).color.a = 1.0;

}
server->insert( int_marker);
server->applyChanges();

}

void AddWayPoint::getRobotModelFrame_slot(const std::string robot_model_frame)
{

  target_frame_.assign(robot_model_frame);
  ROS_INFO_STREAM("The robot model frame is: " << target_frame_);
}

}//end of namespace for add_way_point

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(moveit_cartesian_planner::AddWayPoint,rviz::Panel )