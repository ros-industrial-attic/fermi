#include <moveit_cartesian_planner/add_way_point.h>


namespace moveit_cartesian_planner
{

AddWayPoint::AddWayPoint(QWidget *parent):rviz::Panel(parent), tf_(),  target_frame_("/base_link") //change this depending on the move it frame
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
    server.reset( new interactive_markers::InteractiveMarkerServer("Sphere") );
    ROS_INFO("initializing..");
    menu_handler.insert( "Delete", boost::bind( &AddWayPoint::processFeedback, this, _1 ) );
    menu_handler.setCheckState(menu_handler.insert( "Fine adjustment", boost::bind( &AddWayPoint::processFeedback, this, _1 )),interactive_markers::MenuHandler::UNCHECKED);

    //reserve for the size of the vector, just a number I have chosen
    // positions.reserve(255);
    // orientations.reserve(255);

    count = 0;
    makeBox();
    server->applyChanges();

    ROS_INFO("Initializing path planning widget");
    // creating main layout
    widget_ = new widgets::PathPlanningWidget("~");
    path_generate = new GenerateCartesianPath();
    this->parentWidget()->resize(widget_->width(),widget_->height());
    QHBoxLayout* main_layout = new QHBoxLayout(this);
    main_layout->addWidget(widget_);

    connect(widget_,SIGNAL(addPoint(tf::Transform)),this,SLOT( addPointFromUI( tf::Transform)));
    connect(widget_,SIGNAL(point_del_UI_signal(std::string)),this,SLOT(point_deleted( std::string)));
    connect(this,SIGNAL(addPointFrom_RViz(const tf::Transform&,const int)),widget_,SLOT(insert_row(const tf::Transform&,const int)));
    connect(this,SIGNAL(point_pose_updated_RViz(const tf::Transform&,const char*)),widget_,SLOT(point_pos_updated_slot(const tf::Transform&,const char*)));
    connect(widget_,SIGNAL(point_pos_updated_signal(const tf::Transform&,const char*)),this,SLOT(point_pose_updated(const tf::Transform&,const char*)));
    connect(this,SIGNAL(point_deleted_from_Rviz(int)),widget_,SLOT(remove_row(int)));
    connect(this,SIGNAL(initRviz()),widget_,SLOT(initTreeView()));

    connect(widget_,SIGNAL(parse_waypoint_btn_signal()),this,SLOT(parse_waypoints()));
    connect(this,SIGNAL(way_points_signal(std::vector<geometry_msgs::Pose>)),path_generate,SLOT(move_to_pose(std::vector<geometry_msgs::Pose>)));


    Q_EMIT initRviz();

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

      // tf::Vector3 point_posit;
      // tf::Quaternion point_orient;
      // point_posit = tf::Vector3(point_out.point.x, point_out.point.y, point_out.point.z);
      // //set orientation in the x-axis by default when we have mouse click
      // point_orient = tf::Quaternion(0.0,0.0,0.0,1.0);
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

  std::ostringstream s;

  s<<"Feedback fom marker '" << feedback->marker_name << "' "
                             << "/ control '"<<feedback->control_name << "'";
  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
    {

    tf::Transform point_pos;
    tf::poseMsgToTF(feedback->pose,point_pos);
    //ROS_INFO_STREAM("see if point works:"<<point_pos.getOrigin().x());

    makeArrow(point_pos,count);
    server->applyChanges();
    break;
    }

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    {

      tf::Transform point_pos;
      tf::poseMsgToTF(feedback->pose,point_pos);
      point_pose_updated(point_pos, feedback->marker_name.c_str());

      Q_EMIT point_pose_updated_RViz(point_pos, feedback->marker_name.c_str());
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
        server->applyChanges();
        ROS_INFO("Cube position is updated, %f:",box_pos.getOrigin().x());

      }
     else
     {
      int index = atoi(marker_name);
      
      if ( index > waypoints_pos.size() )
      {
        return;
      }
  
      waypoints_pos[index-1] = point_pos;
      //orientations[index-1] = orientation;
      geometry_msgs::Pose pose;
      tf::poseTFToMsg(point_pos,pose);

      std::stringstream s;
      s << index;
      server->setPose(s.str(),pose);
      server->applyChanges();
      ROS_INFO_STREAM("Arrow: "<<index <<"; position is updated to:"<<pose.position.x<<" , marker stringname: "<<s.str()<<"; \n");     
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
  marker.color.b = 1.0;
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
  server->applyChanges();

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
  marker.color.b = 1.0;
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
  server->applyChanges();

  return msg.controls.back();

}

void AddWayPoint::makeArrow(const tf::Transform& point_pos,int count_arrow)//
{
        InteractiveMarker int_marker;

        //change this later for the user to add different frame ids for the markers
        int_marker.header.frame_id = "/base_link";

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
    server->applyChanges();
    menu_handler.apply(*server,int_marker.name);

}

void AddWayPoint::point_deleted(std::string marker_name)
{
    //ROS_INFO("Info before the delete. Deleting marker: %s, count = %d, vector size: %lu",marker_name,count,positions.size());

    for( int i=0;i<waypoints_pos.size();i++)
            ROS_INFO_STREAM( "vecotr before delete: \n"<<"x:"<< waypoints_pos[i].getOrigin().x()<<"; " << waypoints_pos[i].getOrigin().y()<< "; "<<waypoints_pos[i].getOrigin().z()<<";\n");


    //get the index of the selected marker
    int index = atoi( marker_name.c_str() );
    server->erase(marker_name.c_str());
    waypoints_pos.erase (waypoints_pos.begin()+index-1);
    //orientations.erase (orientations.begin()+index-1);

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

  InteractiveMarkerControl control;
  control.always_visible = true;
  //control.interaction_mode = InteractiveMarkerControl::BUTTON;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;  
  control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  //control.independent_marker_orientation = true;
  msg.controls.push_back( control );

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;  
  control.orientation_mode = InteractiveMarkerControl::INHERIT;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  //control.independent_marker_orientation = true;
  msg.controls.push_back( control );

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;  
  control.orientation_mode = InteractiveMarkerControl::INHERIT;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  //control.independent_marker_orientation = true;

  control.name = "move";
  msg.controls.push_back( control );

  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.name = "button_interaction";

  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale*0.6;
  marker.scale.y = msg.scale*0.6;
  marker.scale.z = msg.scale*0.6;


  //make the markers with interesting color
  marker.color.r = 0.80;
  marker.color.g = 0.20;
  marker.color.b = 0.1;
  marker.color.a = 1.0;

  control.markers.push_back( marker );
  msg.controls.push_back( control );

  return msg.controls.back();
}

void AddWayPoint::makeBox()
{
        InteractiveMarker int_marker;
        int_marker.header.frame_id = "/base_link";
        int_marker.scale = 0.3;

        int_marker.pose.position.x = 0.0;
        int_marker.pose.position.y = 0.0;
        int_marker.pose.position.z = 0.0;

        int_marker.description = "Interaction Marker";

        //positions.push_back( tf::Vector3(0.0,0.0,0.0) );
        // orientation_box = tf::Quaternion(0.0,0.0,0.0,1.0);
        // position_box = tf::Vector3(0.0,0.0,0.0);
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
    ROS_INFO_STREAM( "positions:"<<waypoints[i].position.x<<";"<< waypoints[i].position.y<<"; " << waypoints[i].position.z);
    ROS_INFO_STREAM( "orientations:"<<waypoints[i].orientation.x<<";"<<waypoints[i].orientation.y<<";"<<waypoints[i].orientation.z<<";"<<waypoints[i].orientation.w);
  }

  Q_EMIT way_points_signal(waypoints);

}

}//end of namespace for add_way_point

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(moveit_cartesian_planner::AddWayPoint,rviz::Panel )