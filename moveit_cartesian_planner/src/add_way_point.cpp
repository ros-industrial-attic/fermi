#include <moveit_cartesian_planner/add_way_point.h>

namespace moveit_cartesian_planner
{

AddWayPoint::AddWayPoint(QWidget *parent):rviz::Panel(parent), tf_(),  target_frame_("/point_pos") //change this depending on the move it frame
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
    //this->parentWidget()->resize(widget_->width(),widget_->height());
    QHBoxLayout* main_layout = new QHBoxLayout(this);;
    main_layout->addWidget(widget_);

    connect(widget_,SIGNAL(addPoint(tf::Vector3,tf::Quaternion)),this,SLOT(addPointFromUI( tf::Vector3,tf::Quaternion)));
    connect(widget_,SIGNAL(point_del_UI_signal(std::string)),this,SLOT(point_deleted( std::string)));
    connect(this,SIGNAL(addPointFrom_RViz(const tf::Vector3&,const tf::Quaternion&,const int)),widget_,SLOT(insert_row(const tf::Vector3&,const tf::Quaternion&,const int)));
    connect(this,SIGNAL(point_pose_updated_RViz(const char*,const tf::Vector3&,const tf::Quaternion&)),widget_,SLOT(point_pos_updated_slot(const char*,const tf::Vector3&,const tf::Quaternion&)));
    connect(widget_,SIGNAL(point_pos_updated_signal(const char*,const tf::Vector3&,const tf::Quaternion&)),this,SLOT(point_pose_updated(const char*,const tf::Vector3&,const tf::Quaternion&)));
    connect(this,SIGNAL(point_deleted_from_Rviz(int)),widget_,SLOT(remove_row(int)));
    connect(this,SIGNAL(initRviz()),widget_,SLOT(initTreeView()));

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

      tf::Vector3 point_pos;
      tf::Quaternion point_orient;
      point_pos = tf::Vector3(point_out.point.x, point_out.point.y, point_out.point.z);
      //set orientation in the x-axis by default when we have mouse click
      point_orient = tf::Quaternion(0.0,0.0,0.0,1.0);
      makeArrow(point_pos,point_orient,count);
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

void AddWayPoint::addPointFromUI( const tf::Vector3 position,const tf::Quaternion orientation)
{

  ROS_INFO("Point Added");
  makeArrow(position,orientation,count);
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
 
    tf::Vector3 point_pos;
    tf::Quaternion point_orient;
    point_pos = tf::Vector3(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
    point_orient = tf::Quaternion(feedback->pose.orientation.x,feedback->pose.orientation.y,feedback->pose.orientation.z,feedback->pose.orientation.w);


    makeArrow(point_pos,point_orient,count);
    server->applyChanges();
    break;
    }

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    {


      tf::Vector3 fb_pos(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
      tf::Quaternion fb_orient(feedback->pose.orientation.x,feedback->pose.orientation.y,feedback->pose.orientation.z,feedback->pose.orientation.w);
      
      point_pose_updated(feedback->marker_name.c_str(), fb_pos,fb_orient);
      Q_EMIT point_pose_updated_RViz(feedback->marker_name.c_str(), fb_pos,fb_orient);
      
      

      break;
    }
    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
    {
      //get the menu item which is pressed
      interactive_markers::MenuHandler::EntryHandle menu_item = feedback->menu_entry_id;
      interactive_markers::MenuHandler::CheckState state;

      menu_handler.getCheckState(menu_item,state);

      //ROS_INFO_STREAM(s.str() << ": menu item " << menu_item << " clicked" << mouse_point_ss.str() << ".");
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

void AddWayPoint::point_pose_updated(const char* marker_name, const tf::Vector3& position, const tf::Quaternion& orientation)
{

  if(strcmp("add_point_button",marker_name)==0)
      {
        // tf::Vector3 fb_pos(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
        // tf::Quaternion fb_orient(feedback->pose.orientation.x,feedback->pose.orientation.y,feedback->pose.orientation.z,feedback->pose.orientation.w);

        position_box = position;
        orientation_box = orientation;

        geometry_msgs::Pose pose;
        //pose = feedback->pose;
        //correct this should be only pose that is transfered!!
         pose.position.x = position_box.x();
         pose.position.y = position_box.y();
         pose.position.z = position_box.z();

          // pose.orientation.w = orientation_box.w();
          // pose.orientation.x = orientation_box.x();
          // pose.orientation.y = orientation_box.y();
          // pose.orientation.z = orientation_box.z();

          pose.orientation.w = 1.0;
          pose.orientation.x = 0.0;
          pose.orientation.y = 0.0;
          pose.orientation.z = 0.0;

        std::stringstream s;
        s << "add_point_button";
        server->setPose(s.str(),pose);
        server->applyChanges();
        //changeMarkerControlAndPose( feedback->marker_name.c_str(),false,false,pose);
        ROS_INFO("Cube position is updated, %f:",position_box.x());
        //Q_EMIT point_pose_updated_RViz(marker_name, position,orientation);

      }
     else
     {
      int index = atoi(marker_name);
      // tf::Vector3 fb_pos(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
      // tf::Quaternion fb_orient(feedback->pose.orientation.x,feedback->pose.orientation.y,feedback->pose.orientation.z,feedback->pose.orientation.w);

      if ( index > positions.size() )
      {
        return;
      }
  
      positions[index-1] = position;
      orientations[index-1] = orientation;
      geometry_msgs::Pose pose;

      //pose = feedback->pose;
      pose.position.x = positions[index-1].x();
      pose.position.y = positions[index-1].y();
      pose.position.z = positions[index-1].z();

      pose.orientation.w = orientations[index-1].w();
      pose.orientation.x = orientations[index-1].x();
      pose.orientation.y = orientations[index-1].y();
      pose.orientation.z = orientations[index-1].z();

      std::stringstream s;
      s << index;
      server->setPose(s.str(),pose);
      server->applyChanges();
      ROS_INFO_STREAM("Arrow: "<<index <<"; position is updated to:"<<pose.position.x<<" , marker stringname: "<<s.str()<<"; \n");
      //Q_EMIT point_pose_updated_RViz(marker_name, position,orientation);
      
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

void AddWayPoint::makeArrow(const tf::Vector3& point_pos,const tf::Quaternion& point_orient, int count_arrow)//
{
        InteractiveMarker int_marker;
        int_marker.header.frame_id = "/point_pos";

        //static set of the size of the arrow
        int_marker.scale =0.4;

        //move the marker according to the clicked location, there is a problem with this implementation
        tf::pointTFToMsg(point_pos,int_marker.pose.position);
        tf::quaternionTFToMsg(point_orient,int_marker.pose.orientation);

        ROS_INFO("begining of make arrow function, count is:%d, positions count:%ld, point position at X:%f \n",count,positions.size(),point_pos.x());

        //check if we have arrow at that location,if not found then add it
        std::vector<tf::Vector3 >::iterator it_pos  =std::find((positions.begin()),(positions.end()),point_pos);
        std::vector<tf::Quaternion >::iterator it_orient  =std::find((orientations.begin()),orientations.end(),point_orient);

        //print vector before each arrow create just for debuggin remove it later
        for( int i=0;i<positions.size();i++)
            ROS_INFO_STREAM( "vecotr at start: \n"<<"x:"<< positions[i].x()<<"; " << positions[i].y()<< "; "<<positions[i].z()<<";\n");

        //check the positions and orientations vector if they are emtpy. If they are we have our first marker. Deleted the check of orientations just experimental
        if (positions.empty() ) //&& orientations.empty()
        {

            ROS_INFO("Adding first arrow!");
            count_arrow++;
            count=count_arrow;

            positions.push_back( point_pos );
            orientations.push_back( point_orient );
            Q_EMIT addPointFrom_RViz(point_pos,point_orient,count);


        }

        else if ((it_pos == positions.end()) && (point_pos!=positions[count_arrow]))
        {

           //print vector before each arrow create just for debuggin remove it later
           for( int i=0;i<positions.size();i++)
            ROS_INFO_STREAM( "vecotr at check if duplicates: \n"<<"x:"<< positions[i].x()<<"; " << positions[i].y()<< "; "<<positions[i].z()<<";\n");

            count_arrow++;
            count=count_arrow;

            positions.push_back( point_pos );
            orientations.push_back( point_orient );

          ROS_INFO("Adding new arrow!");
          Q_EMIT addPointFrom_RViz(point_pos,point_orient,count);
        }
    else
    {
            //if we have arrow, ignore adding new one and inform the user that there is arrow (waypoint at that location)
            ROS_INFO("There is already a arrow at that location, can't add new one!!");
    }


        std::stringstream s;
        s << count_arrow;
        ROS_INFO("end of make arrow, count is:%d, positions count:%ld",count,positions.size());
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

    for( int i=0;i<positions.size();i++)
            ROS_INFO_STREAM( "vecotr before delete: \n"<<"x:"<< positions[i].x()<<"; " << positions[i].y()<< "; "<<positions[i].z()<<";\n");


    //get the index of the selected marker
    int index = atoi( marker_name.c_str() );
    server->erase(marker_name.c_str());
    positions.erase (positions.begin()+index-1);
    orientations.erase (orientations.begin()+index-1);

    for( int i=0;i<positions.size();i++)
    ROS_INFO_STREAM( "Vector name:"<<marker_name<<"; vecotr after delete: \n"<<"x:"<< positions[i].x()<<"; " << positions[i].y()<< "; "<<positions[i].z()<<";\n");
            //InteractiveMarker int_marker;
           for(int i=index+1; i<=count;i++)
           {
              std::stringstream s;
              s << i;
              server->erase(s.str());
              makeArrow(positions[i-2],orientations[i-2],(i-1));
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
        int_marker.header.frame_id = "/point_pos";
        int_marker.scale = 0.3;

        int_marker.pose.position.x = 0.0;
        int_marker.pose.position.y = 0.0;
        int_marker.pose.position.z = 0.0;

        int_marker.description = "Interaction Marker";

        //positions.push_back( tf::Vector3(0.0,0.0,0.0) );
        orientation_box = tf::Quaternion(0.0,0.0,0.0,1.0);
        position_box = tf::Vector3(0.0,0.0,0.0);

        //button like interactive marker. Detect when we have left click with the mouse and add new arrow then
        int_marker.name = "add_point_button";

        makeBoxControl(int_marker);

        server->insert( int_marker);
        //add interaction feedback to the markers
        server->setCallback( int_marker.name, boost::bind( &AddWayPoint::processFeedback, this, _1 )); 
}

}//end of namespace for add_way_point

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(moveit_cartesian_planner::AddWayPoint,rviz::Panel )