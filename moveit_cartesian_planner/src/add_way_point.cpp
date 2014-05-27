/*#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSphere.h>*/

#include <ros/console.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>

#include <moveit_cartesian_planner/add_way_point.h>

//#include <rviz/ogre_helpers/arrow.h>

#include <QStringList>

#include <algorithm> 

namespace moveit_cartesian_planner
{

AddWayPoint::AddWayPoint(QWidget* parent)
{

     setObjectName("MoveItPlanner");
     //shortcut_key_='i';
}

AddWayPoint::~AddWayPoint()
{
	/*for (unsigned i = 0; i < sphere_nodes_.size(); i++)
  {
    scene_manager_->destroySceneNode( sphere_nodes_[ i ]);
  }
  server.reset();*/
}

void AddWayPoint::onInitialize()
{
    server.reset( new interactive_markers::InteractiveMarkerServer("Sphere") );
    /*moving_sphere_node_->setVisible(false);
    ros::Duration(0.1).sleep();*/
    ROS_INFO("initializing..");
    //makeSphere();
    count = 0;
    makeBox();
    server->applyChanges();
    ROS_INFO("ready.");

  //ros::spin();
}

void AddWayPoint::activate()
{

  // if (moving_sphere_node_)
  // {
  //   moving_sphere_node_->setVisible(true);

  //   current_sphere_property_ = new rviz::VectorProperty("Sphere" + QString::number(sphere_nodes_.size()));
  //       //this property tells us that the sphere could not be manipulated. Change this later
  //   //current_sphere_property_->setReadOnly(true);
  //   getPropertyContainer()->addChild(current_sphere_property_);
  // }
}

void AddWayPoint::deactivate()
{

  // server.reset();
  // if (moving_sphere_node_)
  // {
  //   moving_sphere_node_->setVisible(false);
  //   delete current_sphere_property_;
  //   current_sphere_property_ = NULL;
  // }


}

int AddWayPoint::processMouseEvents( rviz::ViewportMouseEvent& event)
{

/* if (moving_sphere_node_)
  {
    return Render;
  }*/

  ROS_INFO("in the rviz mouse events..%d",event.x);
  //makeSphere();
  /*Ogre::Vector3 intersection;
  Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
  // if( rviz::getPointOnPlaneFromWindowXY( event.viewport,
  //                                        ground_plane,
  //                                        event.x, event.y, intersection ))
  // {
      moving_sphere_node_->setVisible(true);
      moving_sphere_node_->setPosition(intersection);
      current_sphere_property_->setVector(intersection);

      ROS_INFO("in the plane condition..");
      if (event.leftDown())
      {
        ROS_INFO("left mouse click..");
        makeSphere();
        current_sphere_property_ = NULL; //Drop the reference so the deactivate wont remove it
        return Render | Finished;
      }
 // }
  else
  {
    moving_sphere_node_->setVisible(false);
  }*/
    server->applyChanges();
  return 0;

}


void AddWayPoint::processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{

  std::ostringstream s;

  s<<"Feedback fom marker '" << feedback->marker_name << "' "
                             << "/ control '"<<feedback->control_name << "'";

  std::ostringstream mouse_point_ss;

  if(feedback->mouse_point_valid)
  {

    mouse_point_ss << "at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << "in frame " << feedback->header.frame_id;
  }

  


  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
    {
    ROS_INFO_STREAM(s.str()<<" :mouse position "<<mouse_point_ss.str()<<".");

    tf::Vector3 point_pos;
    //point_pos = tf::Vector3(feedback->mouse_point.x,feedback->mouse_point.y,feedback->mouse_point.z);
    point_pos = tf::Vector3(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
    //check if we have sphere at that location,if not found then
    if (std::find((positions.begin()+1),positions.end(),point_pos)==positions.end())
    {
      /* code */
    //if we dont have sphere to that location add one and increment sphere number
    makeSphere(point_pos,count);
    count++;
    }
    else
    {
      //if we have sphere, ignore adding new one and inform the user that there is sphere (waypoint at that location)
      ROS_INFO("There is already a sphere at that location, cant add new one!!");
    }


  
    break;
    }
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    {
       ROS_INFO("in the pose change condition..");
    
      tf::Vector3 fb_pos(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
      unsigned index = atoi( feedback->marker_name.c_str() );

      if ( index > positions.size() )
      {
        return;
      }
  
      positions[index] = fb_pos;
      //tf::Vector3 fb_delta = fb_pos - positions[index];
      geometry_msgs::Pose pose;
      pose.position.x = positions[index].x();
      pose.position.y = positions[index].y();
      pose.position.z = positions[index].z();

      std::stringstream s;
      s << index;
      server->setPose(s.str(),pose);

      break;
    }
  }
  server->applyChanges();
}


InteractiveMarkerControl& AddWayPoint::makeSphereControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  control.independent_marker_orientation = true;

  Marker marker;

  marker.type = Marker::SPHERE;
  marker.scale.x = msg.scale;
  marker.scale.y = msg.scale;
  marker.scale.z = msg.scale;

  //make the markers with interesting color
  marker.color.r = 0.10;
  marker.color.g = 0.20;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  marker.action = visualization_msgs::Marker::DELETE;

  control.markers.push_back( marker );
  msg.controls.push_back( control );

  return msg.controls.back();
}

void AddWayPoint::makeSphere(const tf::Vector3& point_pos, int count)
{
  int side_length = 5;
  float step = 1.0/ (float)side_length;
  //int count = 0;
  ///Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();


  positions.reserve( side_length*side_length*side_length );

        InteractiveMarker int_marker;
        int_marker.header.frame_id = "/base_link";
        int_marker.scale = step;

/*        int_marker.pose.position.x = point_pos(0);
        int_marker.pose.position.y = point_pos(1);
        int_marker.pose.position.z = point_pos(2);*/
        //move the marker according to the clicked location
        tf::pointTFToMsg(point_pos,int_marker.pose.position);

        positions.push_back( point_pos );

        //add name to the interactive marker, make a variable that increments count!!!
        //think of how to update the names when marker is deleted!!! :D
        std::stringstream s;
        s << count;
        int_marker.name = s.str();

        makeSphereControl(int_marker);

        server->insert( int_marker);
        //add interaction feedback to the markers
        server->setCallback( int_marker.name, boost::bind( &AddWayPoint::processFeedback, this, _1 )); 
}

InteractiveMarkerControl& AddWayPoint::makeBoxControl( InteractiveMarker &msg )
{

  InteractiveMarkerControl control;
  control.always_visible = true;
  //control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  control.independent_marker_orientation = true;
  control.name = "move";
  msg.controls.push_back( control );

  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.name = "button_interaction";



  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale*0.3;
  marker.scale.y = msg.scale*0.3;
  marker.scale.z = msg.scale*0.3;


  //make the markers with interesting color
  marker.color.r = 1.00;
  marker.color.g = 0.20;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  control.markers.push_back( marker );
  msg.controls.push_back( control );

  return msg.controls.back();
}

void AddWayPoint::makeBox()
{
  int side_length = 5;
  float step = 1.0/ (float)side_length;
  int count = 1;
  ///Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();


  //positions.reserve( side_length*side_length*side_length );

        InteractiveMarker int_marker;
        int_marker.header.frame_id = "/base_link";
        int_marker.scale = 0.8;

        int_marker.pose.position.x = 0.0;
        int_marker.pose.position.y = 0.0;
        int_marker.pose.position.z = 0.0;

        positions.push_back( tf::Vector3(0.0,0.0,0.0) );

        //button like interactive marker. Detect when we have left click with the mouse and add new sphere then
        int_marker.name = "add_point_button";

        makeBoxControl(int_marker);

        server->insert( int_marker);
        //add interaction feedback to the markers
        server->setCallback( int_marker.name, boost::bind( &AddWayPoint::processFeedback, this, _1 )); 
}


void AddWayPoint::save(rviz::Config config) const
{
	config.mapSetValue( "Class", getClassId() );

  // The top level of this tool's Config is a map, but our spheres
  // should go in a list, since they may or may not have unique keys.
  // Therefore we make a child of the map (``spheres_config``) to store
  // the list.
  rviz::Config spheres_config = config.mapMakeChild( "Sphere" );

  // To read the positions and names of the spheres, we loop over the
  // the children of our Property container:
  rviz::Property* container = getPropertyContainer();
  int num_children = container->numChildren();
  for( int i = 0; i < num_children; i++ )
  {
    rviz::Property* position_prop = container->childAt( i );
    // For each Property, we create a new Config object representing a
    // single sphere and append it to the Config list.
    rviz::Config spheres_config = spheres_config.listAppendNew();
    // Into the sphere's config we store its name:
    spheres_config.mapSetValue( "Name", position_prop->getName() );
    // ... and its position.
    position_prop->save( spheres_config );
  }

}

void AddWayPoint::load(const rviz::Config& config)
{
  // Here we get the "Spheres" sub-config from the tool config and loop over its entries:
  rviz::Config spheres_config = config.mapGetChild( "Spheres" );
  int num_spheres = spheres_config.listLength();
  for( int i = 0; i < num_spheres; i++ )
  {
    rviz::Config sphere_config = spheres_config.listChildAt( i );
    // At this point each ``sphere_config`` represents a single sphere.
    //
    // Here we provide a default name in case the name is not in the config file for some reason:
    QString name = "Sphere " + QString::number( i + 1 );
    // Then we use the convenience function mapGetString() to read the
    // name from ``sphere_config`` if it is there.  (If no "Name" entry
    // were present it would return false, but we don't care about
    // that because we have already set a default.)
    sphere_config.mapGetString( "Name", &name );
    // Given the name we can create an rviz::VectorProperty to display the position:
    rviz::VectorProperty* prop = new rviz::VectorProperty( name );
    // Then we just tell the property to read its contents from the config, and we've read all the data.
    prop->load( sphere_config );
    // We finish each sphere by marking it read-only (as discussed
    // above), adding it to the property container, and finally making
    // an actual visible sphere object in the 3D scene at the correct
    // position.
    prop->setReadOnly( true );
    getPropertyContainer()->addChild( prop );
    //makeSphere( );
  }
}

}//end of namespace for add_way_point

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(moveit_cartesian_planner::AddWayPoint,rviz::Tool )