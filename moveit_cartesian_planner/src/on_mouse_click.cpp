#include <OgreRay.h>
#include <OgreVector3.h>

#include "rviz/viewport_mouse_event.h"
#include "rviz/load_resource.h"
#include "rviz/render_panel.h"
#include "rviz/display_context.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/view_controller.h"

#include <moveit_cartesian_planner/on_mouse_click.h>

#include "rviz/properties/bool_property.h"
#include "rviz/properties/string_property.h"

#include <geometry_msgs/PointStamped.h>

#include <sstream>

namespace moveit_cartesian_planner
{

OnMouseClick::OnMouseClick()
  : Tool()
{
  topic_property_ = new StringProperty( "Topic", "/clicked_point",
                                        "The topic on which to publish points.",
                                        getPropertyContainer(), SLOT( updateTopic() ), this );

  auto_deactivate_property_ = new BoolProperty( "Single click", true,
                                                "Switch away from this tool after one click.",
                                                getPropertyContainer(), SLOT( updateAutoDeactivate() ), this );

  updateTopic();

  //shortcut key for activating the tool
  shortcut_key_ = 'a';

  
}

OnMouseClick::~OnMouseClick()
{

  //deactivate the tool
  shortcut_key_ = 'd';
}

void OnMouseClick::onInitialize()
{
  hit_cursor_ = cursor_;
  std_cursor_ = getDefaultCursor();
}

void OnMouseClick::activate()
{

  
}

void OnMouseClick::deactivate()
{
  
}

void OnMouseClick::updateTopic()
{
  pub_ = nh_.advertise<geometry_msgs::PointStamped>( topic_property_->getStdString(), 1 );
}

void OnMouseClick::updateAutoDeactivate()
{
}

int OnMouseClick::processMouseEvent( ViewportMouseEvent& event )
{
  int flags = 0;

  Ogre::Vector3 pos;
  bool success = context_->getSelectionManager()->get3DPoint( event.viewport, event.x, event.y, pos );
  setCursor( success ? hit_cursor_ : std_cursor_ );

  if ( success )
  {
    std::ostringstream s;
    s << "<b>Left-Click:</b> Select this point.";
    s.precision(3);
    s << " [" << pos.x << "," << pos.y << "," << pos.z << "]";
    setStatus( s.str().c_str() );

    if( event.leftUp() )
    {
      geometry_msgs::PointStamped ps;
      ps.point.x = pos.x;
      ps.point.y = pos.y;
      ps.point.z = pos.z;
      ROS_INFO("Position of the mouse at x:%f, at y:%f, at z:%f",pos.x,pos.y,pos.z);
      ps.header.frame_id = context_->getFixedFrame().toStdString();
      ps.header.stamp = ros::Time::now();
      pub_.publish( ps );

      if ( auto_deactivate_property_->getBool() )
      {
        flags |= Finished;
      }
    }
  }
  else
  {
    setStatus( "Move over an object to select the target point." );
  }

  return flags;
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(moveit_cartesian_planner::OnMouseClick,rviz::Tool )