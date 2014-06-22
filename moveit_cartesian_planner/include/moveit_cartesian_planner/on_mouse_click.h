#ifndef on_mouse_click__H
#define on_mouse_click__H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <ros/node_handle.h>
# include <ros/publisher.h>

# include "rviz/tool.h"

# include <QCursor>
# include <QObject>
#endif

namespace rviz
{
  class StringProperty;
  class BoolProperty;
}
using namespace rviz;
namespace moveit_cartesian_planner
{
//! The Point Tool allows the user to click on a point which
//! gets published as a PointStamped message.
class OnMouseClick: public Tool
{
  Q_OBJECT
public:
  OnMouseClick();
  virtual ~OnMouseClick();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent( ViewportMouseEvent& event );

public Q_SLOTS:

  void updateTopic();
  void updateAutoDeactivate();

protected:
  QCursor std_cursor_;
  QCursor hit_cursor_;

  ros::NodeHandle nh_;
  ros::Publisher pub_;

  StringProperty* topic_property_;
  BoolProperty* auto_deactivate_property_;
};

}

#endif

