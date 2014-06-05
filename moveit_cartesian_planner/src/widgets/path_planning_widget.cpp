#include <moveit_cartesian_planner/widgets/path_planning_widget.h>

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
		}

	}
}
