cmake_minimum_required(VERSION 2.8.3)

project(moveit_cartesian_plan_plugin)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  pluginlib
  roscpp
  rqt_gui
  rqt_gui_cpp
  rviz
  visualization_msgs
  moveit_ros_move_group
  moveit_ros_planning_interface
  tf
  tf_conversions
)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

set(moveit_cartesian_plan_plugin_SRCS
  src/add_way_point.cpp
  src/point_tree_item.cpp
  src/point_tree_model.cpp
  src/widgets/path_planning_widget.cpp
  src/generate_cartesian_path.cpp
  ${MOC_FILES}
)

set(moveit_cartesian_plan_plugin_HDRS
  include/moveit_cartesian_plan_plugin/add_way_point.hpp
  include/moveit_cartesian_plan_plugin/point_tree_item.hpp
  include/moveit_cartesian_plan_plugin/point_tree_model.hpp
  include/moveit_cartesian_plan_plugin/widgets/path_planning_widget.hpp
  include/moveit_cartesian_plan_plugin/generate_cartesian_path.hpp
)

set(moveit_cartesian_plan_plugin_UIS
  src/widgets/path_planning_widget.ui
)

set(moveit_cartesian_plan_plugin_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS ${moveit_cartesian_plan_plugin_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  interactive_markers
  pluginlib
  roscpp
  rqt_gui
  rviz
  rqt_gui_cpp
  visualization_msgs
)

qt4_wrap_cpp(moveit_cartesian_plan_plugin_MOCS ${moveit_cartesian_plan_plugin_HDRS})
qt4_wrap_ui(moveit_cartesian_plan_plugin_UIS_H ${moveit_cartesian_plan_plugin_UIS})

include_directories(${moveit_cartesian_plan_plugin_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${moveit_cartesian_plan_plugin_SRCS} ${moveit_cartesian_plan_plugin_MOCS} ${moveit_cartesian_plan_plugin_UIS_H}) # ${moveit_cartesian_plan_plugin_UIS_H}
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${moveit_move_group_interface} yaml-cpp)
## target_link_libraries(${PROJECT_NAME} yaml-cpp)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/moveit_cartesian_plan_plugin
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
