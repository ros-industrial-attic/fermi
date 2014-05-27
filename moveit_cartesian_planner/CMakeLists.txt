cmake_minimum_required(VERSION 2.8.3)

project(moveit_cartesian_planner)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  pluginlib
  roscpp
  rqt_gui
  rqt_gui_cpp
  rviz
  visualization_msgs
)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})

## We also use Ogre
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
pkg_check_modules(OGRE OGRE)
include_directories( ${OGRE_INCLUDE_DIRS} )
link_directories( ${OGRE_LIBRARY_DIRS} )

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

set(moveit_cartesian_planner_SRCS
  src/add_way_point.cpp
)

set(moveit_cartesian_planner_HDRS
  include/moveit_cartesian_planner/add_way_point.h
)

set(moveit_cartesian_planner_UIS
  src/path_planner.ui
)

set(moveit_cartesian_planner_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS ${moveit_cartesian_planner_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  interactive_markers
  pluginlib
  roscpp
  rqt_gui
  rqt_gui_cpp
  rviz
  visualization_msgs
)
catkin_python_setup()

qt4_wrap_cpp(moveit_cartesian_planner_MOCS ${moveit_cartesian_planner_HDRS})
qt4_wrap_ui(moveit_cartesian_planner_UIS_H ${moveit_cartesian_planner_UIS})

include_directories(${moveit_cartesian_planner_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${moveit_cartesian_planner_SRCS} ${moveit_cartesian_planner_MOCS} ${moveit_cartesian_planner_UIS_H}) # ${moveit_cartesian_planner_UIS_H}
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${OGRE_LIBRARY_DIRS})

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/moveit_cartesian_planner
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
#)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
#add_executable(rqt_pkg src/my_plugin.cpp)
#target_link_libraries(rqt_pkg ${catkin_LIBRARIES})