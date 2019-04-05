cmake_minimum_required(VERSION 3.5.1)
message("start building cartesian plugin")
add_compile_options(-std=c++11)

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
  tf
  tf_conversions
  moveit_ros_move_group
  moveit_ros_planning_interface
)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

# Qt Stuff
if(rviz_QT_VERSION VERSION_LESS "5")
	find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtWidgets QtCore QtGui)
	include(${QT_USE_FILE})
  set(QT_LIBRARIES ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
else()
  find_package(Qt5Widgets REQUIRED)
  find_package(Qt5Concurrent REQUIRED)
	set(QT_LIBRARIES Qt5::Widgets Qt5::Concurrent)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)


# Support new yaml-cpp API.
find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

set(${PROJECT_NAME}_SRCS
  src/add_way_point.cpp
  src/point_tree_item.cpp
  src/point_tree_model.cpp
  src/widgets/path_planning_widget.cpp
  src/generate_cartesian_path.cpp
  ${MOC_FILES}
)

set(${PROJECT_NAME}_HDRS
  include/moveit_cartesian_plan_plugin/add_way_point.hpp
  include/moveit_cartesian_plan_plugin/point_tree_item.hpp
  include/moveit_cartesian_plan_plugin/point_tree_model.hpp
  include/moveit_cartesian_plan_plugin/widgets/path_planning_widget.hpp
  include/moveit_cartesian_plan_plugin/generate_cartesian_path.hpp
)

set(moveit_cartesian_plan_plugin_UIS
  src/widgets/path_planning_widget.ui
)

set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
  include
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}"
)
if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
 endif()



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

# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")

if(rviz_QT_VERSION VERSION_LESS "5")
    message("Wrapping qt4 cpp")
    qt4_wrap_cpp(moveit_cartesian_plan_plugin_MOCS ${moveit_cartesian_plan_plugin_HDRS})
    message("Wrapping qt4 ui")
    qt4_wrap_ui(moveit_cartesian_plan_plugin_UIS_H ${moveit_cartesian_plan_plugin_UIS})
else()
   message("Wrapping qt5 cpp")
   qt5_wrap_cpp(moveit_cartesian_plan_plugin_MOCS ${moveit_cartesian_plan_plugin_HDRS})
   message("Wrapping qt5 ui")
   qt5_wrap_ui(moveit_cartesian_plan_plugin_UIS_H ${moveit_cartesian_plan_plugin_UIS})
endif()

set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

include_directories(${moveit_cartesian_plan_plugin_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${moveit_cartesian_plan_plugin_SRCS} ${moveit_cartesian_plan_plugin_MOCS} ${moveit_cartesian_plan_plugin_UIS_H}) # ${moveit_cartesian_plan_plugin_UIS_H}
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${moveit_move_group_interface}
  yaml-cpp
  ${Boost_LIBRARIES}
  ${QT_LIBRARIES})

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
