
##1. RViz and RQT User Interface:
There are two types of interactive markers:
  - The red arrow acts as a pointer which the user can move around the RViz enviroment. Fruthermore by clicking on the arrow another blue arrow is added to the RViz enviroment. This arrow acts as way-point for the Cartesian Planner.
  - The blue arrow is the  way point for the cartesian trajectory planning. The orientation of the arrow can be changed by holding the CTRL key and moving it with the mouse.
  - Each arrow has a menu where the user can either delete the selected arrow or it can change its position and orientation by using the 6DOF marker control.
  - When the way-point is out of the IK solution for the Robot the arrow changes its color from blue to yellow.
  - The RQT UI communicates simultaniously with the RViz enviroment and the User can change the state of a marker either through RViz or the RQT UI
  - TreeView displays all the added waypoints. The user can manipulate them directly in the TreeView and see their position and orientation of each waypoint.
  - The user can add new point or delete it through the RQT UI.
  - New tool component has been added for adding Arrows by using a mouse click

##2. How to run the code
Compile with catkin
Don't forget to source the new setup.*sh file

```
$ source devel/setup.bash
```


###2.1 Now you can try any preconfigured robot in MoveIt. For example run the following
```
$roslaunch fermi_sia20d_moveit_config moveit_planning_execution.launch
```

After this step you can add the new panel from the panel tab of the RViz enviroment. The Pannel needs to be added manually. Add InteractiveMarkers to the Displays panel set update topic to /moveit_cartesian_planner/update. Only after adding the MoveIt Cartesian Planner from the Panel Menu from the RViz enviroment the topic in the Interactive Markers visual will be visiable.

##3. Cartesian Impedance/Force control

As part of GSoC 2016, the plugin now supports Cartesian Impedance/Force Path Planning.
To use this feature you need to get the new developed ROS messages contained in the [majorana repository](https://github.com/ros-industrial-consortium/majorana).
The user can set the Cartesian Impedance/Force Control parameters via the UI. Depending on the particular robot driver or controller, the user can set the Impedance or Force Control parameters as well as switch between them during runtime.


For more detailed tutorial please refer to this [page](http://wiki.ros.org/moveit_cartesian_plan_plugin).
