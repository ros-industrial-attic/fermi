
##1. RViz and RQT User Interface:
There are two types of interactive markers:
  - The red arrow which acts as a pointer which the user can move around the RViz enviroment. Fruthermore by clicking on the arrow another blue arrow is added to the RViz enviroment. This arrow acts as way-point for the Cartesian Planner.
  - The blue arrow is the  way point for the cartesian trajectory planning. The orientation of the arrow can be changed by holding the CTRL key and moving it with the mouse.
  - Each arrow has a menu where the user can either delete the selected arrow or it can change its position and orientation by using the 6DOF marker control.
  - The RQT UI communicates simultaniously with the RViz enviroment and the User can change the state of a marker either through RViz or the RQT UI 
  - TreeView represents displays all the added waypoints. The user can manipulate them directly in the TreeView and see their position and orientation of each waypoint.
  - The user can add new point or delete it through the RQT UI.
  - New tool component has been added for adding Arrows by using a mouse click

##2. Things to do, figure out in short term:
   - [x] Clean all the Ogre dependencies
   - [x] User can remove selected sphere
   - [x] Change the AddWayPoint inheritance from Tool to Panel or Display. The AddWay point has been changed to Panel.
   - [x] User Interaction simultaniously between RViz and RQT dockable widged
   - [ ] Extensive testing and Improvement of the the UI, RViz enviroment and the MoveIt interface. 
   - [ ] When user interacts with one way point the other way-points are not visiable.
   - [ ] Add KUKA LWR 4 model and test it
   - [ ] Trace bugs and fix them...


##3. How to run the code
Compile with catkin
Don't forget to source the new setup.*sh file

```
$ source devel/setup.bash
```

###3.1 Now you can try any preconfigured robot in MoveIt. For example run the following
```
$roslaunch sia20d_moveit_config demo.launch
```
or if you followed the ROS-Industrial tutorials for creating MoveIt package you could also run the following
```
$roslaunch sia20d_moveit_config moveit_planning_execution.launch
```

After this step you can add the new panel from the panel tab of the RViz enviroment. Add InteractiveMarkers to the Displays panel set update topic to /moveit_cartesian_planner/update.
