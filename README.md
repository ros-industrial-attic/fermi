fermi
=====

Google Summer of Code Project: Cartesian Path Planner MoveIt Plug-in

[Enrico Fermi](http://en.wikipedia.org/wiki/Enrico_Fermi): _Enrico Fermi (Italian: [enˈri.ko ˈfeɾ.mi]; 29 September 1901 – 28 November 1954) was an Italian physicist, best known for his work on Chicago Pile-1 (the first nuclear reactor), and for his contributions to the development of quantum theory, nuclear and particle physics, and statistical mechanics. He is one of the men referred to as the "father of the atomic bomb". Fermi held several patents related to the use of nuclear power, and was awarded the 1938 Nobel Prize in Physics for his work on induced radioactivity by neutron bombardment and the discovery of transuranic elements. He was widely regarded as one of the very few physicists to excel both theoretically and experimentally._

====
##1. First steps of development:
The idea behind the initial push up is to create two types of interactive markers:
  - The cube(box) which acts as a pointer which the user can drag around the RViz enviroment. Fruthermore by clicking on the box another interactive marker a sphere is added to the RViz enviroment
  - The sphere should be the way point for the cartesian trajectory planning. The user can also move the sphere around the RViz enviroment
  - The RQT UI should communicate simultaniously with the RViz enviroment and the User can change the state of a marker either through RViz or the RQT UI 

##2. Things to do, figure out in short term:
   - [ ] Clean all the Ogre dependencies
   - [ ] User can remove selected sphere
   - [ ] Change the AddWayPoint inheritance from Tool to Panel or Display.
   - [ ] User Interaction simultaniously between RViz and RQT dockable widged
   - [ ] Trace bugs and fix them...


##3. How to run the code
Compile with catkin
Don't forget to source the new setup.*sh file

```
$ source devel/setup.bash
```

###3.1 Run the following
```
$roscore
```
###3.2 In new terminal run
```
$rosrun rviz rviz
```

If everything worked out weel you will be able to add the new plugin (AddWayPoint) to the RViz enviroment, also add InteractiveMarkers to the Displays panel set update topic to /Sphere/update. Now you can play around with the cube and sphere markers.
Let me know of possible changes, code improvements and bugs.
