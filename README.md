# HURBA: Advanced navigation tutorial IV.

[//]: # (Image References)

[image1]: ./documentation/gazebo.png "Gazebo"
[image2]: ./documentation/map.png "Map"
[image3]: ./documentation/frames.png "Frames"
[image4]: ./documentation/goals.png "Goals"
[image5]: ./documentation/waypoints.png "Waypoints"
[image6]: ./documentation/rosgraph_teleop.png "Teleop"
[image7]: ./documentation/rosgraph_navigation.png "Navigation"

### Dependencies:
- [ROS Melodic](http://wiki.ros.org/melodic "ROS Melodic")
- [Gazebo 9](http://wiki.ros.org/gazebo_ros_pkgs "Gazebo ROS package")
- [RViz](http://wiki.ros.org/rviz "RViz")
- [Teleop twist keyboard](http://wiki.ros.org/teleop_twist_keyboard "Teleop twist keyboard")
- [AMCL](http://wiki.ros.org/amcl "AMCL")
- [Map server](http://wiki.ros.org/map_server "Map server")
- [move_base](http://wiki.ros.org/move_base "move_base")
- [robot_pose_ekf](http://wiki.ros.org/robot_pose_ekf "robot_pose_ekf")
- [follow_waypoints](http://wiki.ros.org/follow_waypoints "follow_waypoints")
- [cob_base_velocity_smoother](http://wiki.ros.org/cob_base_velocity_smoother "cob_base_velocity_smoother")
- [teb_local_planner](http://wiki.ros.org/teb_local_planner "teb_local_planner")
- [sbpl_lattice_planner (optional)](http://wiki.ros.org/sbpl_lattice_planner "sbpl_lattice_planner")

### Project build instrctions:
1. Clone this repo inside the `src` folder of a catkin workspace:
`git clone https://github.com/hungarianrobot/Project-4-Advanced-Navigation`
2. Build workspace: `catkin_make`
3. Source environment: `source devel/setup.bash`
4. We'll use a simulated IMU and the robot_pose_ekf package for sensor fusion. robot_pose_ekf will provide transformation between odom frame and base_link so
[we have to disable the default transformation of Gazebo](https://answers.ros.org/question/229722/how-to-stop-gazebo-publishing-tf/ "disable tfs")!
Edit `/opt/ros/melodic/share/gazebo_ros/launch/empty_world.launch`:
```xml
<!-- start gazebo server-->
  <node name="gazebo" pkg="gazebo_ros" type="gzserver" respawn="false" output="screen" 
    args="$(arg command_arg1) $(arg command_arg2) $(arg command_arg3) $(arg world_name)">    
  <remap from="tf" to="gazebo_tf"/> 
</node>
```

Note: we can use the `roswtf` package anytime to debug tf issues! E.g. in this case this is the output of `roswtf`:

```bash
Found 2 error(s).

ERROR TF re-parenting contention:
 * reparenting of [base_footprint] to [odom_combined] by [/robot_pose_ekf]
 * reparenting of [base_footprint] to [odom] by [/gazebo]

ERROR TF multiple authority contention:
 * node [/robot_pose_ekf] publishing transform [base_footprint] with parent [odom_combined] already published by node [/gazebo]
 * node [/gazebo] publishing transform [base_footprint] with parent [odom] already published by node [/robot_pose_ekf]
```

### Test the simulation
1. Start the Gazebo simulation: `roslaunch hurba_advanced_navigation bringup.launch`
2. Start the teleop package: `roslaunch hurba_advanced_navigation teleop.launch`
3. Drive the omnidirectional robot inside the simulated environment.

![alt text][image1]

### Navigation launch files
There are 3 different navigation configuration with 3 different launchfiles in the project.
The difference among these navigation configurations is the used plugin for local and global planning.

##### navigation_basic.launch
- Global planner: [NavfnROS](http://wiki.ros.org/navfn "NavfnROS") 
- Local planner: [TrajectoryPlannerROS](http://wiki.ros.org/base_local_planner "TrajectoryPlannerROS")

##### navigation_advanced.launch
- Global planner: [GlobalPlanner](http://wiki.ros.org/global_planner "global_planner")
- Local planner: [TebLocalPlannerROS](http://wiki.ros.org/teb_local_planner "teb_local_planner")

##### navigation_lattice.launch
- Global planner: [SBPLLatticePlanner](http://wiki.ros.org/sbpl_lattice_planner "sbpl_lattice_planner")
- Local planner: [TebLocalPlannerROS](http://wiki.ros.org/teb_local_planner "teb_local_planner")

Lattice global planner is and advanced planner for robots that handles non-circular footprints and nonholonomic constraints using motion primitives. Motion primitives are short, kinematically feasible motions which form the basis of movements that can be performed by the robot platform. Search-based planners can generate paths from start to goal configurations by combining a series of these motion primitives. The result is a smooth kinematically feasible path for the robot to follow. At the same time it's really resource consuming, so I don't suggest to use it in embedded environment. For more details, you can visit the following link:
https://wiki.ros.org/Events/CoTeSys-ROS-School?action=AttachFile&do=get&target=robschooltutorial_oct10.pdf

#### Launch the navigation with any of the launchfiles from above:

1. Start the Gazebo simulation: `roslaunch hurba_advanced_navigation bringup.launch`
2. Start the navigation package: `roslaunch hurba_advanced_navigation navigation_XXX.launch`
3. Send a navigation goal and move_base will drive the robot to the desired location.

### Waypoint navigation

To send waypoints for the navigation we'll use RViz's 2D Pose Estimate button. Normally, this button sends an initial position for AMCL so we remapped it's functionality in the `bringup.launch` file in the following locations:

```xml
<node name="amcl" pkg="amcl" type="amcl" output="screen">
    <remap from="initialpose" to="initialpose_amcl"/>
...
```

```xml
<node pkg="follow_waypoints" type="follow_waypoints" name="follow_waypoints" output="screen">
    <param name="goal_frame_id" value="map"/>
    <remap from="initialpose" to="waypoint" />
</node>
```

```xml
<node name="rviz" pkg="rviz" type="rviz" respawn="false" args="-d $(find hurba_advanced_navigation)/rviz/navigation.rviz">
        <remap from="initialpose" to="waypoint" />
</node>
```

After remapping the signals new waypoints can be added with the 2D Pose Estimate button and small blue arrows will show the waypoints in RViz:

![alt text][image5]

When the waypoints are set their execution can be started with the following command:
`rostopic pub /path_ready std_msgs/Empty -1`


### Setting goals from code
We can send waypoints for the navigation stack from code, too. To send a predefined list of waypoints we can simply launch the `goals.launch` file. This launcgfile will start 2 ROS nodes:
- add_markers
- nav_goals

Both nodes uses the same array of waypoints:
```cpp
float waypoints[4][3] = { 
                          {-4.25,  0.4,  3.14}, 
                          {-4.25, -4.5,  0.0},
                          { 6.5,  -4.5,  1.57},
                          { 6.5,   3.0, -1.57}  
                        };
```
`nav_goals` node is sending waypoints for the navigation stack, after sending a goal it waits for the results from the navigation stack before it sends the next waypoint.
`add_markers` node creates blue cubes for RViz to indicate the waypoints.

![alt text][image4]


### TF tree and ROS graph of nodes:

#### TF tree of the project:
![alt text][image3]

#### rosgraph of teleoperation
![alt text][image6]

#### rosgraph of navigation
![alt text][image7]

### Project structure:
```bash
 tree -L 3
.
├── documentation
│   ├── frames.png
│   ├── gazebo.png
│   ├── goals.png
│   ├── map.png
│   ├── robschooltutorial_oct10.pdf
│   ├── rosgraph_navigation.png
│   ├── rosgraph_teleop.png
│   └── waypoints.png
├── hurba_advanced_navigation
│   ├── CMakeLists.txt
│   ├── config
│   │   ├── base_local_planner_params.yaml
│   │   ├── costmap_common_params.yaml
│   │   ├── global_costmap_params.yaml
│   │   ├── global_planner_params.yaml
│   │   ├── lattice_global_planner_params.yaml
│   │   ├── local_costmap_params.yaml
│   │   ├── pr2.mprim
│   │   └── teb_local_planner_params.yaml
│   ├── launch
│   │   ├── bringup.launch
│   │   ├── goals.launch
│   │   ├── navigation_advanced.launch
│   │   ├── navigation_basic.launch
│   │   ├── navigation_lattice.launch
│   │   ├── robot_description.launch
│   │   ├── teleop.launch
│   │   └── world.launch
│   ├── maps
│   │   ├── map.pgm
│   │   └── map.yaml
│   ├── meshes
│   │   ├── chassis.dae
│   │   ├── chassis.SLDPRT
│   │   ├── chassis.STEP
│   │   ├── hokuyo.dae
│   │   ├── wheel.dae
│   │   ├── wheel.SLDPRT
│   │   └── wheel.STEP
│   ├── package.xml
│   ├── rviz
│   │   ├── basic_view.rviz
│   │   └── navigation.rviz
│   ├── src
│   │   ├── add_markers.cpp
│   │   └── nav_goals.cpp
│   ├── urdf
│   │   ├── hurba_mecanum.gazebo
│   │   └── hurba_mecanum.xacro
│   └── worlds
│       ├── basic_world.world
│       ├── building.model
│       ├── building.world
│       └── empty.world
└── README.md
```

