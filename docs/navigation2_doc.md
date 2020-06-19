# ROS2 Navigation stack and study for use in our robot

Some key points one of our discussion meetings

* It is important to study navigation2 in detail, to see what is going in and what else is required, ex- behavior trees
* Having messages being exchanged and controllers in process would help us in finalizing the roadmap
* Even though we have Roswan drivers with us, it is not necessary to use previous r2c2 architecture
* See what else functionalities are provided with ros2 for use in future
* Good to look into everything and then decide !

ROS to ROS2 Navigation stack comparison

* Move_base splitted into multiple components
* Ported packages:
1. amcl: Ported to nav2_amcl
2. map_server: Ported to nav2_map_server
3. nav2_planner: Replaces global_planner
4. nav2_controller: Replaces local_planner
5. Navfn: Ported to nav2_navfn_planner
6. DWB: Replaces DWA and ported to ROS2 under
7. nav2_dwb_controller metapackage
8. nav_core: Ported as nav2_core with updates to interfaces
9. costmap_2d: Ported as nav2_costmap_2d

<image1>

* New packages:
1. nav2_bt_navigator: replaces move_base state machine
2. nav2_lifecycle_manager: Handles the server program lifecycles
3. nav2_waypoint_follower: Can take in many waypoints to execute a complex task through
4. nav2_system_tests: A set of integration tests for CI and basic tutorials in simulation
5. nav2_rviz_plugins: An rviz plugin to control the Navigation2 servers, command, cancel, and navigation with
6. nav2_experimental: Experimental (and incomplete) work for deep reinforement learning controllers
7. navigation2_behavior_trees: wrappers for the behavior tree library to call ROS action servers

<image2?

ROS2 goals (Foxy) (Can we use something?)
● Support for navigating multi-story buildings 

● Behavior tree plugin support 

● Add support for map zones  

● Traffic lanes and routes 

● Docking support 

● Voxel layer support 

● Support multiple sensor inputs 

● Support for changing maps 

● Waypoint follower demo 

* Ensure a stable, functional SLAM implementation is available for ROS 2 (outside the nav stack)

● Ensure a stable, functional IMU/Odom fusing solution (eg [Robot Localization] (https://github.com/cra-ros-pkg/robot_localization)) is available for ROS2. (outside the nav stack)

● Provide an option/example for launching nav2 using composed nodes 

● Add Auto-localization Action server #1254
