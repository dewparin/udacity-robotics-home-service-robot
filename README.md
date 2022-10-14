## The home service robot

### List of packages used in the home service robot project.

`turtlebot_gazebo` package: this is the package for deploying the TurtleBot in the Gazebo world. The package provides launch files for deploying robot, SLAM, localization, and navigation. 

Launch files

   - `turtlebot_world.launch`: for deploying the TurtleBot in Gazebo.
   - `gmapping_demo.launch`: start the `slam_gmapping` node (from the `gmapping` package) to perform SLAM.
   - `amcl_demo.launch` -> launch `move_base` node, `map_server` node, and `amcl` node to perform localization and navigation from the known map.

`turtlebot_teleop` package with `keyboard_teleop.launch` file: provides control of the TurtleBot via keyboard.

`turtlebot_rviz_launchers` package with `view_navigation.launch` file: launches RViz with pre-configuration file.

`gmapping` package provides laser-based SLAM. This package is a dependency in the `gmapping_demo.launch` file from the `turtlebot_gazebo` package.

`add_markers` package: my custom implementation, which provides two nodes: 

 - `add_markers_timer`: publishes marker based on time.
 - `add_markers`: publishes markers based on the robot's location set by the `pick_objects` package. 

`pick_objects` package: my custom implementation moves the robot to a predefined pickup point and later to the drop-off point.

### Scripts

`test_slam.sh` This script is for testing SLAM. It deploys the robot in Gazebo world and starts `slam_gmapping`, RViz, and the keyboard teleop. After performing SLAM, you can use `map_server` to save the map.

`test_navigation.sh` This script is for testing robot navigation using the known map. It deploys the robot in Gazebo world, `map_server`, `amcl`, `move_base`, and RViz.

`add_markers.sh` This script demonstrates how to add virtual objects (markers) in the RViz.

`pick_objects.sh` This script moves the robot to the predefined pickup and drop-off points.

`home_service.sh` This script combines the `add_markers` node with the `pick_objects` node to demonstrate a home service robot that picks up an object from the pickup point and then delivers it to the drop-off point.