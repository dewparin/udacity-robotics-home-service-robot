#!/bin/sh

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=\"/home/workspace/catkin_ws/src/map/base.world\"" &
sleep 15

xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=\"/home/workspace/catkin_ws/src/map/map.yaml\" initial_pose_a:=-1.625" &
sleep 15

xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 15
