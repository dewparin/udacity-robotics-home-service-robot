#!/bin/sh
WS="$(cd $(dirname "$0")/../..> /dev/null && pwd)"
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=\"$WS/src/map/base.world\"" &
sleep 15

xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 15

xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 15

xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch"
