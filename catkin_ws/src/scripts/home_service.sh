#!/bin/sh
WS="$(cd $(dirname "$0")/../..> /dev/null && pwd)"
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=\"$WS/src/map/base.world\"" &
sleep 15

xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=\"$WS/src/map/map.yaml\" initial_pose_a:=-1.625" &
sleep 15

xterm -e "rosrun rviz rviz -d $WS/src/rviz/home_robot.rviz" &
sleep 15

xterm -e "rosrun add_markers add_markers" &
sleep 5

xterm -e "rosrun pick_objects pick_objects"