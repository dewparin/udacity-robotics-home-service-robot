#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

double pickup_x = 0.55439;
double pickup_y = 2.7831;
double dropoff_x = -1.83;
double dropoff_y = -5.42;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");

  // Expose robot's state using paramerter server
  ros::NodeHandle nh;
  nh.setParam("robot_at_pickup", false);
  nh.setParam("robot_at_dropoff", false);
  nh.setParam("pickup_x", pickup_x);
  nh.setParam("pickup_y", pickup_y);
  nh.setParam("dropoff_x", dropoff_x);
  nh.setParam("dropoff_y", dropoff_y);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // goto pickup zone
  goal.target_pose.pose.position.x = pickup_x;
  goal.target_pose.pose.position.y = pickup_y;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pickup zone goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("The base has reached the pickup zone");
    nh.setParam("robot_at_pickup", true);
  } else {
    ROS_INFO("The base failed to move to the pickup zone");
  }

  ros::Duration(5).sleep();

  // goto drop-off zone
  goal.target_pose.pose.position.x = dropoff_x;
  goal.target_pose.pose.position.y = dropoff_y;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending drop-off zone goal");

  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("The base has reached the drop-off zone");
    nh.setParam("robot_at_dropoff", true);
  } else {
    ROS_INFO("The base failed to move to the drop-off zone");
  }

   // go back to origin 
  goal.target_pose.pose.position.x = 0.0;
  goal.target_pose.pose.position.y = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending origin point");

  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("The base has reached the origin point");
  } else {
    ROS_INFO("The base failed to move to the origin point");
  }

  return 0;
}