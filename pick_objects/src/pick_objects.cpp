#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// declare the coordinates of interest
double Drop_off[3];
double Pick_up[3];

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");
    ros::NodeHandle n;
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

 //get pick up location .
          n.getParam("/Pick_up/x", Pick_up[0]);
          n.getParam("/Pick_up/y", Pick_up[1]);
          n.getParam("/Pick_up/w", Pick_up[2]);
          
  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = Pick_up[0];
  goal.target_pose.pose.position.y = Pick_up[1];
   goal.target_pose.pose.orientation.w = Pick_up[2];

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Robot successfully moved to the pick up zone");
  else
    ROS_INFO("Robot failed to move to the pick up zone");

   ros::Duration(5.0).sleep();
 
      //get drop off location .
          n.getParam("/Drop_off/x", Drop_off[0]);
          n.getParam("/Drop_off/y", Drop_off[1]);
          n.getParam("/Drop_off/w", Drop_off[2]);

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = Drop_off[0];
  goal.target_pose.pose.position.y = Drop_off[1];
   goal.target_pose.pose.orientation.w = Drop_off[2];

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Robot successfully moved to the drop off zone");
  else
    ROS_INFO("Robot failed to move to the drop off zone");

  return 0;
}
