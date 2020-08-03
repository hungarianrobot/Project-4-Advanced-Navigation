#include "ros/ros.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "tf/tf.h"


// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the nav_goals node
  ros::init(argc, argv, "nav_goals");

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

  // Define a list of positions and orientations for the robot to reach
  float waypoints[4][3] = { 
                            {-4.25,  0.4,  3.14}, 
                            {-4.25, -4.5,  0.0},
                            { 6.5,  -4.5,  1.57},
                            { 6.5,   3.0, -1.57}  
                          };

  int num_points = 4;

  for (int i=0; i < num_points; i++){

      goal.target_pose.pose.position.x = waypoints[i][0];
      goal.target_pose.pose.position.y = waypoints[i][1];
      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(waypoints[i][2]);

       // Send the goal position and orientation for the robot to reach
      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      // Wait an infinite time for the results
      ac.waitForResult();

      // Check if the robot reached its goal
      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Goal reached!");
      else
        ROS_INFO("Failed to reach goal for some reason");
      // Wait 1 second before moving to the next goal
      ros::Duration(1.0).sleep();
  }

  return 0;
} 
