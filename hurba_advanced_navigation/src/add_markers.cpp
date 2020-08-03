#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include <math.h>

float odom_x = 0.0, odom_y = 0.0;

// get the robot's pose to global variables
void get_pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  ::odom_x = msg->pose.pose.position.x;
  ::odom_y = msg->pose.pose.position.y;
  // ROS_INFO("Robot's actual pose: %1.2f, %1.2f", ::odom_x, ::odom_y);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate rate(20);
  ros::Subscriber obom_sub = n.subscribe("/odom", 1, get_pose_cb);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Define the list of positions and orientations for marker
  float waypoints[4][3] = { 
                            {-4.25,  0.4,  3.14}, 
                            {-4.25, -4.5,  0.0},
                            { 6.5,  -4.5,  1.57},
                            { 6.5,   3.0, -1.57}  
                          };

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "add_markers";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = waypoints[0][0];
  marker.pose.position.y = waypoints[0][1];
  marker.pose.position.z = 0.5;
  marker.pose.orientation = tf::createQuaternionMsgFromYaw(waypoints[0][2]);


  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  float x_distance, y_distance;
  float pickup_range = 0.4;
  int i = 0;

  while (ros::ok())
  {
    // Publish the marker
    marker_pub.publish(marker);
    x_distance = fabs(waypoints[i][0] - odom_x);
    y_distance = fabs(waypoints[i][1] - odom_y);

    //ROS_INFO("Distance to pick-up target: %1.2f", sqrt(pow(x_distance, 2) + pow(y_distance, 2)));
    if( sqrt(pow(x_distance, 2) + pow(y_distance, 2)) < pickup_range ) {
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);
        ROS_INFO("Marker reached!");

        i++;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = waypoints[i][0];
        marker.pose.position.y = waypoints[i][1];
        marker.pose.position.z = 0.5;
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(waypoints[i][2]);
    }

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
} 
