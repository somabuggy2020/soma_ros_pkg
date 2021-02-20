#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

void callback(nav_msgs::Odometry::ConstPtr &odom)
{
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "behavior");
  ros::NodeHandle nh;

  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 3, callback);

  ros::spin();
  return 0;
}