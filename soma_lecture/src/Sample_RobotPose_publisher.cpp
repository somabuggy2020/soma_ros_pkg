#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

void imu_callback(sensor_msgs::ImuConstPtr msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Sample_RobotPose_publisher");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/imu/data", 3, imu_callback);

  ros::spin();

  return 0;
}

void imu_callback(sensor_msgs::Imu::ConstPtr msg)
{
  static tf2_ros::TransformBroadcaster br;

  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "robot_link";

  transformStamped.transform.translation.x = 0;
  transformStamped.transform.translation.y = 0;
  transformStamped.transform.translation.z = 0;

  transformStamped.transform.rotation = msg->orientation;

  br.sendTransform(transformStamped);
}