#include "ros/ros.h"

#include <random>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float32MultiArray.h>

std_msgs::Float32MultiArray rpy_ary;

void imu_callback(const sensor_msgs::Imu &imu_data);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Sample_IMU_Example_publisher");

  ros::NodeHandle nh;
  rpy_ary.data.resize(3);
  ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("rpy", 3);
  ros::Subscriber sub = nh.subscribe("imu/data", 3, imu_callback);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    pub.publish(rpy_ary);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

void imu_callback(const sensor_msgs::Imu &imu_data)
{
  tf2::Quaternion quat_imu;
  tf2::fromMsg(imu_data.orientation, quat_imu);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat_imu).getRPY(roll, pitch, yaw);
  // tf2::Matrix3x3(quat_imu).getRPY(rpy_ary.data[0], rpy_ary.data[1], rpy_ary.data[2]);

  rpy_ary.data[0] = (float)roll;
  rpy_ary.data[1] = (float)pitch;
  rpy_ary.data[2] = (float)yaw;
}
