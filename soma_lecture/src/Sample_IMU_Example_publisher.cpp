#include "ros/ros.h"

#include <random>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

geometry_msgs::Quaternion make_quaternion();

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Sample_IMU_Example_publisher");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 3);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    sensor_msgs::Imu msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "imu";

    msg.orientation = make_quaternion();

    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

geometry_msgs::Quaternion make_quaternion()
{
  geometry_msgs::Quaternion quat;

  std::random_device seed_gen;
  std::mt19937 engine(seed_gen());
  std::uniform_real_distribution<> dist(-3.1415/4.0, 0);

  tf2::Quaternion _quat;
  _quat.setRPY(0, dist(engine), 0); //roll, pitch, yaw [radian]

  quat = tf2::toMsg(_quat);

  return quat;
}
