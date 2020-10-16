#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>

#include <random>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <boost/shared_ptr.hpp>

void make_cube_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Sample_IMU_publisher");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);

  ros::Rate loop_rate(2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

  while (ros::ok())
  {
    cloud->clear();

    make_cube_pointcloud(cloud);

    pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
    cloud->header.frame_id = "sensor_link";
    pub.publish(cloud);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

void make_cube_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  std::random_device seed_gen;
  std::mt19937 engine(seed_gen());
  std::uniform_real_distribution<> distX(1.0, 2.0);
  std::uniform_real_distribution<> distY(-0.5, 0.5);
  std::uniform_real_distribution<> distZ(-0.5, 0.5);

  int N = 10000;
  cloud->clear();
  for(int i = 0; i < N; i++){
    pcl::PointXYZ tmp;
    tmp.x = (float)distX(engine);
    tmp.y = (float)distY(engine);
    tmp.z = (float)distZ(engine);
    cloud->push_back(tmp);
  }

  return;
}
