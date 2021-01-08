#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>

#include <random>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <boost/shared_ptr.hpp>

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointN;

void make_cube_pointcloud(pcl::PointCloud<PointT>::Ptr cloud);
void estimation_normal(pcl::PointCloud<PointT>::Ptr input, 
                        pcl::PointCloud<PointN>::Ptr output);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Sample_IMU_publisher");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 3);
  ros::Publisher normal_pub = nh.advertise<sensor_msgs::PointCloud2>("normal_cloud", 3);

  ros::Rate loop_rate(15);

  pcl::PointCloud<PointT>::Ptr cloud;
  cloud.reset(new pcl::PointCloud<PointT>);

  while (ros::ok())
  {
    cloud->clear();

    make_cube_pointcloud(cloud);

    pcl::PointCloud<PointN>::Ptr normal_output(new pcl::PointCloud<PointN>);
    estimation_normal(cloud, normal_output);

    pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
    cloud->header.frame_id = "sensor_link";
    pub.publish(cloud);

    pcl_conversions::toPCL(ros::Time::now(), normal_output->header.stamp);
    normal_output->header.frame_id = "sensor_link";
    normal_pub.publish(normal_output);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

void make_cube_pointcloud(pcl::PointCloud<PointT>::Ptr cloud)
{
  std::random_device seed_gen;
  std::mt19937 engine(seed_gen());
  std::uniform_real_distribution<> distX(1.0, 2.0);
  std::uniform_real_distribution<> distY(-0.5, 0.5);
  std::uniform_real_distribution<> distZ(-0.5, 0.5);

  int N = 15000;
  cloud->clear();
  for(int i = 0; i < N; i++){
    PointT tmp;
    tmp.x = (float)distX(engine);
    tmp.y = (float)distY(engine);
    tmp.z = (float)distZ(engine);
    cloud->push_back(tmp);
  }

  return;
}

void estimation_normal(pcl::PointCloud<PointT>::Ptr input, 
                        pcl::PointCloud<PointN>::Ptr output)
{
  pcl::NormalEstimationOMP<PointT, pcl::Normal> impl;
  impl.setInputCloud(input);
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  impl.setSearchMethod(tree);
  impl.setRadiusSearch(0.1);
  pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
  impl.compute(*normal_cloud);

  output->points.resize(input->points.size());
  for (size_t i = 0; i < output->points.size(); i++) {
    PointN p;
    p.x = input->points[i].x;
    p.y = input->points[i].y;
    p.z = input->points[i].z;
    // p.rgb = input->points[i].rgb;
    p.normal_x = normal_cloud->points[i].normal_x;
    p.normal_y = normal_cloud->points[i].normal_y;
    p.normal_z = normal_cloud->points[i].normal_z;
    output->points[i] = p;
  }
}
