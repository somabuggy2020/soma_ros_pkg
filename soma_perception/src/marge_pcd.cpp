#include "ros/ros.h"
#include "ros/time.h"
#include "nodelet/nodelet.h"
#include "std_msgs/String.h"
#include "pcl/common/io.h"
#include "pcl/point_cloud.h"
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <pluginlib/class_list_macros.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace sensor_msgs;
using namespace message_filters;

namespace soma_perception
{
  class marge_pcd : public nodelet::Nodelet
  {
  public:
    virtual void onInit();
    //void callback(const sensor_msgs::PointCloud2ConstPtr& pt_F, const sensor_msgs::PointCloud2ConstPtr& pt_B);
    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_input);

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub;

    ros::Subscriber sub_test;
    ros::Publisher pub_test;
  };

  void marge_pcd::onInit()
  {
    nh_ = getNodeHandle();

    message_filters::Subscriber<sensor_msgs::PointCloud2> point_F(nh_, "/vg_filter_F/output", 10);
    //message_filters::Subscriber<sensor_msgs::PointCloud2> point_B(nh_, "/vg_filter_B/output", 1);
    
    point_F.registerCallback(&marge_pcd::cloud_callback, this);

    //TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> sync(point_F, point_B, 10);

    //typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    //Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), point_F, point_B);

    //sync.registerCallback(boost::bind(&marge_pcd::callback, this, _1, _2));

    //pub = nh_.advertise<sensor_msgs::PointCloud2>("marge_pcd", 10);

    //sub_test = nh_.subscribe("/vg_filter_F/output", 1, &marge_pcd::cloud_callback, this);
    pub_test = nh_.advertise<sensor_msgs::PointCloud2>("marge_pcd_test", 10);
  }

  void marge_pcd::cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_input)
  {
    pcl::PCLPointCloud2 pcl_pt_test;
    sensor_msgs::PointCloud2 pub_pcd_test;
    pcl_conversions::toPCL(*cloud_input, pcl_pt_test);
    pcl_conversions::fromPCL(pcl_pt_test, pub_pcd_test);
    //pub_test.publish(pub_pcd_test);
    pub_test.publish(cloud_input);

  }

  //void marge_pcd::callback(const sensor_msgs::PointCloud2ConstPtr& pt_F, const sensor_msgs::PointCloud2ConstPtr& pt_B)
  //{
  //  pcl::PCLPointCloud2 pcl_pt_F;
  //  pcl::PCLPointCloud2 pcl_pt_B;
  //  pcl::PCLPointCloud2 pcl_pub_pcd;
  //  sensor_msgs::PointCloud2 pub_pcd;
  //  //pub_pcd.header.frame_id = "soma_link";
  //  //pub_pcd.header.stamp = pt_F->header.stamp;
  //  //pub_pcd.header.stamp = ros::Time();
//
  //  pcl_conversions::toPCL(*pt_F, pcl_pt_F);
  //  pcl_conversions::toPCL(*pt_B, pcl_pt_B);
  //  pcl::concatenatePointCloud(pcl_pt_F, pcl_pt_B, pcl_pub_pcd);
  //  pcl_conversions::fromPCL(pcl_pub_pcd, pub_pcd);
//
  //  pub_test.publish(pt_F);
  //  pub.publish(pub_pcd);
  //}

}  // namespace plugin_lecture

PLUGINLIB_EXPORT_CLASS(soma_perception::marge_pcd, nodelet::Nodelet)