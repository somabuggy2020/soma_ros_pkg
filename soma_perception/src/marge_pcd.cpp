#include "ros/ros.h"
#include "ros/time.h"
#include "nodelet/nodelet.h"
#include "std_msgs/String.h"
#include "pcl/common/io.h"
#include "pcl/point_cloud.h"
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <pluginlib/class_list_macros.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using namespace sensor_msgs;
using namespace message_filters;

namespace soma_perception
{
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;

  class marge_pcd : public nodelet::Nodelet
  {
  public:
    virtual void onInit();
    void callback(const sensor_msgs::PointCloud2ConstPtr& pt_F, const sensor_msgs::PointCloud2ConstPtr& pt_B);

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub;

    message_filters::Subscriber<sensor_msgs::PointCloud2> *point_F_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *point_B_sub;
    message_filters::Synchronizer<MySyncPolicy> *sync;

    ros::Subscriber sub_test;
    ros::Publisher pub_test;
  };

  void marge_pcd::onInit()
  {
    nh_ = getNodeHandle();

    point_F_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/vg_filter_F/output", 10);
    point_B_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/vg_filter_B/output", 10);
    sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(100), *point_F_sub, *point_B_sub);

    sync->registerCallback(boost::bind(&marge_pcd::callback, this, _1, _2));

    pub = nh_.advertise<sensor_msgs::PointCloud2>("marge_pcd", 10);
  }

  void marge_pcd::callback(const sensor_msgs::PointCloud2ConstPtr& pt_F, const sensor_msgs::PointCloud2ConstPtr& pt_B)
  {
    pcl::PCLPointCloud2 pcl_pt_F;
    pcl::PCLPointCloud2 pcl_pt_B;
    pcl::PCLPointCloud2 pcl_pub_pcd;
    sensor_msgs::PointCloud2 pub_pcd;

    pcl_conversions::toPCL(*pt_F, pcl_pt_F);
    pcl_conversions::toPCL(*pt_B, pcl_pt_B);
    pcl::concatenatePointCloud(pcl_pt_F, pcl_pt_B, pcl_pub_pcd);
    pcl_conversions::fromPCL(pcl_pub_pcd, pub_pcd);

    pub_test.publish(pt_F);
    pub.publish(pub_pcd);
  }

}  // namespace plugin_lecture

PLUGINLIB_EXPORT_CLASS(soma_perception::marge_pcd, nodelet::Nodelet)