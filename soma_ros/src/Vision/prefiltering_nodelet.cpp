#include <string>

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace soma_vision
{
class PrefilteringNodelet : public nodelet::Nodelet
{
public:
  typedef pcl::PointXYZRGB PointT;

  PrefilteringNodelet() {}
  virtual ~PrefilteringNodelet() {}

  virtual void onInit()
  {
    nh = getNodeHandle();
    pnh = getPrivateNodeHandle();

    input_points_sub = nh.subscribe("input_points",
                                    3,
                                    &PrefilteringNodelet::points_callback,
                                    this);

    output_points_pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_points", 3);

    //distance filter parameter
    use_distance_filter = pnh.param<bool>("use_distance_filter", true);
    distance_near_thresh = pnh.param<double>("distance_near_thresh", 1.0);
    distance_far_thresh = pnh.param<double>("distance_far_thresh", 100.0);

    downsample_leaf_size = pnh.param<double>("down_sample_leaf_size", 0.1);

    sor_k = pnh.param<int>("sor_k", 20);
    sor_stddev = pnh.param<double>("sor_stddev", 1.0);
  }

private:
  void points_callback(pcl::PointCloud<PointT>::ConstPtr input)
  {
    //if point cloud empty
    if (input->empty())
    {
      return;
    }

    pcl::PointCloud<PointT>::ConstPtr filtered = distance_filter(input);
    filtered = downsample(filtered);
    filtered = sor(filtered);

    output_points_pub.publish(filtered);
  }

  //--------------------------------------------------
  //Distance filter function
  //--------------------------------------------------
  pcl::PointCloud<PointT>::ConstPtr distance_filter(
      const pcl::PointCloud<PointT>::ConstPtr &src) const
  {
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    filtered->reserve(src->size());

    if (!use_distance_filter)
    {
      pcl::copyPointCloud(*src, *filtered);
      return filtered;
    }

    std::copy_if(src->begin(), src->end(),
                 std::back_inserter(filtered->points), [&](const PointT &p) {
      double d = p.getVector3fMap().norm();
      return d > distance_near_thresh && d < distance_far_thresh; });

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->header = src->header;
    return filtered;
  }

  //--------------------------------------------------
  //Down sampling function
  //--------------------------------------------------
  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr &src) const
  {
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());

    pcl::VoxelGrid<PointT> filter;
    filter.setInputCloud(src);
    filter.setLeafSize(downsample_leaf_size, downsample_leaf_size, downsample_leaf_size);
    filter.filter(*filtered);
    filtered->header = src->header;

    return filtered;
  }

  //--------------------------------------------------
  //Statistical Outlier Removal function
  //--------------------------------------------------
  pcl::PointCloud<PointT>::ConstPtr sor(const pcl::PointCloud<PointT>::ConstPtr &src) const
  {
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());

    pcl::StatisticalOutlierRemoval<PointT> filter;
    filter.setInputCloud(src);
    filter.setMeanK(sor_k);
    filter.setStddevMulThresh(sor_stddev);
    filter.filter(*filtered);
    return filtered;
  }

private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;

  ros::Subscriber input_points_sub;
  ros::Publisher output_points_pub;
  tf::TransformListener tf_listener;

  bool use_distance_filter;
  double distance_near_thresh;
  double distance_far_thresh;

  double downsample_leaf_size;

  int sor_k;
  double sor_stddev;
};
} // namespace soma_vision

PLUGINLIB_EXPORT_CLASS(soma_vision::PrefilteringNodelet, nodelet::Nodelet);
