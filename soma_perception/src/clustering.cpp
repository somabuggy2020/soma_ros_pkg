#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace soma_perception
{
class EuclideanClustering : public nodelet::Nodelet
{
public:
	virtual void onInit();
	void call_back();
	
private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;

	ros::Subscriber input_points_sub;

	pcl::search::KdTree<PointT>::Ptr tree_;
  pcl::EuclideanClusterExtraction<PointT> ec_;
  ros::Publisher pub_clusters_;
};



} // namespace soma_perception

PLUGINLIB_EXPORT_CLASS(soma_perception::EuclideanClustering, nodelet::Nodelet);