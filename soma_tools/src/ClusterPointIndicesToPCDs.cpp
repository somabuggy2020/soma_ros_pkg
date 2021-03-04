#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/io.h>
#include <jsk_recognition_utils/pcl_conversion_util.h>
// message filters
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// massages
#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>

namespace soma_tools
{
  class ClusterPointIndicesToPCDs : public nodelet::Nodelet
  {
  public:
    ClusterPointIndicesToPCDs(){}; //constructor

  private:
    virtual void onInit();
    void _callback_points(const sensor_msgs::PointCloud2::ConstPtr &points);
    void _callback_indices(const jsk_recognition_msgs::ClusterPointIndices::ConstPtr &indices);
    void callback(const sensor_msgs::PointCloud2ConstPtr &cloud,
                  const jsk_recognition_msgs::ClusterPointIndicesConstPtr &cluster_indices);

  private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    ros::Subscriber points_sub;
    ros::Subscriber indices_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *input_cloud_sub;
    message_filters::Subscriber<jsk_recognition_msgs::ClusterPointIndices> *cluster_indices_sub;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, jsk_recognition_msgs::ClusterPointIndices> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> *sync;

    std::string prefix_;
  };
  //
  //
  //
  void ClusterPointIndicesToPCDs::onInit()
  {
    NODELET_INFO("call:onInit");

    nh = getNodeHandle();
    pnh = getPrivateNodeHandle();

    // points_sub = nh.subscribe<sensor_msgs::PointCloud2>("input_points", 1,
    //                                                     &ClusterPointIndicesToPCDs::_callback_points, this);
    // indices_sub = nh.subscribe<jsk_recognition_msgs::ClusterPointIndices>("target_indices", 1,
    //                                                                       &ClusterPointIndicesToPCDs::_callback_indices, this);

    input_cloud_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "input_points", 1);
    cluster_indices_sub = new message_filters::Subscriber<jsk_recognition_msgs::ClusterPointIndices>(nh, "target_indices", 1);

    sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *input_cloud_sub, *cluster_indices_sub);
    sync->registerCallback(boost::bind(&ClusterPointIndicesToPCDs::callback, this, _1, _2));

    prefix_ = pnh.param<std::string>("prefix", "/home/hayashi/");

    return;
  }
  //
  //
  //
  void ClusterPointIndicesToPCDs::_callback_points(const sensor_msgs::PointCloud2::ConstPtr &points)
  {
    NODELET_INFO("points callback : %f", points->header.stamp.toSec());
  }
  void ClusterPointIndicesToPCDs::_callback_indices(const jsk_recognition_msgs::ClusterPointIndices::ConstPtr &indices)
  {
    NODELET_INFO("indices callback : %f", indices->header.stamp.toSec());
  }
  void ClusterPointIndicesToPCDs::callback(const sensor_msgs::PointCloud2::ConstPtr &points,
                                           const jsk_recognition_msgs::ClusterPointIndices::ConstPtr &indices)
  {
    NODELET_INFO("Async callback");
    NODELET_INFO("Num:%d", (int)indices->cluster_indices.size());

    pcl::ExtractIndices<pcl::PointXYZRGB> extractor;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<pcl::IndicesPtr> converted_indices;
    pcl::fromROSMsg(*points, *cloud);

    pcl::PCDWriter writer;

    extractor.setInputCloud(cloud);
    for (size_t i = 0; i < indices->cluster_indices.size(); i++)
    {
      NODELET_INFO("Cluster%02d : %d", (int)i, (int)indices->cluster_indices[i].indices.size());

      pcl::IndicesPtr vindices;
      vindices.reset(new std::vector<int>(indices->cluster_indices[i].indices));
      converted_indices.push_back(vindices);
      extractor.setIndices(converted_indices[i]);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr segment(new pcl::PointCloud<pcl::PointXYZRGB>());
      extractor.filter(*segment);

      std::stringstream ss;
      ss << prefix_;
      ss << points->header.stamp;
      ss << "_";
      ss << (int)i;
      ss << ".pcd";
      NODELET_INFO("save to : %s", ss.str().c_str());
      writer.writeASCII(ss.str(), *segment);
    }

    NODELET_INFO("Fin.");
  }

} // namespace soma_tools

PLUGINLIB_EXPORT_CLASS(soma_tools::ClusterPointIndicesToPCDs, nodelet::Nodelet)