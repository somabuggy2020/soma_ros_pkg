#include <ros/ros.h>
#include <ros/time.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pluginlib/class_list_macros.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/exceptions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common.h>
#include <boost/shared_ptr.hpp>

#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <vector>
#include <math.h>

#define PI 3.14159265359

namespace soma_perception
{
  typedef pcl::PointXYZRGB PointT;

  class EuclideanClustering : public nodelet::Nodelet
  {
  public:
    EuclideanClustering() {}
    virtual ~EuclideanClustering() {}

    virtual void onInit()
    {
      NODELET_INFO("Initializing EuclideanClusteringNodelet");

      nh = getNodeHandle();
      pnh = getPrivateNodeHandle();

      initialize_params();
      //advertise
			cloud_transformed_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_transformed", 1);
      marker_pub = nh.advertise<visualization_msgs::MarkerArray>("marker", 1);
      //sub
      points_sub = nh.subscribe("camera_R/filtered", 1, &EuclideanClustering::cloud_callback, this);
		}

  private:
    /**
   * @brief initialize parameters
   */
    void initialize_params()
    {
      base_link_frame = pnh.param<std::string>("base_link", "soma_link");
    }

    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_input)
    {
      NODELET_INFO("cloud_callback function");

      if (cloud_input->data.size() == 0)
      {
        NODELET_WARN("empty point cloud");
        return;
      }

      NODELET_INFO("input point cloud size : %d", (int)cloud_input->data.size());

      //--------------------------------------------------
      // point cloud preprocessing
      // conversion and transform
      //--------------------------------------------------
      pcl::PointCloud<PointT>::Ptr cloud_raw;
      cloud_raw.reset(new pcl::PointCloud<PointT>());
      cloud_raw->clear();
      pcl::fromROSMsg(*cloud_input, *cloud_raw); //conversion

      pcl::PointCloud<PointT>::Ptr cloud_transformed;
      cloud_transformed.reset(new pcl::PointCloud<PointT>);
      cloud_transformed->clear();
      transform_pointCloud(cloud_raw, *cloud_transformed); //transform

      //--------------------------------------------------
      // euclidean cluster extraction process
      //--------------------------------------------------
      extraction_cluster(cloud_transformed);
      NODELET_INFO("total cluster points:%5d", (int)cloud_transformed->size());

			cloud_transformed->header.frame_id = base_link_frame;
      pcl_conversions::toPCL(ros::Time::now(), cloud_transformed->header.stamp);
      cloud_transformed_pub.publish(cloud_transformed);
    }

    void transform_pointCloud(pcl::PointCloud<PointT>::Ptr input,
                              pcl::PointCloud<PointT> &output)
    {
      if (!base_link_frame.empty())
      {
        if (!tf_listener.canTransform(base_link_frame, input->header.frame_id, ros::Time(0)))
        {
          NODELET_WARN("cannot transform %s->%s", input->header.frame_id.c_str(), base_link_frame.c_str());
          return;
        }

        //get transform
        tf::StampedTransform transform;
        tf_listener.waitForTransform(base_link_frame, input->header.frame_id, ros::Time(0), ros::Duration(2.0));
        tf_listener.lookupTransform(base_link_frame, input->header.frame_id, ros::Time(0), transform);
        //apply transform
        pcl_ros::transformPointCloud(*input, output, transform);
        output.header.frame_id = base_link_frame;
        output.header.stamp = input->header.stamp;
        NODELET_INFO("transformed point cloud (frame_id=%s)", output.header.frame_id.c_str());
      }
    }

    void extraction_cluster(pcl::PointCloud<PointT>::Ptr obstacle)
    {
      pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
      tree->setInputCloud(obstacle);
      std::vector<pcl::PointIndices> cluster_indices; //clusters
      pcl::EuclideanClusterExtraction<PointT> ec;
      ec.setClusterTolerance(0.1); //[m] if two point distance is less than the tlerance, it will be cluster
      ec.setMinClusterSize(1000);    //number of points in a cluster
      ec.setMaxClusterSize(25000); //about
      ec.setSearchMethod(tree);    //set kd-tree
      ec.setInputCloud(obstacle);  //set input cloud
      ec.extract(cluster_indices);
      NODELET_INFO("Cluster num: %2d", (int)cluster_indices.size());

      pcl::PointCloud<PointT>::Ptr tmp_obstacle(new pcl::PointCloud<PointT>());
      for (int i = 0; i < cluster_indices.size(); i++)
      {
        pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>);
        pcl::copyPointCloud(*obstacle, cluster_indices[i].indices, *tmp); 
        *tmp_obstacle += *tmp;
      }

      //marker
      visualization_msgs::MarkerArray marker_array;
      visualization_msgs::Marker marker;
      int marker_id = 0;
      //marker_array.markers.clear();
      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(), it_end = cluster_indices.end(); it != it_end; ++it, ++marker_id)
      {
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*obstacle, *it, min_pt, max_pt);
        Eigen::Vector4f cluster_size = max_pt - min_pt;
        if(cluster_size.x() > 0 && cluster_size.y() > 0 && cluster_size.z() > 0)
        {
          marker.header.frame_id = base_link_frame;
          marker.header.stamp = ros::Time();
          marker.ns = "marker";
          marker.id = marker_id;
          marker.type = visualization_msgs::Marker::CUBE;
          marker.action = visualization_msgs::Marker::ADD;
          marker.pose.position.x = cluster_size.x() / 2 + min_pt.x();
          marker.pose.position.y = cluster_size.y() / 2 + min_pt.y();
          marker.pose.position.z = cluster_size.z() / 2 + min_pt.z();
          marker.scale.x = cluster_size.x();
          marker.scale.y = cluster_size.y();
          marker.scale.z = cluster_size.z();
          marker.pose.orientation.x = 0.0;
          marker.pose.orientation.y = 0.0;
          marker.pose.orientation.z = 0.0;
          marker.pose.orientation.w = 1.0;
          marker.color.r = 0.0f;
          marker.color.g = 1.0f;
          marker.color.b = 0.0f;
          marker.color.a = 0.5f;
          marker.lifetime = ros::Duration(0.1);
          marker_array.markers.push_back(marker);
        }
      }

      if(marker_array.markers.empty() == false)
      {
        marker_pub.publish(marker_array);
      }

      obstacle->clear();
      pcl::copyPointCloud(*tmp_obstacle, *obstacle);
    }

  private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    tf::TransformListener tf_listener;

    //params
    std::string base_link_frame; //base_link frame id

    //subscribers
		ros::Subscriber points_sub;

    //publishers
		ros::Publisher cloud_transformed_pub;
    ros::Publisher marker_pub;
  };
} // namespace soma_perception

PLUGINLIB_EXPORT_CLASS(soma_perception::EuclideanClustering, nodelet::Nodelet);