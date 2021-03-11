#include <ros/ros.h>
#include <ros/time.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pluginlib/class_list_macros.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/exceptions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>

#include <boost/shared_ptr.hpp>

#include <string>
#include <vector>
#include <math.h>

namespace soma_perception
{
  typedef pcl::PointXYZRGB PointT;

  class DiameterEstimationNodelet : public nodelet::Nodelet
  {
  public:
    DiameterEstimationNodelet(){};
    virtual ~DiameterEstimationNodelet(){};

    virtual void onInit()
    {
      NODELET_INFO("Initializing DiameterEstimationNodelet");

      nh = getNodeHandle();
      pnh = getPrivateNodeHandle();

      pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_output", 3);
      points_sub = nh.subscribe("input_points",
      3, 
      &DiameterEstimationNodelet::callback, 
      this);

      base_link_frame = pnh.param<std::string>("base_link", "soma_link");
      distance_thres = pnh.param<double>("distance_thres", 0.01);

    }

    void callback(const sensor_msgs::PointCloud2ConstPtr &_input)
    {
      // NODELET_INFO("point size: %d", _input->data.size());

      pcl::PointCloud<PointT>::Ptr input(new pcl::PointCloud<PointT>());
      pcl::fromROSMsg(*_input, *input);

      //--------------------------------------------------
      // transform pointcloud
      //--------------------------------------------------
      pcl::PointCloud<PointT>::Ptr cloud_transformed(new pcl::PointCloud<PointT>());
      transform_pointCloud(input, *cloud_transformed);
      if(cloud_transformed->empty()) return;

      //--------------------------------------------------
      // estimate diameter
      //--------------------------------------------------
      

      //--------------------------------------------------
      // publish
      //--------------------------------------------------
    //   sensor_msgs::PointCloud2 pc_output;
    //   pcl::toROSMsg(*normal_output, pc_output);
    //   pc_output.header.frame_id = base_link_frame;
    //   NODELET_INFO("pub points size:%5d", (int)normal_output->size());
    //   pub.publish(pc_output);
    }

    void transform_pointCloud(pcl::PointCloud<PointT>::Ptr input,
                              pcl::PointCloud<PointT> &output)
    {
      if (!base_link_frame.empty())
      {
        // if (!tf_listener.canTransform(base_link_frame, input->header.frame_id, ros::Time(0)))
        // {
        //   NODELET_WARN("cannot transform %s->%s", input->header.frame_id.c_str(), base_link_frame.c_str());
        //   return;
        // }

        //get transform
        tf::StampedTransform transform;
        tf_listener.waitForTransform(base_link_frame, input->header.frame_id, ros::Time(0), ros::Duration(10.0));
        tf_listener.lookupTransform(base_link_frame, input->header.frame_id, ros::Time(0), transform);
        //apply transform
        pcl_ros::transformPointCloud(*input, output, transform);
        output.header.frame_id = base_link_frame;
        output.header.stamp = input->header.stamp;
        // NODELET_INFO("transformed point cloud (frame_id=%s)", output.header.frame_id.c_str());
      }
    }

    int segmentation(pcl::PointCloud<PointT>::Ptr input,
                     pcl::PointIndices &inliers,
                     pcl::ModelCoefficients &coeffs)
    {
      //そもそも入力点群数が少なすぎるとRANSACの計算ができない
      //RANSACのOptimizeCoefficientsをtrueにするときはたぶん10点以上ないとまずい
      if (input->size() < 10)
        return -1;

      //instance of RANSAC segmentation processing object
      pcl::SACSegmentation<PointT> sacseg;

      //set RANSAC parameters
      sacseg.setOptimizeCoefficients(true);
      sacseg.setModelType(pcl::SACMODEL_PLANE);
      sacseg.setMethodType(pcl::SAC_RANSAC);
      sacseg.setMaxIterations(100);
      sacseg.setDistanceThreshold(distance_thres); //[m]
      sacseg.setInputCloud(input);

      try
      {
        sacseg.segment(inliers, coeffs);
      }
      catch (const pcl::PCLException &e)
      {
        //エラーハンドリング (反応しない...)
        NODELET_WARN("Plane Model Detection Error");
        return -1; //failure
      }

      return 0; //success
    }

  
  private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    tf::TransformListener tf_listener;

    ros::Subscriber points_sub;
    ros::Publisher pub;

    std::string base_link_frame;
    double distance_thres; //segmentation_plane

  };
}
PLUGINLIB_EXPORT_CLASS(soma_perception::DiameterEstimationNodelet, nodelet::Nodelet)