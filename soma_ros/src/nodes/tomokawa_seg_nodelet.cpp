//ros
#include <ros/ros.h>
#include <ros/names.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <pluginlib/class_list_macros.h>

#include <ros/time.h>
#include <tf/transform_listener.h>

//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

//others
#include <string>
#include <math.h>

#define PI 3.14159265359

namespace soma_ros
{
class CloudPlaneSegmentator : public nodelet::Nodelet
{
public:
  typedef pcl::PointXYZRGB PointT;

  CloudPlaneSegmentator() {}
  virtual ~CloudPlaneSegmentator() {}

  virtual void onInit()
  {
    NODELET_INFO("Initializing PlaneSegmentator ");

    nh = getNodeHandle();
    pnh = getPrivateNodeHandle();

    input_points_sub = nh.subscribe("input_points",
                                    3,
                                    &CloudPlaneSegmentator::cloud_callback,
                                    this);
    //advertise topics
    floor_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_floor", 1);
    others_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_others", 1);
    slope_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_slope", 1);
    //    indices_pub = nh.advertise<pcl_msgs::PointIndices>("indices", 1);
    //    coeffs_pub = nh.advertise<pcl_msgs::ModelCoefficients>("coeffs", 1);
    //    tilt_ary_pub = nh.advertise<std_msgs::Float32MultiArray>("tilt_ary", 1);
  }

  /*!
         * \brief cloud_callback
         * \param input
         *
         * ポイントクラウドのコールバック関数
         */

private:
  void cloud_callback(const pcl::PointCloud<PointT>::ConstPtr &input)
  {
    if (input->empty())
    {
      return;
    }

    const int times_of_repeats = 2;
    const float setted_slope_tilt = 5.0;

    tilt_ary.data.resize(times_of_repeats);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::PointCloud<PointT>::Ptr pc_floor(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr pc_slope(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr pc_others(new pcl::PointCloud<PointT>());


    //perform segnemtation
    segment(input, inliers, 0);


    pcl::ExtractIndices<PointT> EI;
    // Create the filtering object
    EI.setInputCloud(input);
    EI.setIndices(inliers);
    // extraction the plannar inlier pointcloud from indices
    EI.setNegative(true);
    EI.filter(*pc_slope);

    EI.setNegative(false);
    EI.filter(*pc_others);


    //    if (tilt_ary.data[0] < setted_slope_tilt)
    //    {
    //      pc_floor = extract(input, inliers);
    //    }
    //    else
    //    {
    //      pc_slope = (input, inliers);
    //    }

    //    pc_others = extract_others(input, inliers);

    //    for (int i = 1; i < times_of_repeats; i++)
    //    {
    //      segment(input, inliers, i);
    //      if (tilt_ary.data[i] < setted_slope_tilt)
    //      {
    //        pc_floor = extract(input, inliers);
    //      }
    //      else
    //      {
    //        pc_slope = (input, inliers);
    //      }
    //      pc_others = extract_others(input, inliers);
    //    }

    // // Convert to ROS msg
    // pcl::toROSMsg(pc_floor, pc_floor_ros);
    // pcl::toROSMsg(pc_others, pc_others_ros);
    // pcl::toROSMsg(pc_slope, pc_slope_ros);

    floor_pub.publish(pc_floor);
    slope_pub.publish(pc_slope);
    others_pub.publish(pc_others);

    //    publish();
  }

  //  void publish()
  //  {
  //            floor_pub.publish(pc_floor_ros);
  //            others_pub.publish(pc_others_ros);
  //            slope_pub.publish(pc_slope_ros);
  //            indices_pub.publish(indices_ros);
  //            coeffs_pub.publish(coeffs_ros);
  //            tilt_ary_pub.publish(tilt_ary);
  //  }

  void segment(const pcl::PointCloud<PointT>::ConstPtr &input,
               pcl::PointIndices::Ptr inliers,
               int i)
  {
    //    pcl::PointCloud<PointT>::Ptr segmented(new pcl::PointCloud<PointT>());

    pcl::ModelCoefficients coeffs;
    pcl::SACSegmentation<PointT> seg;

    // Create the seg object
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.03);
    seg.setInputCloud(input);
    seg.segment(*inliers, coeffs);

    //Calc planar tilt
    Eigen::Vector3d vertical(0, 1, 0);
    Eigen::Vector3d slope(coeffs.values[0], coeffs.values[1], coeffs.values[2]);
    calcTilt(vertical, slope, i);

    //    pcl_conversions::fromPCL(*inliers, indices_ros);
    //    pcl_conversions::fromPCL(coeffs, coeffs_ros);
  }

  //  pcl::PointCloud<PointT>::ConstPtr extract(const pcl::PointCloud<PointT>::ConstPtr &input,
  //                                            pcl::PointIndices::Ptr inliers)
  //  {
  //    pcl::ExtractIndices<PointT> extract;
  //    pcl::PointCloud<PointT>::Ptr done(new pcl::PointCloud<PointT>());

  //    // Create the filtering object
  //    extract.setInputCloud(input);
  //    extract.setIndices(inliers);
  //    // Extract the plannar inlier pointcloud from indices
  //    extract.setNegative(false);
  //    extract.filter(*done);

  //    return done;
  //  }

  //  pcl::PointCloud<PointT>::ConstPtr extract_others(const pcl::PointCloud<PointT>::ConstPtr &input,
  //                                                   pcl::PointIndices::Ptr inliers)
  //  {
  //    pcl::ExtractIndices<PointT> extract;
  //    pcl::PointCloud<PointT>::Ptr done(new pcl::PointCloud<PointT>());

  //    // Create the filtering object
  //    extract.setInputCloud(input);
  //    extract.setIndices(inliers);
  //    // Extract the plannar inlier pointcloud from indices
  //    extract.setNegative(true);
  //    extract.filter(*done);

  //    return done;
  //  }

  void calcTilt(Eigen::Vector3d v, Eigen::Vector3d w, int i)
  {
    float cos_sita = v.dot(w) / v.norm() * w.norm();
    float sita = acos(cos_sita);
    float tilt = sita * 180.0 / PI;
    tilt = 180.0 - tilt;

    //Store tilt data
    tilt_ary.data[i] = tilt;
  }

private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  ros::Subscriber input_points_sub;

protected:
  ros::Publisher floor_pub;
  ros::Publisher others_pub;
  ros::Publisher slope_pub;
  ros::Publisher indices_pub;
  ros::Publisher coeffs_pub;
  ros::Publisher tilt_ary_pub;

  //        sensor_msgs::PointCloud2 pc_floor_ros;
  //        sensor_msgs::PointCloud2 pc_others_ros;
  //        sensor_msgs::PointCloud2 pc_slope_ros;

  //  pcl_msgs::PointIndices indices_ros;
  //  pcl_msgs::ModelCoefficients coeffs_ros;
  std_msgs::Float32MultiArray tilt_ary;

  //  pcl::PointCloud<PointT> cloud_input_pcl_;
};
} // namespace plane_seg_pkg

PLUGINLIB_EXPORT_CLASS(soma_ros::CloudPlaneSegmentator, nodelet::Nodelet)
