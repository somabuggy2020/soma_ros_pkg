//ros
#include <ros/ros.h>
#include <ros/names.h>
#include <ros/time.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pluginlib/class_list_macros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <string>
#include <math.h>

#define PI 3.14159265359

namespace soma_perception
{
class PlaneSegmentationNodelet : public nodelet::Nodelet
{
public:
  typedef pcl::PointXYZRGB PointT;
  typedef message_filters::sync_policies::ApproximateTime
            <sensor_msgs::PointCloud2, sensor_msgs::Imu> MySyncPolicy;

  PlaneSegmentationNodelet() {}
  virtual ~PlaneSegmentationNodelet() {}

  virtual void onInit()
  {
    NODELET_INFO("Initializing PlaneSegmentationNodelet");

    nh = getNodeHandle();
    pnh = getPrivateNodeHandle();

    base_link_frame = nh.param<std::string>("base_link", "soma_link"); 
    //default value is "soma_link"

    message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub(nh, "input_points", 3);
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "input_imu", 3);
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(3), points_sub, imu_sub);
    sync.registerCallback(&PlaneSegmentationNodelet::cloud_callback, this);   

    //advertise topics
    floor_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_floor", 1);
    slope_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_slope", 1);
    others_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_others", 1);
    //    indices_pub = nh.advertise<pcl_msgs::PointIndices>("indices", 1);
    //    coeffs_pub = nh.advertise<pcl_msgs::ModelCoefficients>("coeffs", 1);
    tilt_ary_pub = nh.advertise<std_msgs::Float32MultiArray>("tilt_ary", 1);
  }

private:
  /*!
   * \brief cloud_callback
   * \param input
   */
  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr _input, 
  const sensor_msgs::ImuConstPtr imu_data)
  {
    
    pcl::PointCloud<PointT>::Ptr input;
    input.reset(new pcl::PointCloud<PointT>());
    input->clear();
    pcl::fromROSMsg(*_input, *input);

    if (input->empty())
    {
      return;
    }

    pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());

    if(!base_link_frame.empty()) {
      //Does exist transform tf points frame to base frame?
      if(!tf_listener.canTransform(base_link_frame, input->header.frame_id, ros::Time(0))) {
        return; //nothing
      }

      //get transform
      tf::StampedTransform transform;
      tf_listener.waitForTransform(base_link_frame, input->header.frame_id, ros::Time(0), ros::Duration(2.0));
      tf_listener.lookupTransform(base_link_frame, input->header.frame_id, ros::Time(0), transform);

      pcl_ros::transformPointCloud(*input, *transformed, transform);
      transformed->header.frame_id = base_link_frame;
      transformed->header.stamp = input->header.stamp;
      // input = transformed; //copy?
    }


    const int times_of_repeats = 2;
    const float setted_slope_tilt = 5.0;
    // tilt_ary.data.resize(times_of_repeats);

    //input roll, pitch, yaw
    tilt_ary.data.resize(3);

    //    pcl::PointCloud<PointT>::Ptr pc_floor(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr pc_slope(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr pc_others(new pcl::PointCloud<PointT>());

    //perform segnemtation
    pcl::PointIndices::Ptr inliers;
    pcl::ModelCoefficients::Ptr coeffs;
    inliers.reset(new pcl::PointIndices());
    coeffs.reset(new pcl::ModelCoefficients());

    segmentation(transformed, inliers, coeffs);

    extract_RPY(coeffs);

    pcl::ExtractIndices<PointT> EI;
    EI.setInputCloud(transformed);
    EI.setIndices(inliers);
    EI.setNegative(false);
    EI.filter(*pc_slope);

    EI.setNegative(true);
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

    //    floor_pub.publish(pc_floor);
    slope_pub.publish(pc_slope);
    others_pub.publish(pc_others);
    tilt_ary_pub.publish(tilt_ary);

    //    publish();
  }

  int segmentation(pcl::PointCloud<PointT>::ConstPtr input,
                   pcl::PointIndices::Ptr inliers,
                   pcl::ModelCoefficients::Ptr coeffs)
  {
    pcl::SACSegmentation<PointT> sacseg; //instance of RANSAC segmentation processing object

    //set RANSAC parameters
    sacseg.setOptimizeCoefficients(true);
    sacseg.setModelType(pcl::SACMODEL_PLANE);
    sacseg.setMethodType(pcl::SAC_RANSAC);
    sacseg.setMaxIterations(30);
    sacseg.setDistanceThreshold(0.05);
    sacseg.setInputCloud(input);
    sacseg.segment(*inliers, *coeffs);

    return 0; //success
  }

  //  void segment(pcl::PointCloud<PointT>::ConstPtr input,
  //               pcl::PointIndices::Ptr inliers,
  //               int i)
  //  {
  //    pcl::ModelCoefficients coeffs;
  //    pcl::SACSegmentation<PointT> seg;

  //    // Create the seg object
  //    seg.setOptimizeCoefficients(true);
  //    seg.setModelType(pcl::SACMODEL_PLANE);
  //    seg.setMethodType(pcl::SAC_RANSAC);
  //    seg.setMaxIterations(1000);
  //    seg.setDistanceThreshold(0.03);
  //    seg.setInputCloud(input);
  //    seg.segment(*inliers, coeffs);

  //    //Calc planar tilt
  //    Eigen::Vector3d vertical(0, 1, 0);
  //    Eigen::Vector3d slope(coeffs.values[0], coeffs.values[1], coeffs.values[2]);
  //    calcTilt(vertical, slope, i);

  //    //    pcl_conversions::fromPCL(*inliers, indices_ros);
  //    //    pcl_conversions::fromPCL(coeffs, coeffs_ros);
  //  }

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

  /*!
   * \brief calcTilt
   * \param v
   * \param w
   * \param i
   */
  void calcTilt(Eigen::Vector3d v, Eigen::Vector3d w, int i)
  {
    float cos_sita = v.dot(w) / v.norm() * w.norm();
    float sita = acos(cos_sita);
    float tilt = sita * 180.0 / PI;
    tilt = 180.0 - tilt;

    //Store tilt data
    tilt_ary.data[i] = tilt;
  }

  void extract_RPY(pcl::ModelCoefficients::Ptr coeffs)
  {
    Eigen::Vector3d x_axis(1, 0, 0);
    Eigen::Vector3d y_axis(0, 1, 0);
    Eigen::Vector3d z_axis(0, 0, 1);

    Eigen::Vector3d normal(coeffs->values[0], coeffs->values[1], coeffs->values[2]);

    float roll_rad = x_axis.dot(normal) / ( x_axis.norm() * normal.norm());
    float pitch_rad = y_axis.dot(normal) / ( y_axis.norm() * normal.norm());
    float yaw_rad = z_axis.dot(normal) / ( z_axis.norm() * normal.norm());

    roll_rad = acos(roll_rad);
    pitch_rad = acos(pitch_rad);
    yaw_rad = acos(yaw_rad);

    float roll = 180 - (roll_rad * 180.0 / PI);
    float pitch = 180 - (pitch_rad * 180.0 / PI);
    float yaw = 180 - (yaw_rad * 180.0 / PI);

    tilt_ary.data[0] = roll;
    tilt_ary.data[1] = pitch;
    tilt_ary.data[2] = yaw;
  }

private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;

  std::string base_link_frame; //base_link frame id
  tf::TransformListener tf_listener;

  //subscribers
  ros::Subscriber input_points_sub;

  //publishers
  ros::Publisher floor_pub;
  ros::Publisher others_pub;
  ros::Publisher slope_pub;

  ros::Publisher coeffs_pub;
  ros::Publisher tilt_ary_pub;

  //  pcl_msgs::PointIndices indices_ros;
  //  pcl_msgs::ModelCoefficients coeffs_ros;
  std_msgs::Float32MultiArray tilt_ary;

  //  pcl::PointCloud<PointT> cloud_input_pcl_;
};
} // namespace plane_seg_pkg

PLUGINLIB_EXPORT_CLASS(soma_perception::PlaneSegmentationNodelet, nodelet::Nodelet)
