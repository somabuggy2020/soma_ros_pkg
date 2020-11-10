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
#include <message_filters/time_synchronizer.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
  typedef pcl::PointXYZRGB PointT;

  struct my_pointCloud
  {
    pcl::PointCloud<PointT>::Ptr pc;
    int judge = 3;
    //0...ground, 1...floor, 2...slope, 3...others
  };

  class PlaneSegmentationNodelet : public nodelet::Nodelet
  {
  public:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Imu> MySyncPolicy;

    PlaneSegmentationNodelet() {}
    virtual ~PlaneSegmentationNodelet() {}

    virtual void onInit()
    {
      NODELET_INFO("Initializing PlaneSegmentationNodelet");

      nh = getNodeHandle();
      pnh = getPrivateNodeHandle();

      tfBuf = new tf2_ros::Buffer();
      tfListener = new tf2_ros::TransformListener(*tfBuf);

      initialize_params();

      //advertise topics
      // ground_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_ground", 1);
      // slope_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_slope", 1);
      others_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_others", 1);
      //    indices_pub = nh.advertise<pcl_msgs::PointIndices>("indices", 1);
      //    coeffs_pub = nh.advertise<pcl_msgs::ModelCoefficients>("coeffs", 1);
      tilt_ary_pub = nh.advertise<std_msgs::Float32MultiArray>("tilt_ary", 1);
      rpy_ary_pub = nh.advertise<std_msgs::Float32MultiArray>("rpy_ary", 1);
      adevertise(nh);

      points_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "input_points", 3);
      imu_sub = new message_filters::Subscriber<sensor_msgs::Imu>(nh, "input_imu", 3);
      sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *points_sub, *imu_sub);
      sync->registerCallback(boost::bind(&PlaneSegmentationNodelet::cloud_callback, this, _1, _2));
    }

  private:
    /**
   * @brief initialize parameters
   */
    void initialize_params()
    {
      base_link_frame = pnh.param<std::string>("base_link", "soma_link");
      // times_of_rpeats = nh.param<int>("times_of_repeats", 2);
      setted_slope_tilt = pnh.param<float>("setted_slope_tilt", 25.0);
      setted_ground_tilt = pnh.param<float>("setted_ground_tilt", 3.0);
    }

    void adevertise(ros::NodeHandle nh)
    {
      cloud_1_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_1", 1);
      cloud_2_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_2", 1);
      cloud_3_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_3", 1);
      cloud_4_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_4", 1);
      cloud_5_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_5", 1);
      cloud_6_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_6", 1);
      cloud_7_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_7", 1);
      cloud_8_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_8", 1);
      cloud_9_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_9", 1);
      cloud_10_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_10", 1);
    }

    /**
   **/
    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_input,
                        const sensor_msgs::ImuConstPtr &imu_data)
    {
      NODELET_INFO("cloud_callback function");

      //point cloud conversion to pcl point cloud
      pcl::PointCloud<PointT>::Ptr cloud_raw;
      cloud_raw.reset(new pcl::PointCloud<PointT>());
      cloud_raw->clear();
      pcl::fromROSMsg(*cloud_input, *cloud_raw); //conversion

      if (cloud_raw->empty())
      {
        return;
      }

      //transform point cloud to base_link_frame
      // pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());
      // if (!base_link_frame.empty())
      // {
      //   //Does exist transform tf points frame to base frame?
      //   if (!tf_listener.canTransform(base_link_frame, cloud_raw->header.frame_id, ros::Time(0)))
      //   {
      //     return; //nothing
      //   }

      //   //get transform
      //   tf::StampedTransform transform;
      //   tf_listener.waitForTransform(base_link_frame, cloud_raw->header.frame_id, ros::Time(0), ros::Duration(2.0));
      //   tf_listener.lookupTransform(base_link_frame, cloud_raw->header.frame_id, ros::Time(0), transform);

      //   pcl_ros::transformPointCloud(*cloud_raw, *transformed, transform);
      //   transformed->header.frame_id = base_link_frame;
      //   transformed->header.stamp = cloud_raw->header.stamp;
      //   // input = transformed; //copy?
      // }

      try
      {
        geometry_msgs::TransformStamped tf2base_link;
        tf2base_link = tfBuf->lookupTransform(base_link_frame, cloud_raw->header.frame_id, ros::Time(0));
        // Eigen::MatrixX4f tmat = tf2::transformToEigen(tf2base_link.transform).matrix().cast<float>();
        pcl_ros::transformPointCloud<PointT>(*cloud_raw, *cloud_raw, tf2base_link.transform);
      }
      catch (tf2::TransformException &e)
      {
        ROS_WARN("%s", e.what());
        return;
      }

      //input roll, pitch, yaw
      tilt_ary.data.resize(3);
      rpy_ary.data.resize(3);

      my_pointCloud pc_ary[10];
      for (int i = 0; i < 10; i++)
      {
        pc_ary[i].pc.reset(new pcl::PointCloud<PointT>());
      }
      pcl::PointCloud<PointT>::Ptr pc_others(new pcl::PointCloud<PointT>());

      pcl::PointIndices::Ptr inliers;
      pcl::ModelCoefficients::Ptr coeffs;
      inliers.reset(new pcl::PointIndices());
      coeffs.reset(new pcl::ModelCoefficients());

      convert_imu_RPY(imu_data);

      segmentation(cloud_raw, inliers, coeffs);
      pcl::ExtractIndices<PointT> EI;
      EI.setInputCloud(cloud_raw);
      EI.setIndices(inliers);

      // extract_tilt_RPY(coeffs);
      // EI.setNegative(false);
      // if(setted_slope_tilt < tilt_ary.data[1]) {
      //   pc_ary[0].judge = 2;
      // }
      // EI.filter(*pc_ary[0].pc);

      int i = 0;
      while (cloud_raw->points.size() * 0.3 < pc_others->points.size() || i < 5)
      {
        if (i != 0)
        {
          segmentation(pc_others, inliers, coeffs);
          EI.setInputCloud(pc_others);
          EI.setIndices(inliers);
        }
        extract_tilt_RPY(coeffs);
        if (tilt_ary.data[1] < setted_ground_tilt)
        {
          EI.setNegative(false);
          EI.filter(*pc_ary[i].pc);
          pc_ary[i].judge = 0;
        }
        else if ((tilt_ary.data[1] + rpy_ary.data[1]) < setted_slope_tilt)
        {
          EI.setNegative(false);
          EI.filter(*pc_ary[i].pc);
          pc_ary[i].judge = 1;
        }
        else if (setted_slope_tilt < (tilt_ary.data[1] + rpy_ary.data[1]))
        {
          EI.setNegative(false);
          EI.filter(*pc_ary[i].pc);
          pc_ary[i].judge = 2;
        }

        EI.setNegative(true);
        EI.filter(*pc_others);

        i++;
        NODELET_INFO("segmentation %d times", i);
      }

      //publish topics only when pc_ary[i] is slope
      for (int k = 0; k < 5; k++)
      {
        if (pc_ary[k].judge == 2)
        {
          switch (k)
          {
          case 0:
            cloud_1_pub.publish(pc_ary[0].pc);
            break;
          case 1:
            cloud_2_pub.publish(pc_ary[1].pc);
            break;
          case 2:
            cloud_3_pub.publish(pc_ary[2].pc);
            break;
          case 3:
            cloud_4_pub.publish(pc_ary[3].pc);
            break;
          case 4:
            cloud_5_pub.publish(pc_ary[4].pc);
            break;
          case 5:
            cloud_6_pub.publish(pc_ary[5].pc);
            break;
          case 6:
            cloud_7_pub.publish(pc_ary[6].pc);
            break;
          case 7:
            cloud_8_pub.publish(pc_ary[7].pc);
            break;
          case 8:
            cloud_9_pub.publish(pc_ary[8].pc);
            break;
          case 9:
            cloud_10_pub.publish(pc_ary[9].pc);
            break;
          default:
            break;
          }
        }
      }

      others_pub.publish(pc_others);
      tilt_ary_pub.publish(tilt_ary);
      rpy_ary_pub.publish(rpy_ary);
    }

    /**
   * \brief segmentation
   * \param input
   * \param inliers
   * \param coeffs
   */
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

    /**
   * \brief convert_imu_RPY
   * \param imu_data
   */
    void convert_imu_RPY(const sensor_msgs::ImuConstPtr &imu_data)
    {
      tf2::Quaternion quat_imu;
      tf2::fromMsg(imu_data->orientation, quat_imu);
      double roll, pitch, yaw;
      tf2::Matrix3x3(quat_imu).getRPY(roll, pitch, yaw);

      rpy_ary.data[0] = (float)roll;
      rpy_ary.data[1] = (float)pitch;
      rpy_ary.data[2] = (float)yaw;
    }

    /**
   * \brief extract_tilt_RPY
   * \param coeffs
   */
    void extract_tilt_RPY(pcl::ModelCoefficients::Ptr coeffs)
    {
      Eigen::Vector3d x_axis(1, 0, 0);
      Eigen::Vector3d y_axis(0, 1, 0);
      Eigen::Vector3d z_axis(0, 0, 1);

      Eigen::Vector3d normal(coeffs->values[0], coeffs->values[1], coeffs->values[2]);

      float roll_rad = x_axis.dot(normal) / (x_axis.norm() * normal.norm());
      float pitch_rad = z_axis.dot(normal) / (y_axis.norm() * normal.norm());
      float yaw_rad = y_axis.dot(normal) / (z_axis.norm() * normal.norm());

      roll_rad = acos(roll_rad);
      pitch_rad = acos(pitch_rad);
      yaw_rad = acos(yaw_rad);

      float roll = roll_rad * 180.0 / PI;
      float pitch = pitch_rad * 180.0 / PI;
      float yaw = yaw_rad * 180.0 / PI;

      tilt_ary.data[0] = roll;
      tilt_ary.data[1] = pitch;
      tilt_ary.data[2] = yaw;
    }

  private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    //params
    std::string base_link_frame; //base_link frame id
    // std::string input_imu;
    // std::string input_points;
    tf::TransformListener tf_listener;
    tf2_ros::Buffer *tfBuf;
    tf2_ros::TransformListener *tfListener;
    // int times_of_rpeats;
    float setted_slope_tilt;
    float setted_ground_tilt;

    //subscribers
    ros::Subscriber input_points_sub;

    message_filters::Subscriber<sensor_msgs::PointCloud2> *points_sub;
    message_filters::Subscriber<sensor_msgs::Imu> *imu_sub;
    message_filters::Synchronizer<MySyncPolicy> *sync;

    //publishers
    ros::Publisher others_pub;

    ros::Publisher cloud_1_pub;
    ros::Publisher cloud_2_pub;
    ros::Publisher cloud_3_pub;
    ros::Publisher cloud_4_pub;
    ros::Publisher cloud_5_pub;
    ros::Publisher cloud_6_pub;
    ros::Publisher cloud_7_pub;
    ros::Publisher cloud_8_pub;
    ros::Publisher cloud_9_pub;
    ros::Publisher cloud_10_pub;

    // ros::Publisher coeffs_pub;
    ros::Publisher tilt_ary_pub;
    ros::Publisher rpy_ary_pub;

    //  pcl_msgs::PointIndices indices_ros;
    //  pcl_msgs::ModelCoefficients coeffs_ros;
    std_msgs::Float32MultiArray tilt_ary;
    std_msgs::Float32MultiArray rpy_ary;
  };
} // namespace soma_perception

PLUGINLIB_EXPORT_CLASS(soma_perception::PlaneSegmentationNodelet, nodelet::Nodelet)
