//ros
#include <ros/ros.h>
#include <ros/names.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <pluginlib/class_list_macros.h>


//pcl 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/point_types.h>


//others
#include <math.h>

#define PI 3.14159265359

namespace soma_ros
{
    class CloudPlaneSegmentator : public pcl_ros::PCLNodelet
    {

    public:
        virtual void onInit();
        void operate(const sensor_msgs::PointCloud2 &cloud_input_ros_);
        void publish();

    private:
        ros::NodeHandle nh_;
        ros::Subscriber sub_;

        void calcTilt(Eigen::Vector3d v, Eigen::Vector3d w, int i);
        void segment(pcl::PointCloud<pcl::PointXYZ> cloud_input_, pcl::PointIndices::Ptr inliers, int i);
        void extract(pcl::PointCloud<pcl::PointXYZ> cloud_input_, pcl::PointIndices::Ptr inliers,
                     pcl::PointCloud<pcl::PointXYZ> *cloud_pcl);
        void extract_without(pcl::PointCloud<pcl::PointXYZ> cloud_input_, pcl::PointIndices::Ptr inliers,
                             pcl::PointCloud<pcl::PointXYZ> *cloud_pcl);

    protected:
        ros::Publisher cloud_segmented_pub_;
        ros::Publisher cloud_without_segmented_pub_;
        ros::Publisher cloud_slope_segmented_pub_;

        ros::Publisher indices_pub_;
        ros::Publisher coefficients_pub_;
        ros::Publisher tilt_array_pub_;

        sensor_msgs::PointCloud2 cloud_segmented_ros_;
        sensor_msgs::PointCloud2 cloud_without_segmented_ros_;
        sensor_msgs::PointCloud2 cloud_slope_segmented_ros_;

        pcl_msgs::PointIndices indices_ros_;
        pcl_msgs::ModelCoefficients coefficients_ros_;
        std_msgs::Float32MultiArray tilt_array_;

        pcl::PointCloud<pcl::PointXYZ> cloud_input_pcl_;
    };
    

    void CloudPlaneSegmentator::onInit()
    {
        NODELET_INFO("Initializing PlaneSegmentator ");
        nh_ = getNodeHandle();

        cloud_segmented_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_segmented", 1);
        cloud_without_segmented_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_without_segmented", 1);
        cloud_slope_segmented_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_slope_segmented", 1);
        indices_pub_ = nh_.advertise<pcl_msgs::PointIndices>("indices", 1);
        coefficients_pub_ = nh_.advertise<pcl_msgs::ModelCoefficients>("coefficients", 1);
        tilt_array_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("tilt_array", 1);

        sub_ = nh_.subscribe("cloud_downsampled", 10, &CloudPlaneSegmentator::operate, this);
        // publish();
    }


    void CloudPlaneSegmentator::operate(const sensor_msgs::PointCloud2 &cloud_input_ros_)
    {
        const int times_of_repeats = 2;
        const float setted_slope_tilt = 5.0;
        tilt_array_.data.resize(times_of_repeats);

        pcl::fromROSMsg(cloud_input_ros_, cloud_input_pcl_);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::PointCloud<pcl::PointXYZ> cloud_segmented_pcl;
        pcl::PointCloud<pcl::PointXYZ> cloud_without_segmented_pcl;
        pcl::PointCloud<pcl::PointXYZ> cloud_slope_segmented_pcl;

        segment(cloud_input_pcl_, inliers, 0);
        if (tilt_array_.data[0] < setted_slope_tilt)
        {
            extract(cloud_input_pcl_, inliers, &cloud_segmented_pcl);
        }
        else
        {
            extract(cloud_input_pcl_, inliers, &cloud_slope_segmented_pcl);
        }
        extract_without(cloud_input_pcl_, inliers, &cloud_without_segmented_pcl);

        for (int i = 1; i < times_of_repeats; i++)
        {
            segment(cloud_without_segmented_pcl, inliers, i);
            if (tilt_array_.data[i] < setted_slope_tilt)
            {
                extract(cloud_without_segmented_pcl, inliers, &cloud_segmented_pcl);
            }
            else
            {
                extract(cloud_without_segmented_pcl, inliers, &cloud_slope_segmented_pcl);
            }
            extract_without(cloud_without_segmented_pcl, inliers, &cloud_without_segmented_pcl);
        }

        // Convert to ROS msg
        pcl::toROSMsg(cloud_segmented_pcl, cloud_segmented_ros_);
        pcl::toROSMsg(cloud_without_segmented_pcl, cloud_without_segmented_ros_);
        pcl::toROSMsg(cloud_slope_segmented_pcl, cloud_slope_segmented_ros_);
        publish();
    }

    void CloudPlaneSegmentator::publish()
    {
        cloud_segmented_pub_.publish(cloud_segmented_ros_);
        cloud_without_segmented_pub_.publish(cloud_without_segmented_ros_);
        cloud_slope_segmented_pub_.publish(cloud_slope_segmented_ros_);
        indices_pub_.publish(indices_ros_);
        coefficients_pub_.publish(coefficients_ros_);
        tilt_array_pub_.publish(tilt_array_);
    }

    void CloudPlaneSegmentator::calcTilt(Eigen::Vector3d v, Eigen::Vector3d w, int i)
    {
        float cos_sita = v.dot(w) / v.norm() * w.norm();
        float sita = acos(cos_sita);
        float tilt = sita * 180.0 / PI;
        tilt = 180.0 - tilt;

        //Store tilt data
        tilt_array_.data[i] = tilt;
    }

    void CloudPlaneSegmentator::segment(pcl::PointCloud<pcl::PointXYZ> cloud_input_, pcl::PointIndices::Ptr inliers, int i)
    {
        pcl::ModelCoefficients coefficients_pcl;
        pcl::SACSegmentation<pcl::PointXYZ> segmentation;

        // Create the segmentation object
        segmentation.setOptimizeCoefficients(true);
        segmentation.setModelType(pcl::SACMODEL_PLANE);
        segmentation.setMethodType(pcl::SAC_RANSAC);
        segmentation.setMaxIterations(1000);
        segmentation.setDistanceThreshold(0.03);
        segmentation.setInputCloud(cloud_input_.makeShared());
        segmentation.segment(*inliers, coefficients_pcl);

        //Calc planar tilt
        Eigen::Vector3d vertical(0, 1, 0);
        Eigen::Vector3d slope(coefficients_pcl.values[0], coefficients_pcl.values[1], coefficients_pcl.values[2]);
        calcTilt(vertical, slope, i);

        pcl_conversions::fromPCL(*inliers, indices_ros_);
        pcl_conversions::fromPCL(coefficients_pcl, coefficients_ros_);
    }

    void CloudPlaneSegmentator::extract(pcl::PointCloud<pcl::PointXYZ> cloud_input_, pcl::PointIndices::Ptr inliers,
                                        pcl::PointCloud<pcl::PointXYZ> *cloud_pcl)
    {
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        // Create the filtering object
        extract.setInputCloud(cloud_input_.makeShared());
        extract.setIndices(inliers);
        // Extract the plannar inlier pointcloud from indices
        extract.setNegative(false);
        extract.filter(*cloud_pcl);
    }

    void CloudPlaneSegmentator::extract_without(pcl::PointCloud<pcl::PointXYZ> cloud_input_, pcl::PointIndices::Ptr inliers,
                                                pcl::PointCloud<pcl::PointXYZ> *cloud_pcl)
    {
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        // Create the filtering object
        extract.setInputCloud(cloud_input_.makeShared());
        extract.setIndices(inliers);
        // Extract the plannar inlier pointcloud from indices
        extract.setNegative(true);
        extract.filter(*cloud_pcl);
    }

} // namespace soma_ros

PLUGINLIB_EXPORT_CLASS(soma_ros::CloudPlaneSegmentator, nodelet::Nodelet)