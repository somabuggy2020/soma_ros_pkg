#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <chrono>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                        sensor_msgs::PointCloud2>
    ASyncPolicy;

namespace soma_ros
{
    class points_marge_nodelet : public nodelet::Nodelet
    {
    public:
        virtual void onInit();

    private:
        void callback(const sensor_msgs::PointCloud2ConstPtr &_pc1,
                      const sensor_msgs::PointCloud2ConstPtr &_pc2);

    private:
        ros::NodeHandle nh;
        ros::NodeHandle pnh;

        //topic name parameter
        std::string frame_id;
        std::string src_frame1, src_frame2;
        std::string points1_topic, points2_topic;
        std::string output_points_topic;

        //subscribers
        message_filters::Subscriber<sensor_msgs::PointCloud2> points1_sub;
        message_filters::Subscriber<sensor_msgs::PointCloud2> points2_sub;
        message_filters::Synchronizer<ASyncPolicy> *sync;

        ros::Publisher output_points_pub;

        tf::TransformListener tf_lisner;
        tf::StampedTransform tf1, tf2;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener *tf2_lister;

        std::chrono::system_clock::time_point start, end;
        double elapsed;
    };

    void points_marge_nodelet::onInit()
    {
        NODELET_INFO("Points Marge Nodelet Init");

        nh = getNodeHandle();
        pnh = getPrivateNodeHandle();

        frame_id = pnh.param<std::string>("frame_id", "frame");
        src_frame1 = pnh.param<std::string>("src_frame1", "src_frame1");
        src_frame2 = pnh.param<std::string>("src_frame2", "src_frame2");
        points1_topic = pnh.param<std::string>("points1_topic", "points1");
        points2_topic = pnh.param<std::string>("points2_topic", "points2");
        output_points_topic = pnh.param<std::string>("output_points_topic", "output_points");

        points1_sub.subscribe(nh, points1_topic, 3);
        points2_sub.subscribe(nh, points2_topic, 3);

        sync = new message_filters::Synchronizer<ASyncPolicy>(ASyncPolicy(3),
                                                              points1_sub,
                                                              points2_sub);

        //callback regist
        sync->registerCallback(boost::bind(&points_marge_nodelet::callback, this, _1, _2));

        tf2_lister = new tf2_ros::TransformListener(tfBuffer);

        output_points_pub = nh.advertise<sensor_msgs::PointCloud2>(output_points_topic, 3);
    }

    void points_marge_nodelet::callback(const sensor_msgs::PointCloud2ConstPtr &_pc1,
                                        const sensor_msgs::PointCloud2ConstPtr &_pc2)
    {
        // NODELET_INFO("Subscribed two point clouds");

        //convert to pointcloud2
        sensor_msgs::PointCloud2 pc1_tf, pc2_tf;

        geometry_msgs::TransformStamped tfs;
        Eigen::Matrix4f m;

        try
        {
            tfs = tfBuffer.lookupTransform(frame_id,
                                           src_frame1,
                                           ros::Time(0),
                                           ros::Duration(5.0));
            m = tf2::transformToEigen(tfs.transform).matrix().cast<float>();
            pcl_ros::transformPointCloud(m, *_pc1, pc1_tf);

            tfs = tfBuffer.lookupTransform(frame_id,
                                           src_frame2,
                                           ros::Time(0),
                                           ros::Duration(5.0));
            m = tf2::transformToEigen(tfs.transform).matrix().cast<float>();
            pcl_ros::transformPointCloud(m, *_pc2, pc2_tf);
        }
        catch (tf2::TransformException &e)
        {
            NODELET_WARN("Lost transform");
            NODELET_WARN("%s", e.what());
            return;
        }

        //convert to pcl point cloud
        pcl::PointCloud<pcl::PointXYZRGB> pcl_pc1, pcl_pc2;
        pcl::PointCloud<pcl::PointXYZRGB> _pcl_out;

        pcl::fromROSMsg(pc1_tf, pcl_pc1);
        pcl::fromROSMsg(pc2_tf, pcl_pc2);

        //marge points
        _pcl_out += pcl_pc1;
        _pcl_out += pcl_pc2;

        //convert to message type
        sensor_msgs::PointCloud2 out;
        pcl::toROSMsg(_pcl_out, out);
        out.header.frame_id = frame_id;
        out.header.stamp = ros::Time::now();

        output_points_pub.publish(out);

    } // namespace soma_ros

} // namespace soma_ros

PLUGINLIB_EXPORT_CLASS(soma_ros::points_marge_nodelet, nodelet::Nodelet);