#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>

#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>

#include <pcl/filters/project_inliers.h>

#include <pcl_conversions/pcl_conversions.h>

#include <map>
#include <string>

namespace sen_msgs = sensor_msgs;

typedef std::string sstring;
typedef sen_msgs::PointCloud2 MsgsPointCloud;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace soma_ros
{

  class vision_nodelet : public nodelet::Nodelet
  {
  public:
    virtual void onInit();

  private:
    void setup();
    void points_callback(const sen_msgs::PointCloud2ConstPtr &_input);

    void Filtering(PointCloudT::Ptr &in, PointCloudT::Ptr &out);
    void ExtractNeighbor(PointCloudT::Ptr &in, PointCloudT::Ptr &out);
    void GroundRemove(PointCloudT::Ptr &in, PointCloudT::Ptr &p_out, PointCloudT::Ptr &n_out);
    void Projection(PointCloudT::Ptr &in, PointCloudT::Ptr &out);
    void Clustering(PointCloudT::Ptr &in);

  private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    ros::Timer timer;

    std::string frame_id;
    std::string map_frame_id;

    std::string input_points_topic;

    //subscribers
    ros::Subscriber input_points_sub;

    //publishers
    ros::Publisher filtered_points_pub;
    ros::Publisher neighbor_points_pub;
    ros::Publisher np_pub;
    ros::Publisher ground_points_pub;
    ros::Publisher obstacle_points_pub;
    ros::Publisher projected_points_pub;
    ros::Publisher obs_num_pub;

    //point clouds
    PointCloudT::Ptr raw_points;
    PointCloudT::Ptr filtered_points;
    PointCloudT::Ptr neighbor_points;
    PointCloudT::Ptr ground_points, obstacle_points;
    PointCloudT::Ptr projected_points;

    int obs_num;
  };

  //----------------------------------------------------------------------------------------------------
  //----------------------------------------------------------------------------------------------------
  //----------------------------------------------------------------------------------------------------
  void vision_nodelet::onInit()
  {
    NODELET_INFO("Vision Nodelet Init");

    // 1.
    nh = getNodeHandle();
    pnh = getPrivateNodeHandle();

    // 2. setup
    // frame, topic names
    setup();

    //set subscribe
    input_points_sub = nh.subscribe<MsgsPointCloud>(input_points_topic,
                                                    3,
                                                    &vision_nodelet::points_callback,
                                                    this);

    //set publishers
    filtered_points_pub = nh.advertise<MsgsPointCloud>("filtered_points", 3);
    neighbor_points_pub = nh.advertise<MsgsPointCloud>("neighbor_points", 3);
    np_pub = nh.advertise<std_msgs::Int32>("np_value", 3);
    ground_points_pub = nh.advertise<MsgsPointCloud>("ground_points", 3);
    obstacle_points_pub = nh.advertise<MsgsPointCloud>("obstacle_points", 3);
    projected_points_pub = nh.advertise<MsgsPointCloud>("projected_points", 3);
    obs_num_pub = nh.advertise<std_msgs::Int8>("obs_num", 3);

    //initialize point clouds
    raw_points.reset(new PointCloudT);
    filtered_points.reset(new PointCloudT);
    neighbor_points.reset(new PointCloudT);
    ground_points.reset(new PointCloudT);
    obstacle_points.reset(new PointCloudT);
    projected_points.reset(new PointCloudT);
  }

  //----------------------------------------------------------------------------------------------------
  //----------------------------------------------------------------------------------------------------
  //----------------------------------------------------------------------------------------------------
  void vision_nodelet::setup()
  {
    frame_id = pnh.param<sstring>("frame_id", "soma_link");
    map_frame_id = pnh.param<sstring>("map_frame_id", "map");

    input_points_topic = pnh.param<sstring>("input_points_topic", "input_points_topic");
  }

  //----------------------------------------------------------------------------------------------------
  //----------------------------------------------------------------------------------------------------
  //----------------------------------------------------------------------------------------------------
  void vision_nodelet::points_callback(const sensor_msgs::PointCloud2ConstPtr &_input)
  {
    MsgsPointCloud tmp_msgs;

    //Convert to pcl point cloud type
    raw_points->clear();
    pcl::fromROSMsg(*_input, *raw_points);

    // ----------------------------------------------------------------------------------------------------
    // Point Cloud filtering process
    filtered_points->clear();
    Filtering(raw_points, filtered_points);

    // Publish filtered point cloud
    pcl::toROSMsg(*filtered_points, tmp_msgs);
    filtered_points_pub.publish(tmp_msgs);
    //----------------------------------------------------------------------------------------------------

    // ----------------------------------------------------------------------------------------------------
    // extraction neighbor points for avoid obstacle
    neighbor_points->clear();
    ExtractNeighbor(filtered_points, neighbor_points);
    pcl::toROSMsg(*neighbor_points, tmp_msgs);
    neighbor_points_pub.publish(tmp_msgs);

    int np = (int)neighbor_points->size();
    std_msgs::Int32 i32msgs;
    i32msgs.data = np;
    np_pub.publish(i32msgs);
    //----------------------------------------------------------------------------------------------------

    //----------------------------------------------------------------------------------------------------
    // Gound Remove
    ground_points->clear();
    obstacle_points->clear();
    GroundRemove(filtered_points, ground_points, obstacle_points);

    // Publish ground point cloud
    pcl::toROSMsg(*ground_points, tmp_msgs);
    ground_points_pub.publish(tmp_msgs);

    // Publish obstacle point cloud
    pcl::toROSMsg(*obstacle_points, tmp_msgs);
    obstacle_points_pub.publish(tmp_msgs);
    //----------------------------------------------------------------------------------------------------

    //----------------------------------------------------------------------------------------------------
    // Euclidean clustering
    Clustering(obstacle_points);
    std_msgs::Int8 imsg;
    imsg.data = obs_num;
    obs_num_pub.publish(imsg);
    //----------------------------------------------------------------------------------------------------

    //----------------------------------------------------------------------------------------------------
    // Projection to 2D plane (z=0)
    projected_points->clear();
    //    Projection(obstacle_points, projected_points);
    pcl::toROSMsg(*projected_points, tmp_msgs);
    projected_points_pub.publish(tmp_msgs);
    //----------------------------------------------------------------------------------------------------
  }

  //----------------------------------------------------------------------------------------------------
  //----------------------------------------------------------------------------------------------------
  //----------------------------------------------------------------------------------------------------
  void vision_nodelet::Filtering(PointCloudT::Ptr &in, PointCloudT::Ptr &out)
  {
    // Pass through filter
    pcl::PassThrough<PointT> PT;
    PT.setInputCloud(in);
    PT.setFilterFieldName("x");   //Fowared direction
    PT.setFilterLimits(0.0, 5.0); //
    PT.filter(*out);              //

    //Down sampling filter
    // pcl::VoxelGrid<PointT> VG;
    // VG.setInputCloud(out);
    // VG.setLeafSize(0.01, 0.01, 0.01);
    // VG.filter(*out);

    //Statical outlier removal
    // pcl::StatisticalOutlierRemoval<PointT> SOR;
    // SOR.setInputCloud(out);
    // SOR.setMeanK(20);
    // SOR.setStddevMulThresh(0.5);
    // SOR.filter(*out);
  }

  void vision_nodelet::ExtractNeighbor(PointCloudT::Ptr &in, PointCloudT::Ptr &out)
  {
    pcl::PointIndices::Ptr indices;

    pcl::PassThrough<PointT> PT;
    PT.setInputCloud(in);
    PT.setFilterFieldName("z");   //Vertical direction
    PT.setFilterLimits(0.3, 1.7); //
    PT.filter(*out);

    PT.setInputCloud(out);
    PT.setFilterFieldName("x");
    PT.setFilterLimits(0.1, 3.0);
    PT.filter(*out);

    PT.setInputCloud(out);
    PT.setFilterFieldName("y");
    PT.setFilterLimits(-0.6, 0.6);
    PT.filter(*out);
  }

  //----------------------------------------------------------------------------------------------------
  //----------------------------------------------------------------------------------------------------
  //----------------------------------------------------------------------------------------------------
  void vision_nodelet::GroundRemove(PointCloudT::Ptr &in, PointCloudT::Ptr &p_out, PointCloudT::Ptr &n_out)
  {
    // Pass through filter for vertical
    pcl::PointIndices::Ptr indices;
    indices.reset(new pcl::PointIndices);
    pcl::PassThrough<PointT> PT;
    PT.setInputCloud(in);
    PT.setFilterFieldName("z");    //Vertical direction
    PT.setFilterLimits(-5.0, 0.3); //
    PT.filter(indices->indices);   //

    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);
    seg.setInputCloud(in);
    seg.setIndices(indices);
    seg.segment(*inliers, *coeffs);

    // coeffs = [a, b, c, d]
    // normal vector = (a,b,c)
    Eigen::Vector3f ref = Eigen::Vector3f::UnitZ();
    Eigen::Vector3f vCoeffs;
    vCoeffs << coeffs->values[0], coeffs->values[1], coeffs->values[2];

    double dot = vCoeffs.dot(ref);
    if (abs(dot) < cos(80.0 * M_PI / 180.0))
    {
      pcl::copyPointCloud(*in, *n_out);
      return;
    }

    pcl::ExtractIndices<PointT> EI;
    EI.setInputCloud(in);
    EI.setIndices(inliers);
    EI.filter(*p_out);
    EI.setNegative(true);
    EI.filter(*n_out);
  }

  //----------------------------------------------------------------------------------------------------
  //----------------------------------------------------------------------------------------------------
  //----------------------------------------------------------------------------------------------------
  void vision_nodelet::Projection(PointCloudT::Ptr &in, PointCloudT::Ptr &out)
  {
    pcl::ModelCoefficients::Ptr coeffs;
    coeffs.reset(new pcl::ModelCoefficients);
    coeffs->values.resize(4);
    coeffs->values[0] = 0;
    coeffs->values[1] = 0;
    coeffs->values[2] = 1.0;
    coeffs->values[3] = 0;

    pcl::ProjectInliers<PointT> PI;
    PI.setModelType(pcl::SACMODEL_PLANE);
    PI.setModelCoefficients(coeffs);
    PI.setInputCloud(in);
    PI.filter(*out);
  }

  //----------------------------------------------------------------------------------------------------
  //----------------------------------------------------------------------------------------------------
  //----------------------------------------------------------------------------------------------------

  void vision_nodelet::Clustering(PointCloudT::Ptr &in)
  {
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(in);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> EC;
    EC.setClusterTolerance(0.05); //meter
    EC.setMinClusterSize(100);
    EC.setMaxClusterSize(300000);
    EC.setSearchMethod(tree);
    EC.setInputCloud(in);
    EC.extract(cluster_indices);

    obs_num = (int)cluster_indices.size();
  }

  //--------------------------------------------------
  //--------------------------------------------------
  //--------------------------------------------------
} // namespace soma_ros

PLUGINLIB_EXPORT_CLASS(soma_ros::vision_nodelet, nodelet::Nodelet);