#include "soma_ros/Data/Data.h"

#define ROS_DIST 1 //0:kinetic 1:melodic

Data::Data()
{
  ROS_INFO("Create Data object");
  // ros_distro = std::string(std::getenv("ROS_DISTRO"));

  state = last_state = State::Stop;
  action = Action::Stop;

  clutch = 0; //clutch Free state

  // vt = wt = 0.0;
  // np = 0;

  tf = new tf::TransformListener();

  tfBuf = new tf2_ros::Buffer();
  tf_2 = new tf2_ros::TransformListener(*tfBuf);

  ROS_INFO("Create local costmap");

//for kinetic distribution
#if ROS_DIST == 0
  local_costmap = new costmap_2d::Costmap2DROS("local_costmap", *tf);
  local_planner = new dwa_local_planner::DWAPlannerROS();
  local_planner->initialize("dwa_local_planner", tf, local_costmap);
#endif
#if ROS_DIST == 1
  //for merodic distribution
  local_costmap = new costmap_2d::Costmap2DROS("local_costmap", *tfBuf);
  local_planner = new dwa_local_planner::DWAPlannerROS();
  local_planner->initialize("dwa_local_planner", tfBuf, local_costmap);
#endif
}

Data::~Data() {}
