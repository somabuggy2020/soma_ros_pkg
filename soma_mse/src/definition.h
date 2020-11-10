#ifndef DEFINITION_H
#define DEFINITION_H

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <string>
#include <map>

namespace geo_msgs = geometry_msgs;

namespace State
{
  const int Stop = 0;
  const int MoveTo = 1;
  const int GoHome = 2;

  const std::map<int, std::string> Str = {
      {Stop, "Stop"},
      {MoveTo, "MoveTo"},
      {GoHome, "GoHome"}};
} // namespace State

namespace Command
{
  const int Stop = 0;
  const int MoveTo = 1;
  const int GoHome = 2;

  const std::map<int, std::string> Str = {
      {Stop, "Stop"},
      {MoveTo, "MoveTo"},
      {GoHome, "GoHome"}};
} //namespace Command

struct Data_t
{
  tf2_ros::Buffer *tfBuf;
  tf2_ros::TransformListener *tfListener;

  costmap_2d::Costmap2DROS *local_costmap;
  dwa_local_planner::DWAPlannerROS *local_planner;

  int state;   //state variavle (State::Stop, ...)
  int command; //command variavle (Command::Stop, ...)

  geo_msgs::PointStamped pg;   //global target position
  geo_msgs::Twist u_t;  //control input
  geo_msgs::Pose x_t;   //current pose

  Data_t() : state(State::Stop),
             command(Command::Stop)
  {
    //make transform listener
    tfBuf = new tf2_ros::Buffer();
    tfListener = new tf2_ros::TransformListener(*tfBuf);

    //make local costmap object
    local_costmap = new costmap_2d::Costmap2DROS("local_costmap", *tfBuf);

    //make dwa local planner
    local_planner = new dwa_local_planner::DWAPlannerROS();
    local_planner->initialize("dwa_local_planner", tfBuf, local_costmap);
  }
};

#define POW2(x) pow(x, 2.0)

static double Dist(double x1, double x2, double y1, double y2)
{
  double d = 0.0;
  double dx = x2 - x1;
  double dy = y2 - y1;
  d = sqrt(POW2(dx) + POW2(dy));
  return d;
}

#endif
