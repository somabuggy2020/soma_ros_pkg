#ifndef DEFINITION_H
#define DEFINITION_H

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <string>
#include <map>

namespace geo_msgs = geometry_msgs;

namespace State
{
const int Stop = 0;
const int MoveTo = 1;
const int Home = 2;

const std::map<int, std::string> Str = {
  {Stop, "Stop"},
  {MoveTo, "MoveTo"},
  {Home, "Home"}};
} // namespace State

namespace Command {
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

  int state;
  int command;

  geo_msgs::Point pg; //global target position
  geo_msgs::Twist u_t;
  geo_msgs::Pose x_t;

  Data_t() :
    state(State::Stop),
    command(Command::Stop) {
    tfBuf = new tf2_ros::Buffer();
    tfListener = new tf2_ros::TransformListener(*tfBuf);
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



