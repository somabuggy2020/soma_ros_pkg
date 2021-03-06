#ifndef DATA_H
#define DATA_H

#include <costmap_2d/costmap_2d_ros.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <navfn/navfn_ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Pose.h>

#include <QtCore/QList>
#include <QtCore/QPointF>
#include <iostream>
#include <map>
#include <string>

#include "GeoCoord.h"
#include "soma_ros/common.h"

struct ControlInput_t
{
  double lambda, v;

  ControlInput_t() : lambda(0.0), v(0.0) {}

  double yawrate() { return v * sin(lambda) / (double)WHEEL_BASE; }

  void from(double v, double omega)
  {
    this->v = v;

    if (v < -0.0001 || 0.0001 < v)
    {
      this->lambda = asin(omega * (double)WHEEL_BASE / v);
    }
    else
    {
      this->lambda = 0;
    }

    return;
  }

  friend std::ostream &operator<<(std::ostream &out, ControlInput_t &u)
  {
    out << "U=[" << u.lambda << "," << u.v << "]";
    return out;
  }
};

// struct StateVector_t
// {
//   double x, y, theta;

//   StateVector_t() : x(0.0), y(0.0), theta(0.0) {}

//   StateVector_t motion(ControlInput_t u, float dt)
//   {
//     StateVector_t _x;

//     _x.x = x - u.v / u.yawrate() * sin(theta) +
//            u.v / u.yawrate() * sin(theta + u.yawrate() * dt);
//     _x.y = y + u.v / u.yawrate() * cos(theta) -
//            u.v / u.yawrate() * cos(theta + u.yawrate() * dt);
//     _x.theta = theta + u.yawrate() * dt;

//     return _x;
//   }

//   friend std::ostream &operator<<(std::ostream &out, StateVector_t &x)
//   {
//     out << "X=[" << x.x << "," << x.y << "," << x.theta << "]";
//     return out;
//   }
// };

class Data
{
public:
  Data();
  ~Data();

public:
  int state, last_state;
  int action;

  geometry_msgs::Pose Xt;    //state vector (slam /odom)
  geometry_msgs::Pose Xt_DR; //state vector (calculated dead recogning)

  int clutch;   //clutch direction state (1:foward, 2:reverse)

  float vt; // Current robot linear velocity (x-axis)
  float wt; // Current robot angular velocity (round z-axis)

  int np; // number of neighborfood points

  geometry_msgs::Point Pg; // target point in global coordinate system
  ControlInput_t Uin;      // Control input vector

  // GeoCoord geocoord; // gps geo coordinate (latitude, longitude)

  tf::TransformListener *tf;
  tf2_ros::Buffer *tfBuf;
  tf2_ros::TransformListener *tf_2;

  costmap_2d::Costmap2DROS *local_costmap;
  dwa_local_planner::DWAPlannerROS *local_planner;
};

namespace State
{
const int Stop = 0;
const int MoveTo = 1; // for Matsumoto
const int Home = 2;   // for Matsumoto

// for wander behavior
const int Wander = 3;

namespace WanderSubState {
}

const std::map<int, std::string> Str = {
  {Stop, "Stop"}, {MoveTo, "MoveTo"}, {Home, "Home"}, {Wander, "Wander"}};
} // namespace State

namespace Action
{
const int Stop = 0;
const int Start = 1; //Autonomous
const int MoveTo = 2;
const int Home = 3;

const std::map<int, std::string> Str = {
  {Stop, "Stop"}, {Start, "Start"},{MoveTo, "MoveTo"}, {Home, "Home"}};
} // namespace Action

#endif
