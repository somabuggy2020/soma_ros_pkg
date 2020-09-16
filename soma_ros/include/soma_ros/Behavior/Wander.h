#ifndef WANDER_H
#define WANDER_H

#include <base_local_planner/trajectory_planner_ros.h>
#include <dwa_local_planner/dwa_planner_ros.h>
// #include <teb_local_planner/teb_local_planner_ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <navfn/navfn_ros.h>

#include "soma_ros/Behavior/StateBase/StateBase.h"
#include "soma_ros/Data/Data.h"

class Wander : public StateBase
{
public:
  Wander();
  ~Wander();

  int _Transition(Data *data);
  int _Enter(Data *data);
  int _Process(Data *data);
  int _Exit(Data *data);

private:
  tf::TransformListener *tf;
  costmap_2d::Costmap2DROS *local_costmap;
  // base_local_planner::TrajectoryPlannerROS *local_planner;
  dwa_local_planner::DWAPlannerROS *local_planner;
  // teb_local_planner::TebLocalPlannerROS *local_planner;

  geometry_msgs::PoseStamped fixed_start;
  geometry_msgs::PoseStamped dummy_target;
};


#endif