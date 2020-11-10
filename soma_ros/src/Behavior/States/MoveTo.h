#ifndef MOVE_TO_H
#define MOVE_TO_H

#include "soma_ros/Behavior/StateBase/StateBase.h"
#include "soma_ros/Data/Data.h"
#include "dwa_local_planner/dwa_planner.h"

class MoveTo : public StateBase {
 public:
  MoveTo(double _lim_d);
  ~MoveTo();

  int _Transition(Data *data);
  int _Enter(Data *data);
  int _Process(Data *data);
  int _Exit(Data *data);

 private:
  // DWA::Config DwaCfg;
  // PurePursuit::Config PpCfg;

  double lim_d;  // finish distance [m]
  geometry_msgs::PoseStamped fixed_start;
  geometry_msgs::PoseStamped dummy_target;
};

#endif