#ifndef MOVE_TO_H
#define MOVE_TO_H

#include <tf/tf.h>
#include <dwa_local_planner/dwa_planner.h>
#include "../StateBase/StateBase.h"

class MoveTo : public StateBase {
 public:
  MoveTo(double _lim_d);
  ~MoveTo();

  int _Transition(Data_t *data);
  int _Enter(Data_t *data);
  int _Process(Data_t *data);
  int _Exit(Data_t *data);

 private:
  // DWA::Config DwaCfg;
  // PurePursuit::Config PpCfg;

  double lim_d;  // finish distance [m]
  geometry_msgs::PoseStamped fixed_start;
  geometry_msgs::PoseStamped fixed_target;
};

#endif
