#ifndef HOME_H
#define HOME_H

#include "soma_ros/Behavior/MotionControl/PurePursuit.h"
#include "soma_ros/Behavior/StateBase/StateBase.h"
#include "soma_ros/Data/Data.h"

class Home : public StateBase {
 public:
  Home(double _lim_d);
  ~Home() {}

  int _Transition(Data *data);
  int _Enter(Data *data);
  int _Process(Data *data);
  int _Exit(Data *data);

 private:
  double lim_d;  // finish distance [m]
  PurePursuit::Config PpCfg;
};

#endif
