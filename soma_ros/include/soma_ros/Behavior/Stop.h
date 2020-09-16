#ifndef STOP_H
#define STOP_H

#include "soma_ros/Behavior/StateBase/StateBase.h"
#include "soma_ros/Data/Data.h"

class Stop : public StateBase {
 public:
  Stop();

  int _Transition(Data *data);
  int _Enter(Data *data);
  int _Process(Data *data);
  int _Exit(Data *data);
};

#endif