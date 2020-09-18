#ifndef WANDER_H
#define WANDER_H

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
  std::map<int, StateBase *> states;
};


#endif
