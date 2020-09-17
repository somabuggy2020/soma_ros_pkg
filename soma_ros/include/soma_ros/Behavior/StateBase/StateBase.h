#ifndef STATEBASE_H
#define STATEBASE_H

#include <string>
#include <map>

#include "soma_ros/Data/Data.h"

class StateBase {
public:
  StateBase();
  virtual ~StateBase();

  int Transition(Data *data);
  int Enter(Data *data);
  int Process(Data *data);
  int Exit(Data *data);

protected:
  virtual int _Transition(Data *data) = 0;
  virtual int _Enter(Data *data) = 0;
  virtual int _Process(Data *data) = 0;
  virtual int _Exit(Data *data) = 0;
};

#endif
