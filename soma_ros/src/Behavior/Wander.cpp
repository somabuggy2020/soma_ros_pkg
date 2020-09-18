#include "soma_ros/Behavior/Wander.h"

Wander::Wander()
{

}

Wander::~Wander()
{
}

int Wander::_Transition(Data *data)
{
}

int Wander::_Enter(Data *data) {
  return 0;
}

int Wander::_Process(Data *data)
{
  // Finite State Machine
  int new_state = states[data->state]->Transition(data);

  if (new_state != data->state)
  {
      states[data->state]->Exit(data);
      states[new_state]->Enter(data);
      data->state = new_state;
  }
  else
      states[data->state]->Process(data);

  return 0;
}

int Wander::_Exit(Data *data) {}
