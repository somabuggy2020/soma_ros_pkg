#include "soma_ros/Behavior/Stop.h"

Stop::Stop() {}

int Stop::_Transition(Data *data)
{
  if (data->action == Action::MoveTo)
    return State::MoveTo;

  if (data->action == Action::Home)
    return State::Home;

  if (data->action == Action::Start)
    return State::Wander;

  return State::Stop;
}

int Stop::_Enter(Data *data)
{
  data->Uin.lambda = 0.0;
  data->Uin.v = 0.0;
  return 0;
}

int Stop::_Process(Data *data)
{
  data->Uin.lambda = 0.0;
  data->Uin.v = 0.0;
  return 0;
}

int Stop::_Exit(Data *data) { return 0; }
