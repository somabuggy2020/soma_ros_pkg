#include "stopping.h"

Stopping::Stopping()
{
  T += 0.0;
}

int Stopping::_isTransition(Data *data)
{
  //Transition to Stop
  if(T >= 1.0) return State::Stop;

  return State::Stopping;
}

int Stopping::_Enter(Data *data)
{
  data->hardware.steering.In.pos = MotorPos::Home;
  data->hardware.accel.In.pos = MotorPos::Min;
  data->hardware.rearBrake.In.pos = MotorPos::Max;
  T = 0.0;
  return 0;
}

int Stopping::_Process(Data *data)
{
  T += data->dt;
  //	if(data->hardware.rotary.v <= 0.3) T += data->dt;

  return 0;
}

void Stopping::_Exit(Data *data)
{
  T = 0.0;
  data->hardware.frontBrake.In.pos = MotorPos::Min;
}
