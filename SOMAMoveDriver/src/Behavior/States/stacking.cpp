#include "stacking.h"

Stacking::Stacking()
{
	rot_pulse = 0;
}

int Stacking::_isTransition(Data *data)
{
	if(data->cmd.mode == Mode::Stop
		 || data->cmd.mode == Mode::Remote::Stop){
		return State::Stopping;
	}
	//	if(data->commRecv.s == 0.0){
	//		return State::Stopping;
	//	}

	if(data->hardware.rotary.pulse[0] - rot_pulse > 0){
		return State::Traveling;
	}

	return State::Stacking;
}

int Stacking::_Enter(Data *data)
{
	data->hardware.accel.In.pos = data->hardware.accelProfs.PosPush;
	rot_pulse = data->hardware.rotary.pulse[0];
	return 0;
}

int Stacking::_Process(Data *data)
{
	return 0;
}

void Stacking::_Exit(Data *data)
{
	rot_pulse = 0;
	return;
}
