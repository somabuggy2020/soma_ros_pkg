#include "starting.h"

Starting::Starting()
{
	//	cfg = new Config();
	//  RearLowRPM = (long)cfg->getInteger("STARTING", "REAR_LOW_RPM");
	T = 0.0;
}

int Starting::_isTransition(Data *data)
{
	switch(data->cmd.mode){
		case Mode::Stop:
		case Mode::Remote::Stop:
			data->mode = data->cmd.mode;
			return State::Stopping;	//transition to Stopping
	}

	//Dangerouse command
	if((data->cmd.mode == Mode::Backward && data->hardware.clutch.Out == Mode::Forward)
		 || (data->cmd.mode == Mode::Forward && data->hardware.clutch.Out == Mode::Backward)
		 || (data->cmd.mode == Mode::Remote::Back && data->hardware.clutch.Out == Mode::Forward)
		 || (data->cmd.mode == Mode::Remote::Go && data->hardware.clutch.Out == Mode::Backward)){
		return State::Stopping;
	}

	//Transition to Travelling???
	//	if(data->hardware.rotary.pulse[0] > 0) return State::Traveling;
	if(data->v[0] > 0.3) return State::Traveling;

	return State::Starting;
}

int Starting::_Enter(Data *data)
{
	//Change clutch direction
	if(data->cmd.mode == Mode::Forward
		 || data->cmd.mode == Mode::Remote::Go){
		data->hardware.clutch.In = ClutchInfo::Forward;
	}

	if(data->cmd.mode == Mode::Backward
		 || data->cmd.mode == Mode::Remote::Back){
		data->hardware.clutch.In = ClutchInfo::Backward;
	}

	data->hardware.steering.In.pos = data->cmd.steer;
	data->hardware.rearBrake.In.rpm = RearLowRPM;
	data->hardware.accel.In.rpm = 250;

	T = 0.0;
	return 0;
}

int Starting::_Process(Data *data)
{
	//Wait for clutch state change
	//  if(data->hardware.clutch.In != data->hardware.clutch.Out) return 0;

	data->hardware.steering.In.pos = data->cmd.steer;

	data->hardware.rearBrake.In.pos = MotorPos::Min;
	data->hardware.frontBrake.In.pos = MotorPos::Min;
	data->hardware.accel.In.pos = data->hardware.accelProfs.PosRegular;

	//  if(data->hardware.rotary.v <= 0.3
	//     && (-0.1 <= data->hardware.rearBrake.Out.pos && data->hardware.rearBrake.Out.pos <= 0.1)){
	//    T += data->dt;
	//  }

	if(T >= data->hardware.accelProfs.PushTime){
		data->hardware.accel.In.rpm = 10;
		data->hardware.accel.In.pos = data->hardware.accelProfs.PosPush;
	}

	return 0;
}

void Starting::_Exit(Data *data)
{
	T = 0.0;
	data->hardware.rearBrake.In.rpm = MotorRPM::Default;
	data->hardware.accel.In.rpm = MotorRPM::Default;
}


