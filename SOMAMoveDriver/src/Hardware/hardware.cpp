#include "hardware.h"

#define IS_TRACE 0
#if IS_TRACE==1
#define TRACE(msg) qDebug()<<msg
#else
#define TRACE(msg)
#endif

Hardware::Hardware(QObject *parent)
	: QObject(parent)
{
	clutch = new Clutch();
	rotary = new Rotary();
	steering		= new Motor(MotorInfo::Names::Steering,		MotorInfo::Roles::Drive);
	rearBrake		= new Motor(MotorInfo::Names::RearBrake,	MotorInfo::Roles::Drive);
	frontBrake	= new Motor(MotorInfo::Names::FrontBrake, MotorInfo::Roles::Drive);
	accel				= new Motor(MotorInfo::Names::Accel,			MotorInfo::Roles::Drive);

	//  pan		= new Motor(MotorInfo::Names::Pan, MotorInfo::Roles::Vision);
	//  tilt	= new Motor(MotorInfo::Names::Tilt, MotorInfo::Roles::Vision);
}

Hardware::~Hardware()
{
}

int Hardware::initialize()
{
	TRACE("Actuator initialize");

	if(clutch->initialize() == -1) return -1;
	if(rotary->initialize() == -1) return -1;
	if(steering->initialize() == -1) return -1;
	if(rearBrake->initialize() == -1) return -1;
	if(frontBrake->initialize() == -1) return -1;
	if(accel->initialize() == -1) return -1;

	//  pan->initialize();
	//  tilt->initialize();

	if(clutch->open() == -1) return -1;
	if(rotary->open() == -1) return -1;
	if (steering->open() == -1)		return -1;
	if (rearBrake->open() == -1)		return -1;
	if (frontBrake->open() == -1)	return -1;
	if (accel->open() == -1)			return -1;

	//	if (weedingMechanism.open() == -1)	return -1;
	//  if (pan->open() == -1)	return -1;
	//  if (tilt->open() == -1)	return -1;

	//  int ret = rotary->CountReset();

	//  tilt->move(15.0);

	return 0;
}

int Hardware::setThread(QThread *th)
{
	clutch->setThread(th);
	rotary->setThread(th);
	clutch->moveToThread(th);
	rotary->moveToThread(th);
	return 0;
}

void Hardware::finalize()
{
	TRACE("Hardware finalize");

	TRACE("Actuator set home position");
	steering->moveto(0.0);
	rearBrake->moveto(0.0);
	frontBrake->moveto(0.0);
	accel->moveto(0.0);
	//  pan->move(0.0);
	//  tilt->move(0.0);
	//	weedingMechanism.setTrgtPos(0.0);

	TRACE("Actuator close");
	clutch->close();
	steering->close();
	rearBrake->close();
	frontBrake->close();
	accel->close();
	//  pan->close();
	//  tilt->close();
	//	weedingMechanism.close();
	//	Pan.close();
	//	Tilt.close();

	return;
}


void Hardware::recv(Data *data)
{
	//  int ret = 0;

	TRACE("Clutch recv");
	clutch->recv(data->hardware.clutch);
	//  if(ret == -1) qWarning() << "Clutch recv error";

	TRACE("Rotary recv");
	rotary->recv(data->hardware.rotary);
	//  if(ret == -1) qWarning() << "Rotary recv error";

	TRACE("Actuators data recieve");
	steering->recv(data->hardware.steering);
	rearBrake->recv(data->hardware.rearBrake);
	frontBrake->recv(data->hardware.frontBrake);
	accel->recv(data->hardware.accel);
	//  pan->recv(data->hardware.pan);
	//  tilt->recv(data->hardware.tilt);

	return;
}

void Hardware::send(Data *data)
{
	//Clutch
	TRACE("Clutch set");
	clutch->set(data->hardware.clutch);

	//Rotary
	//  if(data->cmd.isRotReset){
	//    TRACE("Rotary count reset");
	//    rotary->CountReset();
	//    data->cmd.isRotReset = false;
	//  }

	//Each motor
	steering->setMaxRPM(data->hardware.steering.In.rpm);
	rearBrake->setMaxRPM(data->hardware.rearBrake.In.rpm);
	frontBrake->setMaxRPM(data->hardware.frontBrake.In.rpm);
	accel->setMaxRPM(data->hardware.accel.In.rpm);

	steering->moveto(data->hardware.steering.In.pos);
	rearBrake->moveto(data->hardware.rearBrake.In.pos);
	frontBrake->moveto(data->hardware.frontBrake.In.pos);
	accel->moveto(data->hardware.accel.In.pos);

	return;
}
