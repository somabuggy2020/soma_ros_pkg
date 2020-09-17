#include "traveling.h"

Traveling::Traveling()
{
	T = 0.0;
	max_acc = 6.0;

	//  qlClient = new QLClient();
	//  if(qlClient->initialize() != 0)	exit(1);
}

int Traveling::setThread(QThread *th)
{
	//  qlClient->moveToThread(th);
	return 0;
}

int Traveling::_isTransition(Data *data)
{
	if(data->cmd.mode == Mode::Stop
		 || data->cmd.mode == Mode::Remote::Stop){
		data->mode = data->cmd.mode;
		return State::Stopping;
	}

	//Dangerouse command
	if((data->cmd.mode == Mode::Backward && data->hardware.clutch.Out == Mode::Forward)
		 || (data->cmd.mode == Mode::Forward && data->hardware.clutch.Out == Mode::Backward)
		 || (data->cmd.mode == Mode::Remote::Back && data->hardware.clutch.Out == Mode::Forward)
		 || (data->cmd.mode == Mode::Remote::Go && data->hardware.clutch.Out == Mode::Backward)){
		return State::Stopping;
	}

	//	if(data->cmd.mode == Mode::Forward
	//		 || data->cmd.mode == Mode::Backward
	//		 || data->cmd.mode == Mode::Remote::Go
	//		 || data->cmd.mode == Mode::Remote::Back){
	//	if(data->commRecv.s != 0.0){
	//		if(T >= data->hardware.accelProfs.PushTime){
	//			return State::Stacking;
	//		}
	//	}

	return State::Traveling;
}

int Traveling::_Enter(Data *data)
{
	T = 0.0;
	acc = data->hardware.accelProfs.PosRegular;
	if(data->VeloController == VeloControlMode::RL){
		//    if(qlClient->connect() != 0)	exit(1);
	}

	return 0;
}

int Traveling::_Process(Data *data)
{
	// steering position
	data->hardware.steering.In.pos = data->cmd.steer;

	switch(data->VeloController){
		case VeloControlMode::FIX:
			data->hardware.accel.In.pos = data->hardware.accelProfs.PosRegular;
			break;
		case VeloControlMode::PID:
			data->hardware.accel.In.pos = getAccel_PD(data);
			break;
		case VeloControlMode::RL:
			data->hardware.accel.In.pos= getAccel_QL(data);
			//			data->hardware.accel.In.pos = getAccel_RL_PD(data);
			break;
	}

	return 0;
}

void Traveling::_Exit(Data *data)
{
	T = 0.0;

	//	QLInfo::Send_t send;
	//	send.x_t = (float)data->x.x;
	//	send.y_t = (float)data->x.y;
	//	send.theta_t = (float)data->x.theta;
	//	send.phi_t = (float)data->imuInfo.pitch;
	//	send.psi_t = (float)data->imuInfo.roll;
	//	send.v_t = (float)data->v[0];
	//	send.e_t = (float)data->ev[0];
	//	send.acc_t = (float)data->hardware.accel.Out.pos;
	//	send.Pgain = (float)data->P;
	//	send.v_gps = (float)data->gpsInfo.velocity;
	//	send.v_rot = (float)data->hardware.rotary.v;

	//  qlClient->logout();

	return;
}


double Traveling::getAccel_RL_PD(Data *data)
{
	QLInfo::Send_t send;
	send.x_t = (float)data->X_t.x;
	send.y_t = (float)data->X_t.y;
	send.theta_t = (float)data->X_t.theta;
	//	send.phi_t = (float)data->imuInfo.pitch;
	//	send.psi_t = (float)data->imuInfo.roll;
	send.v_t = (float)data->v[0];
	send.e_t = (float)data->ev[0];
	send.acc_t = (float)data->hardware.accel.Out.pos;
	send.Pgain = (float)data->P;
	//	send.v_gps = (float)data->gpsInfo.v;
	//  send.v_rot = (float)data->hardware.rotary.v;
	//	send.gps_qual = (float)data->gpsInfo.quality;
	QLInfo::Recv_t reply;
	//  qlClient->update(send, reply);
	TRACE(QString("Action:%1").arg(reply.action));

	switch(reply.action){
		case 0: //Keep
			delta_p = 0.0;
			break;
		case 1: //increase
			delta_p = 0.01;
			break;
		case 2: //decreasing
			delta_p = -0.01;
			break;
		case -1:
			qWarning() << "Trial failure :" << reply.action;
			data->P = 0.2;
			return data->hardware.accelProfs.PosRegular;
		default:
			break;
	}

	//PD velocity controller
	data->P += delta_p;
	//calculate gains
	data->Pout = data->P * data->ev[0];
	//	data->Dout = data->D * ((data->ev[0] - data->ev[1]) / data->dt);

	//	qInfo() << "------------------------------------------------";
	//	qInfo() << "velo:" << data->v[0] << "v_ref:" << data->V_ref;
	//	qInfo() << "ev:" << data->ev[0] << "v_err:" << data->V_err;
	//	qInfo() << "KP:" << data->P << "," << "Pout:" << data->Pout;
	//	qInfo() << "KD:" << data->D << "," << "Dout:" << data->Dout;

	double delta_acc = data->Pout /* - data->Dout*/;
	//	qInfo() << "Delta Acc[deg]:" << delta_acc;

	acc += delta_acc;
	//	qInfo() << "acc[deg]:" << acc;

	acc = std::max<double>(data->hardware.accelProfs.PosRegular, acc);
	acc = std::min<double>(acc, data->hardware.accel.max_pos);

	return acc;
}


double Traveling::getAccel_PD(Data *data)
{
	//PD velocity controller
	//calclate gains
	data->Pout = data->P * (data->ev[0] - data->ev[1]);
	data->Dout = data->D / data->dt  * (data->ev[0] - 2 * data->ev[1] + data->ev[2]);

	qInfo() << "------------------------------------------------";
	qInfo() << "velo:" << data->v[0] << "v_ref:" << data->V_ref;
	qInfo() << "ev:" << data->ev[0] << "v_err:" << data->V_err;
	qInfo() << "KP:" << data->P << "," << "Pout:" << data->Pout;
	qInfo() << "KD:" << data->D << "," << "Dout:" << data->Dout;

	double delta_acc = data->Pout + data->Dout;
	qInfo() << "Delta Acc[deg]:" << delta_acc;

	acc += delta_acc;
	qInfo() << "acc[deg]:" << acc;

	acc = std::max<double>(data->hardware.accelProfs.PosRegular, acc);
	acc = std::min<double>(acc, data->hardware.accel.max_pos);

	return acc;
}

double Traveling::getAccel_QL(Data *data)
{
	QLInfo::Send_t send;
	send.x_t = (float)data->X_t.x;
	send.y_t = (float)data->X_t.y;
	send.theta_t = (float)data->X_t.theta;
	//	send.phi_t = (float)data->imuInfo.pitch;
	//	send.psi_t = (float)data->imuInfo.roll;
	send.v_t = (float)data->v[0];
	send.e_t = (float)data->ev[0];
	send.acc_t = (float)data->hardware.accel.Out.pos;
	send.Pgain = (float)data->P;
	//	send.v_gps = (float)data->gpsInfo.v;
	//  send.v_rot = (float)data->hardware.rotary.v;

	QLInfo::Recv_t reply;
	//  qlClient->update(send, reply);
	TRACE(QString("Action:%1").arg(reply.action));

	switch(reply.action){
		case 0: //Keep
			acc += 0.0;
			break;
		case 1: //increase
			acc += 0.1;
			break;
		case 2: //decreasing
			acc -= 0.1;
			break;
		case -1:
			qWarning() << "Trial failure :" << reply.action;
			acc += data->hardware.accelProfs.PosRegular;
			break;
		default:
			break;
	}
	acc = std::max<double>(data->hardware.accelProfs.PosRegular, acc);
	acc = std::min<double>(acc, data->hardware.accel.max_pos);
	qInfo() << "acc[deg]:" << acc;
	return acc;
}
