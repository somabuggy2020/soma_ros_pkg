#include "xbox.h"


#define IS_TRACE 0
#if IS_TRACE == 1
#define TRACE(msg) qDebug()<<msg;
#else
#define TRACE(msg)
#endif

Xbox::Xbox(QObject *parent) : QObject(parent)
{
	GamepadInit();

	if(GamepadIsConnected(GAMEPAD_0) == GAMEPAD_FALSE){
		qWarning() << "Xbox not available";
	}

	qInfo() << "Xbox available";


}

void Xbox::recv(Data *data)
{
	GamepadUpdate();

	//flag shift
	_isRemote = data->isRemote;

	if(GamepadButtonTriggered(GAMEPAD_0, BUTTON_START)){
		TRACE("Triggered START");
		data->isRemote = !data->isRemote; //switch remote of not
	}





	if(!data->isRemote && _isRemote){
		data->cmd.mode = Mode::Stop;
		return;
	}

	if(data->isRemote && !_isRemote){
		data->cmd.mode = Mode::Remote::Stop;
		return;
	}

	if(!data->isRemote) return;

	//check A, B, X, Y
	if(GamepadButtonDown(GAMEPAD_0, BUTTON_A)
		 == GamepadButtonDown(GAMEPAD_0, BUTTON_B)){
		data->cmd.mode = Mode::Remote::Stop;
	}
	else if(GamepadButtonDown(GAMEPAD_0, BUTTON_A) == GAMEPAD_TRUE
					&& GamepadButtonDown(GAMEPAD_0, BUTTON_B) == GAMEPAD_FALSE){
		data->cmd.mode = Mode::Remote::Go;
	}
	else if(GamepadButtonDown(GAMEPAD_0, BUTTON_A) == GAMEPAD_FALSE
					&& GamepadButtonDown(GAMEPAD_0, BUTTON_B) == GAMEPAD_TRUE){
		data->cmd.mode = Mode::Remote::Back;
	}

	//check ten key
	if(GamepadButtonDown(GAMEPAD_0, BUTTON_DPAD_RIGHT)
		 == GamepadButtonDown(GAMEPAD_0, BUTTON_DPAD_LEFT)){
		data->cmd.steer = 0.0;
	}
	else if(GamepadButtonDown(GAMEPAD_0, BUTTON_DPAD_RIGHT) == GAMEPAD_TRUE
					&& GamepadButtonDown(GAMEPAD_0, BUTTON_DPAD_LEFT) == GAMEPAD_FALSE){
		data->cmd.steer	= -15.0;
		if(GamepadButtonDown(GAMEPAD_0, BUTTON_RIGHT_SHOULDER) == GAMEPAD_TRUE){
			data->cmd.steer	= -25.0;
		}
	}
	if(GamepadButtonDown(GAMEPAD_0, BUTTON_DPAD_RIGHT) == GAMEPAD_FALSE
		 && GamepadButtonDown(GAMEPAD_0, BUTTON_DPAD_LEFT) == GAMEPAD_TRUE){
		data->cmd.steer	= 15.0;
		if(GamepadButtonDown(GAMEPAD_0, BUTTON_RIGHT_SHOULDER) == GAMEPAD_TRUE){
			data->cmd.steer	= 25.0;
		}
	}
}
