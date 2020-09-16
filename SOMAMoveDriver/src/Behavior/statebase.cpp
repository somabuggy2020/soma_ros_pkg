#include "statebase.h"

StateBase::StateBase()
{

}

int StateBase::initialize()
{
	return 0;
}

int StateBase::isTransition(Data *data)
{
	TRACE(__FUNCTION__);
	return _isTransition(data);
}

int StateBase::Enter(Data *data)
{
	TRACE(__FUNCTION__);
	return _Enter(data);
}

int StateBase::Process(Data *data)
{
	TRACE(__FUNCTION__);
	return _Process(data);
}

void StateBase::Exit(Data *data)
{
	TRACE(__FUNCTION__);
	return _Exit(data);
}

int StateBase::_isTransition(Data *data)
{
	return 0;
}

int StateBase::_Enter(Data *data)
{
	return 0;
}

int StateBase::_Process(Data *data)
{
	return 0;
}

void StateBase::_Exit(Data *data)
{
}
