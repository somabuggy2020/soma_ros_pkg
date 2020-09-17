#include "soma_ros/Behavior/StateBase/StateBase.h"

StateBase::StateBase() {}

StateBase::~StateBase() {}

int StateBase::Transition(Data *data) { return _Transition(data); }
int StateBase::Enter(Data *data) { return _Enter(data); }
int StateBase::Process(Data *data) { return _Process(data); }
int StateBase::Exit(Data *data) { return _Exit(data); }

// pure virtual functions
int StateBase::_Transition(Data *data) { return 0; }
int StateBase::_Enter(Data *data) { return 0; }
int StateBase::_Process(Data *data) { return 0; }
int StateBase::_Exit(Data *data) { return 0; }
