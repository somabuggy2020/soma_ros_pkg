#ifndef STOPPING_H
#define STOPPING_H

//#include "../../Config/config.h"
#include "../../Data/data.h"
#include "../../Hardware/hardware.h"
#include "../statebase.h"

class Stopping : public StateBase
{
public:
	Stopping();

	// StateBase interface
public:
	int _isTransition(Data *data);
	int _Enter(Data *data);
	int _Process(Data *data);
	void _Exit(Data *data);

private:
	double T;
};

#endif // STOPPING_H
