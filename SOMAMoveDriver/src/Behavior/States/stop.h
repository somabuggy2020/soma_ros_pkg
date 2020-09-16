#ifndef STPO_H
#define STPO_H

//#include "../../Config/config.h"
#include "../../Data/data.h"
#include "../../Hardware/hardware.h"
#include "../statebase.h"

class Stop : public StateBase
{
public:
	Stop();

	// StateBase interface
public:
	int _isTransition(Data *data);
	int _Enter(Data *data);
	int _Process(Data *data);
	void _Exit(Data *data);
};

#endif // STPO_H
