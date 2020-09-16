#ifndef STACKING_H
#define STACKING_H

#include <QObject>
#include <QString>
#include <QDebug>

#include "../statebase.h"

class Stacking : public StateBase
{
public:
	explicit Stacking();

	// StateBase interface
protected:
	int _isTransition(Data *data) override;
	int _Enter(Data *data) override;
	int _Process(Data *data) override;
	void _Exit(Data *data) override;

private:
	long rot_pulse;
};

#endif // STACKING_H
