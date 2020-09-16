#ifndef TRAVELING_H
#define TRAVELING_H

#include <QThread>

//#include "../../Config/config.h"
#include "../../Data/data.h"
#include "../statebase.h"
#include "../QLClient/qlclient.h"

class Traveling : public StateBase
{
public:
	Traveling();
	int setThread(QThread *th);

public:
	int _isTransition(Data *data);
	int _Enter(Data *data);
	int _Process(Data *data);
	void _Exit(Data *data);

public:
	//menber variables
	double T;

private:
	double getAccel_PD(Data *data);
	double getAccel_QL(Data *data);
	double getAccel_RL_PD(Data *data);

private:
	double acc;
	double max_acc;
	double offset;

	double KP; //Pゲイン
	double delta_p;
	double KD; //Dゲイン

	double Pout,Dout; //PDそれぞれの項の出力

	// QLClient *qlClient;
};

#endif // TRAVELING_H
