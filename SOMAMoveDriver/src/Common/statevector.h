#pragma once

#include <QDebug>
#include <QString>

/*!
 * \brief The StateVector class
 */
class StateVector
{
public:
	double x;
	double y;
	double theta;

public:
	StateVector(){
		x = y = theta = 0.0;
	}

	StateVector(double x, double y, double theta){
		this->x = x;
		this->y = y;
		this->theta = theta;
	}

	~StateVector(){}

	friend QDebug operator<< (QDebug dbg, const StateVector x){
		dbg << QString("[%1, %2, %3]").arg(x.x).arg(x.y).arg(x.theta);
		return dbg;
	}
};
