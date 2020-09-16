#pragma once

namespace RotaryInfo
{
	struct Data_t
	{
		long pulse[2];
		double v;
		double d[2];
		double T;

		Data_t() : pulse(), v(), d(), T() {}
	};

	namespace ReqCode
	{
		const int Pulse = 1;
		const int Velocity = 2;
		const int Distance = 3;
		const int All = 4;		//pulse,velocity,distance
		const int Reset = 5;	//pulse count reset
	}
}

