#pragma once

#include "./Motor/motorinfo.h"
#include "./Clutch/clutchinfo.h"
#include "./Rotary/rotaryinfo.h"

/*!
 * \brief The HardwareData class
 */
class HardwareData
{
public:
	ClutchInfo::Data_t clutch;
	RotaryInfo::Data_t rotary;
	MotorInfo::Data_t steering;
	MotorInfo::Data_t rearBrake;
	MotorInfo::Data_t frontBrake;
	MotorInfo::Data_t accel;
	//  MotorInfo::Data_t weedingMechanism;
	//  MotorInfo::Data_t pan;
	//  MotorInfo::Data_t tilt;

	//	RotaryInfo::Data_t rotary;

	struct Accel_Profiles_t
	{
		double PosOffset;
		double PosRegular;
		double PosPush;
		double PushTime;
	} accelProfs;

public:
	HardwareData() {}
	~HardwareData() {}
};
