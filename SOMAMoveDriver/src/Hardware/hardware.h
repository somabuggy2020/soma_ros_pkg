#pragma once

#include <QObject>
#include <QDebug>
#include <QString>

#include "../Data/data.h"
#include "Motor/motor.h"
#include "Clutch/clutch.h"
#include "Rotary/rotary.h"

class Hardware : public QObject
{
	Q_OBJECT

public:
	explicit Hardware(QObject *parent = nullptr);		//コンストラクタ
	~Hardware();

	int initialize();
	int setThread(QThread *th);
	void finalize();

	void recv(Data *data);
	void send(Data *data);

private:
	//	void startWeedingProc(Data &data);	//スレッド開始関数
	//	void stopWeedingProc(Data &data);	//スレッド停止関数
	//	void weedingProc(Data &data);		//スレッド実態関数

private:

public:

	Motor *steering;
	Motor *rearBrake;
	Motor *frontBrake;
	Motor *accel;
	Clutch *clutch;
	Rotary *rotary;

	//  Motor *pan;
	//  Motor *tilt;
	//  Motor *weedingMechanism;

	//下刈機構制御は別スレッドで実行
	//	bool isWeedingProc;				//スレッドループフラグ
	//	boost::thread thWeedingProc;	//スレッド実態
};

