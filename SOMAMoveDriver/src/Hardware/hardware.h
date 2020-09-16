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
	explicit Hardware(QObject *parent = nullptr);		//�R���X�g���N�^
	~Hardware();

	int initialize();
	int setThread(QThread *th);
	void finalize();

	void recv(Data *data);
	void send(Data *data);

private:
	//	void startWeedingProc(Data &data);	//�X���b�h�J�n�֐�
	//	void stopWeedingProc(Data &data);	//�X���b�h��~�֐�
	//	void weedingProc(Data &data);		//�X���b�h���Ԋ֐�

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

	//�����@�\����͕ʃX���b�h�Ŏ��s
	//	bool isWeedingProc;				//�X���b�h���[�v�t���O
	//	boost::thread thWeedingProc;	//�X���b�h����
};

