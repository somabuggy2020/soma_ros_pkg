#ifndef MOTOR_H
#define MOTOR_H

#include <QObject>
#include <QString>
#include <QDebug>
#include <QSettings>
#include <QDir>

#include <Epos.h>

#include "../../Common/configfilepath.h"
#include "motorinfo.h"

class Motor : public QObject
{
	Q_OBJECT
public:
	explicit Motor(QObject *parent = nullptr);
	explicit Motor(QString name, QString role, QObject *parent = nullptr);
	~Motor();

	int initialize();

	int open();
	void close();

	int moveto(double pos, bool minmax = true, bool immediatery = true);
	int setMaxRPM(unsigned int MaxRPM);
	int recv(MotorInfo::Data_t &d);

private:
	int FindMainHandle();
	int FindSubHandle();
  QString strVCSError(unsigned int error_code);

private:
	bool isUse;

	QString name;
	QString role;
	MotorInfo::Config_t mcfg; //Motor configs

  static void *MainHandle;
  static void *SubHandle;
  void *handle;

	double dTrgPos;
};

#endif // MOTOR_H
