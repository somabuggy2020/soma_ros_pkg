#pragma once

#include <QObject>
#include <QString>
#include <QDebug>
#include <QTcpSocket>
#include <QUdpSocket>
#include <QSettings>

#include "../../Common/configfilepath.h"
#include "rotaryinfo.h"

class Rotary : public QObject
{
	Q_OBJECT
public:
	explicit Rotary(QObject *parent = nullptr);

	int initialize();
	void setThread(QThread *th);

	int open();
	void close();

	int CountReset();
	int recv(RotaryInfo::Data_t &data);

private:
	bool isUse;
	QString IP;
	int port;

	int udp_recv_port, udp_send_port;
	QUdpSocket *sock_recv, *sock_send;
};
