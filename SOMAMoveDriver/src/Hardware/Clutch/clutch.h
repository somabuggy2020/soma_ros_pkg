#pragma once

#include <QObject>
#include <QString>
#include <QDebug>
#include <QUdpSocket>
#include <QTcpSocket>
#include <QSettings>
#include <QThread>
#include <QDateTime>

#include "../../Common/configfilepath.h"
#include "clutchinfo.h"

class Clutch : public QObject
{
	Q_OBJECT
public:
	explicit Clutch(QObject *parent = nullptr);
	~Clutch();

	int initialize();
	void setThread(QThread *th);

	int open();
	void close();

	void disconnect();

	int set(ClutchInfo::Data_t &data);
	int recv(ClutchInfo::Data_t &data);

private:
	int Forward();
	int Backward();
	int Free();

private:
	bool isUse;
	QString IP;
	int port;

	int udp_recv_port, udp_send_port;
	QUdpSocket *sock_recv, *sock_send;

	int state;
};
