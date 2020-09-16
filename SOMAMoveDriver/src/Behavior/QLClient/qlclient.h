#ifndef QLCLIENT_H
#define QLCLIENT_H

#include <QObject>
#include <QDebug>
#include <QString>
#include <QSettings>
#include <QTcpSocket>

namespace QLInfo
{
	struct Send_t
	{
		float x_t, y_t, theta_t, phi_t, psi_t;
		float v_t, e_t, acc_t;
		float Pgain;
		float v_gps, v_rot, gps_qual;
	};

	struct Recv_t
	{
		int action;
		float size;
	};
}
class QLClient : public QTcpSocket
{
	Q_OBJECT
public:
	explicit QLClient(QObject *parent = nullptr);

	int initialize();

	int connect();
	void logout();

	int update(QLInfo::Send_t d, QLInfo::Recv_t &rep);

signals:

public slots:

private:
	bool isUse;
	QString IP;
	int port;

};

#endif // QLCLIENT_H
