#include "rotary.h"

Rotary::Rotary(QObject *parent)
	: QObject(parent)
{
}

/*!
 * \brief Rotary::initialize
 * \return
 */
int Rotary::initialize()
{
	QSettings *cfg = new QSettings(CONFIG_FILE_PATH,
																 QSettings::IniFormat,
																 this);

	cfg->beginGroup("USE");
	isUse = cfg->value("ROTARY").toBool();
	cfg->endGroup();

	cfg->beginGroup("ROTARY");
	IP = cfg->value("IP").toString();
	port = cfg->value("PORT").toInt();

	udp_recv_port = cfg->value("UDP_RECV_PORT").toInt();
	udp_send_port = cfg->value("UDP_SEND_PORT").toInt();
	cfg->endGroup();

	qInfo() << "Init." << IP;
	qInfo() << "SendPort:" << udp_send_port;
	qInfo() << "RecvPort:" << udp_recv_port;
	return 0;
}

void Rotary::setThread(QThread *th)
{
	sock_send->moveToThread(th);
	sock_recv->moveToThread(th);
}

/*!
 * \brief Rotary::connect
 * \return
 */
int Rotary::open()
{
	sock_send = new QUdpSocket();
	sock_recv = new QUdpSocket();
	sock_recv->bind(udp_recv_port);
	return 0;
}

/*!
 * \brief Rotary::disconnect
 */
void Rotary::close()
{
	sock_send->close();
	sock_recv->close();
	return;
}

/*!
 * \brief Rotary::CountReset
 * \return
 */
int Rotary::CountReset()
{
	return 0;
}

int Rotary::recv(RotaryInfo::Data_t &data)
{
	if(!isUse) return 0;
	bool ret = false;

	//Request
	//	ret = isWritable();
	//	if(!ret){
	//		qWarning() << errorString();
	//		return -1;
	//	}

	//	int comm = RotaryInfo::ReqCode::All;
	//	write((char*)(&comm), sizeof(int));

	//	ret = waitForBytesWritten();
	//	if(!ret){
	//		qWarning() << errorString();
	//		return -1;
	//	}


	//struct for data receive
	struct Recv_t
	{
		long pulse;
		//		double dis;
		double velo;
	} recv;

	//	int size = read((char*)(&d), sizeof(Recv_t));
	//qInfo() << sock_recv->bytesAvailable();
	if(sock_recv->waitForReadyRead(33)){
		//		if(sock_recv->bytesAvailable() > 0){
		while(sock_recv->bytesAvailable() > 0){
			sock_recv->readDatagram((char*)&recv, sizeof(Recv_t));
		}
		//		else return 0;
		//				}
	}
	else{
		return 0;
	}

	data.pulse[1] = data.pulse[1];
	data.pulse[0] = recv.pulse;
	//	data.d[1] = data.d[0];
	//	data.d[0] = recv.dis;
	data.v = recv.velo;

	return 0;
}



