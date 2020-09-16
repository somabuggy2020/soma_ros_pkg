#include "clutch.h"

#define IS_TRACE 0
#if IS_TRACE==1
#define TRACE(msg) qDebug()<<msg
#else
#define TRACE(msg)
#endif

#define CONFIG_PATH "../resource/config_3G.ini"
#define COMM 15

Clutch::Clutch(QObject *parent)
	: QObject(parent)
{
}

Clutch::~Clutch()
{
}

int Clutch::initialize()
{
	QSettings *cfg = new QSettings(CONFIG_FILE_PATH,
																 QSettings::IniFormat,
																 this);
	cfg->beginGroup("USE");
	isUse = cfg->value("CLUTCH").toBool();
	cfg->endGroup();

	cfg->beginGroup("CLUTCH");
	IP = cfg->value("IP").toString();
	udp_recv_port = cfg->value("UDP_RECV_PORT").toInt();
	udp_send_port = cfg->value("UDP_SEND_PORT").toInt();
	cfg->endGroup();

	qInfo() << "Init." << IP;
	qInfo() << "SendPort:" << udp_send_port;
	qInfo() << "RecvPort:" << udp_recv_port;

	return 0;
}

void Clutch::setThread(QThread *th)
{
	sock_send->moveToThread(th);
	sock_recv->moveToThread(th);
}

int Clutch::open()
{
	sock_send = new QUdpSocket();
	sock_recv = new QUdpSocket();

	sock_recv->bind(udp_recv_port);
	return 0;
}

void Clutch::close()
{
	sock_send->close();
	sock_recv->close();

	return;
}

int Clutch::set(ClutchInfo::Data_t &data)
{
	int ret = -1;

	TRACE(QString("In: %1").arg(data.In));

	switch(data.In){
		case ClutchInfo::Forward:
			ret = Forward();
			break;
		case ClutchInfo::Backward:
			ret = Backward();
			break;
		case ClutchInfo::Free:
			ret = Free();
			break;
		default:
			break;
	}

	return 0;
}

int Clutch::recv(ClutchInfo::Data_t &data)
{
	if(!isUse){
		data.Out = data.In;
		return 0;
	}

	bool ret = false;

	if(sock_recv->waitForReadyRead(33)){
		while(sock_recv->bytesAvailable() > 0){
			int recv; //Integer type 4byte date

			sock_recv->readDatagram((char*)&recv,
															sizeof(int));

			data.Out = recv;
			this->state = data.Out;
		}
	}
	else{
		//		qWarning() << "Not ready to read";
	}

	return 0;
}


int Clutch::Forward(){
	int send = ClutchInfo::Forward;
	sock_send->writeDatagram((char*)&send,
													 sizeof(int),
													 QHostAddress(IP),
													 udp_send_port);
	return 0;
}
int Clutch::Backward(){
	int send = ClutchInfo::Backward;
	sock_send->writeDatagram((char*)&send,
													 sizeof(int),
													 QHostAddress(IP),
													 udp_send_port);
	return 0;
}
int Clutch::Free(){
	int send = ClutchInfo::Free;
	sock_send->writeDatagram((char*)&send,
													 sizeof(int),
													 QHostAddress(IP),
													 udp_send_port);
	return 0;
}
