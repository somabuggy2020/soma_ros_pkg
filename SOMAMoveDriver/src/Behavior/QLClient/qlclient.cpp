#include "qlclient.h"

#define IS_TRACE 0
#if IS_TRACE == 1
#define TRACE(msg) qDebug()<<msg
#elif IS_TRACE == 0
#define TRACE(msg)
#endif

#define CONFIG_PATH "../resource/config_2G.ini"

QLClient::QLClient(QObject *parent)
	: QTcpSocket(parent)
{
}


int QLClient::initialize()
{
	qInfo() << "Initialize";
	QSettings *cfg = new QSettings(CONFIG_PATH,
																 QSettings::IniFormat,
																 this);
	cfg->beginGroup("USE");
	isUse = cfg->value("QL").toBool();
	cfg->endGroup();
	cfg->beginGroup("QL");
	IP = cfg->value("IP").toString();
	port = cfg->value("PORT").toInt();
	cfg->endGroup();
	qInfo() << "Init. QL" << QString("%1:%2").arg(IP).arg(port);

	return 0;
}

int QLClient::connect()
{

	if(!isUse) return 0;

	connectToHost(IP, port);
	if(!waitForConnected(1000)){
		qCritical() << "QL" << errorString();
		return -1;
	}
	qInfo() << "Connect succeeded";
	return 0;
}

void QLClient::logout()
{
	if(!isUse) return;
	if(isOpen()) disconnectFromHost();
	return;
}

int QLClient::update(QLInfo::Send_t d, QLInfo::Recv_t &rep)
{
	qInfo() << "UPDATE";
	if(!isUse) return 0;

	bool ret = false;

	ret = isWritable();
	if(!ret){
		TRACE("Not writable");
		exit(1);
	}

	int size = write((char*)(&d), sizeof(QLInfo::Send_t));
	TRACE(QString("Send size:%1").arg(size));
	if(size == -1){
		qDebug() << errorString();
		exit(1);
	}

	ret = waitForBytesWritten();
	flush();

	ret = waitForReadyRead(2500);
	if(!ret){
		qDebug() << errorString();
		exit(1);
	}

	//	qDebug() << "Buf :" << bytesAvailable();
	size = read((char*)(&rep), sizeof(QLInfo::Recv_t));
	(&rep)->size = size;
	TRACE(QString("Recv size:%1").arg(size));

	if(size == -1){
		qDebug() << errorString();
		exit(1);
	}
	return 0;
}
