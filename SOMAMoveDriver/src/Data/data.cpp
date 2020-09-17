#include "data.h"

Data::Data(QObject *parent)
	: QObject(parent)
{
	cfg = new QSettings();

	st = QDateTime::currentDateTime();
	ct = QDateTime::currentDateTime();

	state = last_state = State::Init;
	mode = Mode::Stop;

	//	purePursuit = new PurePursuit();
	//	gPath = new Path();
	//  curIdx = 0;

	v[0] = v[1] = v[2] = 0.0;
	ev[0] = ev[1] = ev[2] = 0.0;

	//	gpsInfo.offset.setX(cfg->getDouble("GPS", "OFFSET_X"));
	//	gpsInfo.offset.setY(cfg->getDouble("GPS", "OFFSET_Y"));

	isRemote = false;
	isCommClient = false;
	isLog = false;

	//make logs top directory
	qInfo() << "Make Log directory";
	QDir dir;
	dir.mkdir("Logs");
	//	dir.mkdir("Logs/GPS");
	log = new QFile();
	out = new QTextStream();
	//	logGPS = new QFile();
	//	outGPS = new QTextStream();

	//Hardware profiles
	hardware.accelProfs.PosOffset		= 0.0;
	hardware.accelProfs.PosRegular	= 6.0;
	hardware.accelProfs.PosPush			= 12.0;
	hardware.accelProfs.PushTime		= 2.0;

	VeloController = VeloControlMode::FIX;
	//  P = cfg->getDouble("VELOCITY_CONTROL", "P");
	//  D = cfg->getDouble("VELOCITY_CONTROL", "D");

	V_err = 0.0;
	Pout = 0.0;
	Dout = 0.0;
}

Data::~Data()
{
}

//void Data::setNaviServerRecv(NaviServerInfo::Recv_t d){this->naviRecv = d;}
//void Data::setGlobalReferencePoint(QPointF pg){this->pg = pg;}

void Data::updateState()
{
	//(x_t, y_t) estimation
	//	if(gpsInfo.isConnect){
	//		double dLat = gpsInfo.geoCoord.latitude() - GPSInfo::ORIGIN::LAT;
	//		double dLon = gpsInfo.geoCoord.longitude() - GPSInfo::ORIGIN::LON;
	//		this->X_t.x = dLon*GPSInfo::K::LON;
	//		this->X_t.y = dLat*GPSInfo::K::LAT;
	//	}
	//	else{
	this->X_t.x = 0.0;
	this->X_t.y = 0.0;
	//	}

	//	this->X_t.x += gpsInfo.offset.x();
	//	this->X_t.y += gpsInfo.offset.y();

	//	double _theta = qDegreesToRadians(imuInfo.yaw);
	//	_theta += qDegreesToRadians(imuInfo.offset_yaw);
	//	this->X_t.theta = atan2(sin(_theta), cos(_theta));

	//	this->X_t.x = floorf(this->X_t.x*100.0)/100.0;
	//	this->X_t.y = floorf(this->X_t.y*100.0)/100.0;
	//	this->X_t.theta = floorf(this->X_t.theta*100.0)/100.0;


	//velocity estimation
	v[2] = v[1];
	v[1] = v[0];
	//  v[0] = hardware.rotary.v;

	//	v[2] = v[1];
	//	v[1] = v[0];
	//	v[0] = gpsInfo.v;
	//	v[0] = floorf(v[0]*100.0)/100.0; //[m/s]

	//	v[0] = (v[1] + v[2] + gpsInfo.velocity) / 3;
	//	v[0] = v[1] * 0.9 + gpsInfo.velocity * 0.1;

	//	v[0] = hardware.rotary.v;
	//	ev[2] = ev[1];
	//	ev[1] = ev[0];
	//	ev[0] = V_ref - v[0];
	//	ev[0] = floorf(ev[0]*100.0)/100.0;

	//	V_err = ev[0];

	//	double da = 10.0;
	//	QRectF area = QRectF(QPointF(15.0-da, 15.0+da),QPointF(15.0+da,15.0-da));
	//	if(!area.contains(QPointF(this->x.x, this->x.y))){
	//		pg = QPointF(15.0, 15.0);
	//		pl = purePursuit->cnv2Local(x.x,x.y,x.theta,pg);
	//		return;
	//	}

	//Global reference
	//	pg = purePursuit->calc(X_t.x, X_t.y, X_t.theta, 1.0, 3.0, *gPath);
	//	pl = purePursuit->cnv2Local(X_t.x, X_t.y, X_t.theta, pg);
}

void Data::startTimeMeasurement()
{
	log_startTime = QDateTime::currentDateTime();
	log_currentTime = log_startTime;
}

void Data::updatePeriod()
{
	QDateTime previous = ct;
	ct = QDateTime::currentDateTime();

	T	= (double)st.msecsTo(ct)/1000.0;
	dt = (double)(previous.msecsTo(ct))/1000.0;
}

QString Data::getTimeStampStr()
{
	return ct.toString("yyyy_MM_dd hh_mm_ss_zzz");
}

void Data::SetupLogging(QString prefix, QString suffix)
{
	// make log file
	QString fname = QString("Logs/");
	fname += prefix;
	fname += QDateTime::currentDateTime().toString("_yyyy_MM_dd_hh_mm_ss_");
	fname += suffix;
	fname += QString(".csv");

	log->setFileName(fname);
	if(!log->open(QFile::Text | QFile::WriteOnly)){
		qCritical() << "error";
		exit(1);
	}
	out->setDevice(log);

	// write headers
	(*out) << "T[sec],dt[sec],"
				 << "State,Mode,"
				 << "x,y,theta,";
	(*out) << "xg,yg,"
				 << "(rot)v[m/s],"
				 << "(gps)v[m/s],"
				 << "lat,lon,"
				 << "Qual,"
				 << "\n";

	//	fname = QString("Logs/GPS/");
	//	fname += prefix;
	//	fname += QDateTime::currentDateTime().toString("_yyyy_MM_dd_hh_mm_ss_");
	//	fname += QString(".csv");

	//	logGPS->setFileName(fname);
	//	if(!logGPS->open(QFile::Text | QFile::WriteOnly)){
	//		qCritical() << "error";
	//		exit(1);
	//	}
	//	outGPS->setDevice(logGPS);

	//	//Headers
	//	(*outGPS) << "T[sec],"
	//						<< "dt[sec],"
	//						<< "Latitude,"
	//						<< "Longitude,"
	//						<< "Velocity[m/s],"
	//						<< "Quality,"
	//						<< "\n";

	startTimeMeasurement();
	isLog = true;
}

void Data::Logging()
{
	if(!isLog) return;
	if(!log->isOpen()) return;

	QDateTime prev = log_currentTime;
	log_currentTime = QDateTime::currentDateTime();

	/*
	(*out) << "T[sec],dt[sec]"
				 << "State,Mode,"
				 << "x,y,theta,";
	(*out) << "xg,yg,"
				 << "(rot)v[m/s],"
				 << "(gps)v[m/s],"
				 << "Qual,"
				 << "\n";
	*/

	//  (*out) << (double)log_startTime.msecsTo(log_currentTime)/1000.0 << ","
	//         << (double)(prev.msecsTo(log_currentTime))/1000.0 << ","
	//         << State::map2str[state] << ","
	//         << Mode::map2str[hardware.clutch.Out] << ","
	//         << X_t.x << "," << X_t.y << "," << X_t.theta << ",";

	//	(*out) << pg.x() << "," << pg.y() << ","
	//				 << hardware.rotary.v << ","
	//				 << gpsInfo.v << ","
	//						//				 << gpsInfo.raw_lat << "," << gpsInfo.raw_lon << ","
	//				 << gpsInfo.geoCoord.toString(QGeoCoordinate::CoordinateFormat::Degrees) << ","
	//				 << gpsInfo.geoCoord.toString(QGeoCoordinate::CoordinateFormat::DegreesMinutes) << ","
	//				 << gpsInfo.quality << ","
	//				 << imuInfo.yaw << ","
	//				 << imuInfo.pitch << ","
	//				 << imuInfo.roll << ","
	//				 << "\n";

	//	(*outGPS) << (double)log_startTime.msecsTo(log_currentTime)/1000.0 << ","
	//						<< (double)(prev.msecsTo(log_currentTime))/1000.0 << ","
	//						<< gpsInfo.geoCoord.latitude() << ","
	//						<< gpsInfo.geoCoord.longitude() << ","
	//						<< gpsInfo.v << ","
	//						<< gpsInfo.strQuality << ","
	//						<< "\n";

	return;
}

void Data::QuitLogging()
{
	isLog = false;
	// close log file
	if(log->isOpen()){
		log->close();
	}

	//	if(logGPS->isOpen()){
	//		logGPS->close();
	//	}
}

void Data::restoreConfig()
{
	//	cfg->setDouble("GPS", "OFFSET_X", gpsInfo.offset.x());
	//	cfg->setDouble("GPS", "OFFSET_Y", gpsInfo.offset.y());
}

//void Data::On_GPS_connected()
//{
//  //  purePursuit->reset();
//}

//void Data::On_CommServer_received(CommServerInfo::Recv_t d){ this->cmd = d; }
//void Data::On_GPS_data_updated(GPSInfo::Data_t d){this->gpsInfo = d;}
//void Data::On_IMU_data_updated(IMUInfo::Data_t d){this->imuInfo = d;}


