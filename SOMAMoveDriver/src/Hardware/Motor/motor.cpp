#include "motor.h"

void* Motor::MainHandle = NULL;
void* Motor::SubHandle = NULL;

#define IS_TRACE 0
#if IS_TRACE == 1
#define TRACE(msg) qDebug()<<msg;
#else
#define TRACE(msg)
#endif


/*!
 * \brief Motor::Motor
 * \param parent
 */
Motor::Motor(QObject *parent) : QObject(parent)
{
	handle = NULL;
	dTrgPos = 0.0;
}

/*!
 * \brief Motor::Motor
 * \param name
 * \param role
 * \param parent
 */
Motor::Motor(QString name, QString role, QObject *parent)
	: QObject(parent)
{
	this->name = name;
	this->role = role;
}

/*!
 * \brief Motor::~Motor
 */
Motor::~Motor()
{
}

/*!
 * \brief Motor::initialize
 * \return
 */
int Motor::initialize()
{
	qInfo() << "Init." << name << "/" << role;

	QSettings *cfg = new QSettings(QString(CONFIG_FILE_PATH),
																 QSettings::IniFormat,
																 this);

	if(cfg->status() != QSettings::NoError){
		qCritical() << "Failure open config file";
		qCritical() << "PATH:" << cfg->fileName();
		return -1;
	}

	cfg->beginGroup("USE");
	isUse = cfg->value(role).toBool();
	cfg->endGroup();

	cfg->beginGroup(name);
	mcfg.DeviceName = cfg->value("DeviceName").toString();
	mcfg.ProtocolStackName = cfg->value("ProtocolStackName").toString();
	mcfg.InterfaceName = cfg->value("InterfaceName").toString();
	mcfg.PortName = cfg->value("PortName").toString();

	mcfg.Node		= cfg->value("NODE").toUInt();
	mcfg.MaxRPM	= cfg->value("MAXRPM").toUInt();
	mcfg.Accel	= cfg->value("ACCEL").toUInt();
	mcfg.Decel	= cfg->value("DECEL").toUInt();
	mcfg.EncReso		= cfg->value("ENCRESO").toInt();
	mcfg.GearRatio	= cfg->value("GEARRATIO").toDouble();
	mcfg.MaxPos = cfg->value("MAXPOS").toDouble();
	mcfg.MinPos = cfg->value("MINPOS").toDouble();
	cfg->endGroup();

	return 0;
}

/*!
 * \brief Motor::open
 * \return
 */
int Motor::open()
{
	if(!isUse) return 0;

	qInfo() << "Open" << name << "/" << role;

	if(MainHandle == NULL){
		if(FindMainHandle() == -1) return -1;
		else handle = MainHandle;
	}

	if(SubHandle == NULL){
		if(FindSubHandle() == -1) return -1;

	}
	else handle = SubHandle;

	unsigned int error = 0;
	int ret = true;

	//(1) Clear Faults
	ret = VCS_ClearFault(handle, (unsigned short)mcfg.Node, &error);
	if(!ret){
		qCritical() << name << ":" << strVCSError(error);
		return -1;
	}

	//(2) Set Enable
	ret = VCS_SetEnableState(handle, (unsigned short)mcfg.Node, &error);
	if(!ret){
		qCritical() << name << ":" << strVCSError(error);
		return -1;
	}

	//(3) Set Operation Mode (Profile Position Mode)
	ret = VCS_SetOperationMode(handle, (unsigned short)mcfg.Node, OMD_PROFILE_POSITION_MODE, &error);
	if(!ret){
		qCritical() << name << ":" << strVCSError(error);
		return -1;
	}

	//(4) Set Max Following Error
	ret = VCS_SetMaxFollowingError(handle, (unsigned short)mcfg.Node, 30000, &error);
	if(!ret){
		qCritical() << name << ":" << strVCSError(error);
		return -1;
	}

	//(5) Set Position Profile
	ret = VCS_SetPositionProfile(handle,
															 mcfg.Node,
															 mcfg.MaxRPM,
															 mcfg.Accel,
															 mcfg.Decel,
															 &error);
	if(!ret){
		qCritical() << name << ":" << strVCSError(error);
		return -1;
	}

	TRACE("Success motor open");
	return 0;
}

/*!
 * \brief Motor::close
 */
void Motor::close()
{
	if(!isUse) return;
	if(handle == NULL) return;

	unsigned int error = 0;

	if(MainHandle != NULL && SubHandle != NULL){
		VCS_CloseAllDevices(&error);
	}

	SubHandle = NULL;
	MainHandle = NULL;

	qInfo() << "Closed" << name << "/" << role;

	return;
}

/*!
 * \brief Motor::move
 * \param pos
 * \param minmax
 * \param immediatery
 * \return
 */
int Motor::moveto(double pos, bool minmax, bool immediatery)
{
	//limit
	TRACE(QString("(Move to %1)").arg(pos));
	if(minmax){
		pos = std::min<double>(pos, mcfg.MaxPos);
		pos = std::max<double>(pos, mcfg.MinPos);
	}
	TRACE(QString("Move to %1").arg(pos));

	if(!isUse){
		dTrgPos = pos;
		return 0;
	}

	//Calculation qc
	double deg = pos/360.0*mcfg.GearRatio;
	long qc = deg*(double)(mcfg.EncReso);

	unsigned int error = 0;
	bool ret = VCS_MoveToPosition(handle,
																mcfg.Node,
																qc,
																true,	//Absolute
																immediatery,
																&error);

	if(!ret){
		qWarning() << strVCSError(error);
		return -1;
	}

	TRACE("Success set target position");
	return 0;
}

int Motor::setMaxRPM(unsigned long MaxRPM)
{
	MaxRPM = std::min<unsigned long>(MaxRPM, mcfg.MaxRPM);

	if(!isUse){
		return 0;
	}

	TRACE(QString("Set to %1").arg(MaxRPM));

	unsigned int error = 0;
	bool ret = VCS_SetPositionProfile(handle,
																		mcfg.Node,
																		MaxRPM,
																		mcfg.Accel,
																		mcfg.Decel,
																		&error);
	if(!ret){
		qWarning() << strVCSError(error);
		return -1;
	}

	TRACE("Success set target RPM");
	return 0;
}

/*!
 * \brief Motor::recv
 * \param d
 * \return
 *
 * アクチュエータの出力データ(電流、速度、角度)を取得
 */
int Motor::recv(MotorInfo::Data_t &d)
{
	if(!isUse){
		d.Out.trgPos = dTrgPos;
		return 0;
	}

	int ret = 0;
	unsigned int error = 0;

	int qc = 0;
	ret = VCS_GetPositionIs(handle, mcfg.Node, &qc, &error);
	if(!ret){
		qWarning() << strVCSError(error);
		return -1;
	}
	d.Out.pos = (double)qc/mcfg.EncReso*360.0/mcfg.GearRatio;

	long trgQc = 0;
	ret = VCS_GetTargetPosition(handle, mcfg.Node, &trgQc, &error);
	if(!ret){
		qWarning() << strVCSError(error);
		return -1;
	}
	d.Out.trgPos = (double)trgQc/mcfg.EncReso*360.0/mcfg.GearRatio;

	ret = VCS_GetVelocityIs(handle, mcfg.Node, &d.Out.rpm, &error);
	if(!ret){
		qWarning() << strVCSError(error);
		return -1;
	}

	ret = VCS_GetCurrentIs(handle, mcfg.Node, &d.Out.cur, &error);
	if(!ret){
		qWarning() << strVCSError(error);
		return -1;
	}

	d.min_pos = mcfg.MinPos;
	d.max_pos = mcfg.MaxPos;

	return 0;
}

/*!
 * \brief Motor::FindMainHandle
 * \return
 */
int Motor::FindMainHandle()
{
	unsigned int error = 0;

	char *DeviceName = new char[mcfg.DeviceName.length()];
	char *ProtocolStackName = new char[mcfg.ProtocolStackName.length()];
	char *InterfaceName = new char[mcfg.InterfaceName.length()];
	char *PortName = new char[mcfg.PortName.length()];

	memcpy(DeviceName, mcfg.DeviceName.toStdString().c_str(), mcfg.DeviceName.length());
	memcpy(ProtocolStackName, mcfg.ProtocolStackName.toStdString().c_str(), mcfg.ProtocolStackName.length());
	memcpy(InterfaceName, mcfg.InterfaceName.toStdString().c_str(), mcfg.InterfaceName.length());
	memcpy(PortName, mcfg.PortName.toStdString().c_str(), mcfg.PortName.length());

	MainHandle = VCS_OpenDevice(DeviceName,
															ProtocolStackName,
															InterfaceName,
															PortName,
															&error);

	if(MainHandle == NULL){
		qCritical() << QString("%1 : Device Open Failed").arg(name);
		return -1;
	}

	return 0;
}

/*!
 * \brief Motor::FindSubHandle
 * \return
 */
int Motor::FindSubHandle()
{
	unsigned int error = 0;

	char DeviceName[mcfg.DeviceName.length()] = {'\0'};
	char ProtocolStackName[mcfg.ProtocolStackName.length()] = {'\0'};
	char InterfaceName[mcfg.InterfaceName.length()] = {'\0'};

	memcpy(DeviceName, mcfg.DeviceName.toStdString().c_str(), mcfg.DeviceName.length());
	memcpy(ProtocolStackName, mcfg.ProtocolStackName.toStdString().c_str(), mcfg.ProtocolStackName.length());
	memcpy(InterfaceName, mcfg.InterfaceName.toStdString().c_str(), mcfg.InterfaceName.length());

	SubHandle = VCS_OpenSubDevice(
				MainHandle,
				DeviceName,
				ProtocolStackName,
				&error);

	if(SubHandle == NULL){
		qCritical() << QString("%1 : Device Open Failed").arg(name);
		return -1;
	}

	return 0;
}

/*!
 * \brief Motor::strVCSError
 * \param error_code
 * \return
 *
 * EPOSのエラー詳細文字列を取得
 *
 */
QString Motor::strVCSError(unsigned int error_code)
{
	char str[256] = {'\0'};
	VCS_GetErrorInfo(error_code, str, 256);
	return QString(str);
}
