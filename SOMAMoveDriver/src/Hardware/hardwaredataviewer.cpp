#include "hardwaredataviewer.h"
#include "ui_hardwaredataviewer.h"

/*!
 * \brief HardwareDataViewer::HardwareDataViewer
 * \param parent
 */
HardwareDataViewer::HardwareDataViewer(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::HardwareDataViewer)
{
	ui->setupUi(this);

	ui->tw->setColumnCount(2);
	QStringList headers;
	headers << "key" << "value";
	ui->tw->setHeaderLabels(headers);

	//Tree view class for actuator data
	twiSteering = new TWI_Actuator(ui->tw, "Steering");
	twiRearBrake = new TWI_Actuator(ui->tw, "Rear");
	twiFrontBrake = new TWI_Actuator(ui->tw, "Front");
	twiAccel = new TWI_Actuator(ui->tw, "Accel");

	twiClutch = new QTreeWidgetItem(ui->tw);
	twiClutch->setText(0, "Clutch");
	twiClutch->setText(1, "-");

	twiRotary = new TWI_Rotary(ui->tw);
}

/*!
 * \brief HardwareDataViewer::~HardwareDataViewer
 */
HardwareDataViewer::~HardwareDataViewer()
{
	delete ui;
}

/*!
 * \brief HardwareDataViewer::setData
 * \param hd
 *
 * slot function
 */
void HardwareDataViewer::set(HardwareData data)
{

	QString str;
	str += QString("%1").arg(ClutchInfo::str[data.clutch.Out]);
	str += QString("/");
	str += QString("%2").arg(ClutchInfo::str[data.clutch.In]);

	twiClutch->setText(1, str);

	twiRotary->set(data.rotary);

	twiSteering->set(data.steering);
	twiRearBrake->set(data.rearBrake);
	twiFrontBrake->set(data.frontBrake);
	twiAccel->set(data.accel);
}

//--------------------------------------------------
//
//--------------------------------------------------
TWI_Actuator::TWI_Actuator(QTreeWidget *parent,
													 QString name)
{
	this->parent = parent;

	label = new QTreeWidgetItem(parent);
	label->setText(0, name);
	label->setText(1, "0.0/0.0");

	pos = new QTreeWidgetItem(label);
	pos->setText(0, "Pos[deg]");
	pos->setText(1, "0.0");

	trgt_pos = new QTreeWidgetItem(label);
	trgt_pos->setText(0, "Target[deg]");
	trgt_pos->setText(1, "0.0");

	rpm = new QTreeWidgetItem(label);
	rpm->setText(0, "MaxRPM");
	rpm->setText(1, "0.0");
}

/*!
 * \brief TWI_Actuator::set
 * \param data
 */
void TWI_Actuator::set(MotorInfo::Data_t data)
{
	QString str;
	str += QString("%1").arg(QS_NUM2(data.Out.pos));
	str += QString("/");
	str += QString("%2").arg(QS_NUM2(data.Out.trgPos));
	label->setText(1, str);

	pos->setText(1, QS_NUM2(data.Out.pos));
	trgt_pos->setText(1, QS_NUM2(data.Out.trgPos));
	rpm->setText(1, QS_NUM(data.Out.rpm));
}

//--------------------------------------------------
//
//--------------------------------------------------
TWI_Rotary::TWI_Rotary(QTreeWidget *parent)
{
	this->parent = parent;

	label = new QTreeWidgetItem(parent);
	label->setText(0, "Rotary");

	pulse = new QTreeWidgetItem(label);
	pulse->setText(0, "Pulse");
	pulse->setText(1, "0");

	distance = new QTreeWidgetItem(label);
	distance->setText(0, "Distance[m]");
	distance->setText(1, "0.00");

	velocity = new QTreeWidgetItem(label);
	velocity->setText(0, "Velocity[m/s]");
	velocity->setText(1, "0.00");
}

void TWI_Rotary::set(RotaryInfo::Data_t data)
{
	QString str = QString("%1,%2,%3").arg(data.pulse[0]).arg(data.d[0]).arg(data.v);
	label->setText(1, str);
	pulse->setText(1, QString::number(data.pulse[0]));
	distance->setText(1, QString::number(data.d[0]));
	velocity->setText(1, QString::number(data.v));
}

//--------------------------------------------------
//
//--------------------------------------------------
