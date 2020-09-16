#include "hardwaredataviewer.h"
#include "ui_hardwaredataviewer.h"

HardwareDataViewer::HardwareDataViewer(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::HardwareDataViewer)
{
	ui->setupUi(this);

	ui->tw->setColumnCount(2);
	QStringList headers;
	headers << "key" << "value";
	ui->tw->setHeaderLabels(headers);

	twiSteering = new TWI_Actuator(ui->tw, "Steering");
	twiRearBrake = new TWI_Actuator(ui->tw, "Rear");
	twiFrontBrake = new TWI_Actuator(ui->tw, "Front");
	twiAccel = new TWI_Actuator(ui->tw, "Accel");

	twiClutch = new QTreeWidgetItem(ui->tw);
	twiClutch->setText(0, "Clutch");
	twiClutch->setText(1, "-");

	twiRotary = new TWI_Rotary(ui->tw);

	//	mapStrClutchState[ClutchInfo::Forward] = ClutchInfo::Str::Forward;
	//	mapStrClutchState[ClutchInfo::Backward] = ClutchInfo::Str::Backward;
	//	mapStrClutchState[ClutchInfo::Free] = ClutchInfo::Str::Free;
}

HardwareDataViewer::~HardwareDataViewer()
{
	delete ui;
}

void HardwareDataViewer::setData(HardwareData hd)
{
	QString str;
	str += QString("%1").arg(ClutchInfo::str[hd.clutch.Out]);
	str += QString("/");
	str += QString("%2").arg(ClutchInfo::str[hd.clutch.In]);
	twiClutch->setText(1, str);

	twiRotary->setData(hd.rotary);

	twiSteering->setData(hd.steering);
	twiRearBrake->setData(hd.rearBrake);
	twiFrontBrake->setData(hd.frontBrake);
	twiAccel->setData(hd.accel);
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

	max_rmp = new QTreeWidgetItem(label);
	max_rmp->setText(0, "MaxRPM");
	max_rmp->setText(1, "0.0");
}

void TWI_Actuator::setData(MotorInfo::Data_t data)
{
	QString str;
	str += QString("%1").arg(QS_NUM2(data.Out.pos));
	str += QString("/");
	str += QString("%2").arg(QS_NUM2(data.Out.trgPos));
	label->setText(1, str);

	pos->setText(1, QS_NUM2(data.Out.pos));
	trgt_pos->setText(1, QS_NUM2(data.Out.trgPos));
	max_rmp->setText(1, QS_NUM(data.Out.rpm));
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

void TWI_Rotary::setData(RotaryInfo::Data_t data)
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
