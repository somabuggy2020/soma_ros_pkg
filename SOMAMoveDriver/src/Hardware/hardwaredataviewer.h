#ifndef HARDWAREDATAVIEWER_H
#define HARDWAREDATAVIEWER_H

#include <QWidget>
#include <QDebug>
#include <QString>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QMap>

#include "../Common/definitions.h"
#include "../Data/data.h"
#include "Motor/motorinfo.h"
#include "Clutch/clutchinfo.h"
#include "Rotary/rotaryinfo.h"

class TWI_Actuator;
class TWI_Rotary;

namespace Ui {
	class HardwareDataViewer;
}

class HardwareDataViewer : public QWidget
{
	Q_OBJECT

public:
	explicit HardwareDataViewer(QWidget *parent = 0);
	~HardwareDataViewer();

private slots:
	void setData(HardwareData hdData);

private:
	Ui::HardwareDataViewer *ui;

	QTreeWidgetItem *twiClutch;
	TWI_Actuator *twiSteering;
	TWI_Actuator *twiRearBrake;
	TWI_Actuator *twiFrontBrake;
	TWI_Actuator *twiAccel;
	TWI_Rotary *twiRotary;

	QMap<int, QString> mapStrClutchState;
};

/*!
 * \brief The TWI_Actuator class
 */
class TWI_Actuator
{
public:
	TWI_Actuator(QTreeWidget *parent, QString name);
	void setData(MotorInfo::Data_t data);

private:
	QTreeWidget *parent;
	QTreeWidgetItem *label;
	QTreeWidgetItem *pos, *trgt_pos;
	QTreeWidgetItem *max_rmp;
};

/*!
 * \brief The TWI_Rotary class
 */
class TWI_Rotary
{
public:
	TWI_Rotary(QTreeWidget *parent);
	void setData(RotaryInfo::Data_t data);
private:
	QTreeWidget *parent;
	QTreeWidgetItem *label;
	QTreeWidgetItem *pulse;
	QTreeWidgetItem *distance;
	QTreeWidgetItem *velocity;
};

#endif // HARDWAREDATAVIEWER_H
