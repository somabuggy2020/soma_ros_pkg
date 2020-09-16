#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <QString>
#include <QDebug>
#include <QMap>

//#include "../Config/config.h"
#include "../Data/data.h"

#include "statebase.h"
#include "States/stop.h"
#include "States/starting.h"
#include "States/traveling.h"
#include "States/stopping.h"
#include "States/stacking.h"
#include "States/stacking.h"

class Behavior
{
public:
  Behavior();

  int initialize();
  int setThread(QThread *th);
  void main(Data *data);

private:
  bool MotorCalibrationState(Data *data);

private:
  Stop *stop;
  Starting *starting;
  Traveling *traveling;
  Stopping *stopping;
  //	Stacking *stacking;

private:
  QMap<int, StateBase*> state;
};

#endif // BEHAVIOR_H
