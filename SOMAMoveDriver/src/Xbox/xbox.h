#pragma once

#include <QObject>
#include <QString>
#include <QDebug>

#include "Gamepad/gamepad.h"
#include "../Data/data.h"

class Xbox : public QObject
{
  Q_OBJECT
public:
  explicit Xbox(QObject *parent = nullptr);

public:
  void recv(Data *data);

private:
  bool _isRemote; //local flag
};
