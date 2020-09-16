#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QString>
#include <QDebug>
#include <QDateTime>
#include <QThread>
#include <QTimer>
#include <QDockWidget>

#include "Data/data.h"
#include "Hardware/hardware.h"
#include "Behavior/behavior.h"
#include "Xbox/xbox.h"

#include "Hardware/hardwaredataviewer.h"

namespace Ui {
  class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

private:
  void setup();
  void start();


  QDockWidget *addDock(Qt::DockWidgetArea area, QWidget *widget)
  {
    QDockWidget *dw = new QDockWidget(this);
    dw->setWidget(widget);
    addDockWidget(area, dw);
    return dw;
  }

private slots:
  void main();

signals:
  void updateLocalTime(QDateTime stamp);
  void updateTime(double dt, double T);
  void updateState(int state, int mode);
  void updatedHardwareData(HardwareData);

private:
  Ui::MainWindow *ui;

  //thread instances
  QThread *thread;
  QTimer *timer;
  bool isFin;

  //main instances
  Data *data;
  Hardware *hardware;
  Behavior *behavior;
  Xbox *xbox;

  HardwareDataViewer *HdDtVwr;

protected:
  void closeEvent(QCloseEvent *event);
};

#endif // MAINWINDOW_H
