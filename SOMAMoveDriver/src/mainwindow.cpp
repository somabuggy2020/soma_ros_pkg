#include "mainwindow.h"
#include "ui_mainwindow.h"

#define IS_TRACE 0
#if IS_TRACE==1
#define TRACE(msg) qDebug()<<msg
#else
#define TRACE(msg)
#endif

/*!
 * \brief MainWindow::main
 */
void MainWindow::main()
{
	QMetaObject::invokeMethod(timer, "stop");

	emit updateLocalTime(QDateTime::currentDateTime());
	data->updatePeriod();
	emit updateTime(data->dt, data->T);

	//main process
	xbox->recv(data);
	hardware->recv(data);
	behavior->main(data);
	hardware->send(data);
	//end main process

	if(isFin){
		this->thread->exit(0);
		return;
	}


	//Emit signals
	emit updateState(data->state, data->mode);
	emit updatedHardwareData(data->hardware);

	QMetaObject::invokeMethod(timer, "start");
	return;
}

/*!
 * \brief MainWindow::MainWindow
 * \param parent
 *
 * constructor
 */
MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow)
{
	ui->setupUi(this);

	setup();

	data = new Data();
	hardware = new Hardware();
	behavior = new Behavior();
	xbox = new Xbox();

	if(hardware->initialize() == -1){}
	if(behavior->initialize() == -1){}

	qRegisterMetaType<StateVector>("StateVector");
	qRegisterMetaType<HardwareData>("HardwareData");

	start();
}

/*!
 * \brief MainWindow::~MainWindow
 *
 * destructore
 */
MainWindow::~MainWindow()
{
	delete ui;
}

void MainWindow::setup()
{
	HdDtVwr = new HardwareDataViewer(this);
	addDock(Qt::LeftDockWidgetArea, HdDtVwr);
	connect(this, SIGNAL(updatedHardwareData(HardwareData)),
					HdDtVwr, SLOT(setData(HardwareData)));

	connect(this, &MainWindow::updateTime,
					this, [=](double dt, double T){
		ui->lbldt->setText(QS_NUM(dt));
		ui->lblT->setText(QS_NUM2(T));
	});

	connect(this, &MainWindow::updateState,
					this, [=](int state, int mode){
		ui->lblState->setText(State::str[state]);
		ui->lblMode->setText(Mode::str[mode]);
	});

	//Show Local Time on status bar
	ui->statusbar->setFont(QFont("Consolas",12,QFont::Bold));
	connect(this, &MainWindow::updateLocalTime,
					this, [=](QDateTime stamp){
		ui->statusbar->showMessage(stamp.toString("hh:mm:ss:zzz"));
	}, Qt::QueuedConnection);
}

/*!
 * \brief MainWindow::start
 *
 * main process thread start function
 */
void MainWindow::start()
{
	thread = new QThread();
	timer = new QTimer();

	timer->setInterval(33);
	timer->moveToThread(thread);

	connect(timer, SIGNAL(timeout()), this,
					SLOT(main()), Qt::DirectConnection);

	hardware->setThread(thread);
	//	hardware->moveToThread(thread);

	thread->start();

	qInfo() << "Start Main Process";
	isFin = false;
	QMetaObject::invokeMethod(timer, "start");
}

/*!
 * \brief MainWindow::closeEvent
 * \param event
 *
 * close event
 * ex) Alt+F4, close buttom, ...
 */
void MainWindow::closeEvent(QCloseEvent *event)
{
	isFin = true;
	QThread::msleep(250);
	qInfo() << "Finish Main Process";
}



