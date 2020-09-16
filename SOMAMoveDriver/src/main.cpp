#include <QApplication>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

		qSetMessagePattern("[%{function}](%{line}) %{message}");

    MainWindow w;
    w.show();

    return a.exec();
}
