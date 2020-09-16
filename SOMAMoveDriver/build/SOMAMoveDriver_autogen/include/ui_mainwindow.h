/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QVBoxLayout *verticalLayout;
    QFrame *frame_2;
    QHBoxLayout *horizontalLayout_2;
    QLabel *lbldt;
    QLabel *lblT;
    QSpacerItem *horizontalSpacer;
    QLabel *lblState;
    QLabel *label;
    QLabel *lblMode;
    QSpacerItem *horizontalSpacer_2;
    QSpacerItem *verticalSpacer;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(404, 200);
        MainWindow->setMinimumSize(QSize(0, 200));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        verticalLayout = new QVBoxLayout(centralwidget);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        frame_2 = new QFrame(centralwidget);
        frame_2->setObjectName(QStringLiteral("frame_2"));
        frame_2->setFrameShape(QFrame::Panel);
        frame_2->setFrameShadow(QFrame::Raised);
        horizontalLayout_2 = new QHBoxLayout(frame_2);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(-1, 3, -1, 3);
        lbldt = new QLabel(frame_2);
        lbldt->setObjectName(QStringLiteral("lbldt"));
        lbldt->setMinimumSize(QSize(80, 0));
        lbldt->setFrameShape(QFrame::Panel);
        lbldt->setFrameShadow(QFrame::Raised);
        lbldt->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        horizontalLayout_2->addWidget(lbldt);

        lblT = new QLabel(frame_2);
        lblT->setObjectName(QStringLiteral("lblT"));
        lblT->setMinimumSize(QSize(80, 0));
        lblT->setFrameShape(QFrame::Panel);
        lblT->setFrameShadow(QFrame::Raised);
        lblT->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        horizontalLayout_2->addWidget(lblT);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer);

        lblState = new QLabel(frame_2);
        lblState->setObjectName(QStringLiteral("lblState"));
        lblState->setFrameShape(QFrame::NoFrame);
        lblState->setFrameShadow(QFrame::Raised);
        lblState->setAlignment(Qt::AlignCenter);

        horizontalLayout_2->addWidget(lblState);

        label = new QLabel(frame_2);
        label->setObjectName(QStringLiteral("label"));
        label->setFrameShape(QFrame::NoFrame);
        label->setFrameShadow(QFrame::Raised);

        horizontalLayout_2->addWidget(label);

        lblMode = new QLabel(frame_2);
        lblMode->setObjectName(QStringLiteral("lblMode"));
        lblMode->setFrameShape(QFrame::NoFrame);
        lblMode->setFrameShadow(QFrame::Raised);

        horizontalLayout_2->addWidget(lblMode);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_2);


        verticalLayout->addWidget(frame_2);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 404, 28));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "ATV Move Driver", Q_NULLPTR));
        lbldt->setText(QApplication::translate("MainWindow", "0.0", Q_NULLPTR));
        lblT->setText(QApplication::translate("MainWindow", "0", Q_NULLPTR));
        lblState->setText(QApplication::translate("MainWindow", "State", Q_NULLPTR));
        label->setText(QApplication::translate("MainWindow", " / ", Q_NULLPTR));
        lblMode->setText(QApplication::translate("MainWindow", "Mode", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
