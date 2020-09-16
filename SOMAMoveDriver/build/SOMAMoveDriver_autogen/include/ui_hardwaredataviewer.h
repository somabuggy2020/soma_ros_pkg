/********************************************************************************
** Form generated from reading UI file 'hardwaredataviewer.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_HARDWAREDATAVIEWER_H
#define UI_HARDWAREDATAVIEWER_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_HardwareDataViewer
{
public:
    QHBoxLayout *horizontalLayout;
    QTreeWidget *tw;

    void setupUi(QWidget *HardwareDataViewer)
    {
        if (HardwareDataViewer->objectName().isEmpty())
            HardwareDataViewer->setObjectName(QStringLiteral("HardwareDataViewer"));
        HardwareDataViewer->resize(262, 198);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(HardwareDataViewer->sizePolicy().hasHeightForWidth());
        HardwareDataViewer->setSizePolicy(sizePolicy);
        HardwareDataViewer->setMinimumSize(QSize(250, 0));
        horizontalLayout = new QHBoxLayout(HardwareDataViewer);
        horizontalLayout->setSpacing(3);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(3, 3, 3, 3);
        tw = new QTreeWidget(HardwareDataViewer);
        QTreeWidgetItem *__qtreewidgetitem = new QTreeWidgetItem();
        __qtreewidgetitem->setText(0, QStringLiteral("1"));
        tw->setHeaderItem(__qtreewidgetitem);
        tw->setObjectName(QStringLiteral("tw"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(tw->sizePolicy().hasHeightForWidth());
        tw->setSizePolicy(sizePolicy1);
        tw->setMinimumSize(QSize(0, 0));
        QFont font;
        font.setFamily(QStringLiteral("Consolas"));
        font.setPointSize(9);
        font.setBold(true);
        font.setWeight(75);
        tw->setFont(font);

        horizontalLayout->addWidget(tw);


        retranslateUi(HardwareDataViewer);

        QMetaObject::connectSlotsByName(HardwareDataViewer);
    } // setupUi

    void retranslateUi(QWidget *HardwareDataViewer)
    {
        HardwareDataViewer->setWindowTitle(QApplication::translate("HardwareDataViewer", "Form", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class HardwareDataViewer: public Ui_HardwareDataViewer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_HARDWAREDATAVIEWER_H
