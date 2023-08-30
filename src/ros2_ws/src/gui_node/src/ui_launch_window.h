/********************************************************************************
** Form generated from reading UI file 'launch_window.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LAUNCH_WINDOW_H
#define UI_LAUNCH_WINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_LaunchWindow
{
public:
    QGroupBox *groupBox;

    void setupUi(QWidget *LaunchWindow)
    {
        if (LaunchWindow->objectName().isEmpty())
            LaunchWindow->setObjectName(QString::fromUtf8("LaunchWindow"));
        LaunchWindow->resize(873, 581);
        groupBox = new QGroupBox(LaunchWindow);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(410, 60, 291, 391));

        retranslateUi(LaunchWindow);

        QMetaObject::connectSlotsByName(LaunchWindow);
    } // setupUi

    void retranslateUi(QWidget *LaunchWindow)
    {
        LaunchWindow->setWindowTitle(QApplication::translate("LaunchWindow", "Form", nullptr));
        groupBox->setTitle(QApplication::translate("LaunchWindow", "Launch nodes", nullptr));
    } // retranslateUi

};

namespace Ui {
    class LaunchWindow: public Ui_LaunchWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LAUNCH_WINDOW_H
