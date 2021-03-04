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
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QLabel *CameraUp_label;
    QLabel *CameraDown_label;
    QPushButton *Change_pushButton;
    QLabel *CameraMid_label;
    QLineEdit *lineEdit_SurfNum_1;
    QLabel *CameraSplice_label;
    QLabel *label;
    QLineEdit *lineEdit_SurfNum_2;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *label_5;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1920, 1080);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        CameraUp_label = new QLabel(centralWidget);
        CameraUp_label->setObjectName(QStringLiteral("CameraUp_label"));
        CameraUp_label->setGeometry(QRect(10, 20, 640, 480));
        CameraDown_label = new QLabel(centralWidget);
        CameraDown_label->setObjectName(QStringLiteral("CameraDown_label"));
        CameraDown_label->setGeometry(QRect(30, 520, 640, 480));
        Change_pushButton = new QPushButton(centralWidget);
        Change_pushButton->setObjectName(QStringLiteral("Change_pushButton"));
        Change_pushButton->setGeometry(QRect(670, 50, 121, 51));
        Change_pushButton->setFocusPolicy(Qt::WheelFocus);
        CameraMid_label = new QLabel(centralWidget);
        CameraMid_label->setObjectName(QStringLiteral("CameraMid_label"));
        CameraMid_label->setGeometry(QRect(1290, -10, 640, 480));
        lineEdit_SurfNum_1 = new QLineEdit(centralWidget);
        lineEdit_SurfNum_1->setObjectName(QStringLiteral("lineEdit_SurfNum_1"));
        lineEdit_SurfNum_1->setGeometry(QRect(930, 50, 113, 25));
        CameraSplice_label = new QLabel(centralWidget);
        CameraSplice_label->setObjectName(QStringLiteral("CameraSplice_label"));
        CameraSplice_label->setGeometry(QRect(10, 270, 640, 480));
        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(860, 50, 64, 17));
        lineEdit_SurfNum_2 = new QLineEdit(centralWidget);
        lineEdit_SurfNum_2->setObjectName(QStringLiteral("lineEdit_SurfNum_2"));
        lineEdit_SurfNum_2->setGeometry(QRect(1150, 50, 113, 25));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(1070, 50, 64, 17));
        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(20, 0, 71, 17));
        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(720, 180, 81, 17));
        label_5 = new QLabel(centralWidget);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(10, 510, 91, 20));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1920, 22));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", Q_NULLPTR));
        CameraUp_label->setText(QApplication::translate("MainWindow", "TextLabel", Q_NULLPTR));
        CameraDown_label->setText(QApplication::translate("MainWindow", "TextLabel", Q_NULLPTR));
        Change_pushButton->setText(QApplication::translate("MainWindow", "SplicePlay", Q_NULLPTR));
        CameraMid_label->setText(QApplication::translate("MainWindow", "TextLabel", Q_NULLPTR));
        lineEdit_SurfNum_1->setText(QApplication::translate("MainWindow", "100", Q_NULLPTR));
        CameraSplice_label->setText(QApplication::translate("MainWindow", "TextLabel", Q_NULLPTR));
        label->setText(QApplication::translate("MainWindow", "matches1", Q_NULLPTR));
        lineEdit_SurfNum_2->setText(QApplication::translate("MainWindow", "100", Q_NULLPTR));
        label_2->setText(QApplication::translate("MainWindow", "matches2", Q_NULLPTR));
        label_3->setText(QApplication::translate("MainWindow", "CameraUp", Q_NULLPTR));
        label_4->setText(QApplication::translate("MainWindow", "CameraMid", Q_NULLPTR));
        label_5->setText(QApplication::translate("MainWindow", "CameraDown", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
