#include "mainwindow.h"
#include <QApplication>
#include "actuatorcontroller.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ActuatorController::initController(argc,argv);
    ActuatorController::getInstance()->autoRecoginze();
    MainWindow w;
    w.show();

    return a.exec();
}
