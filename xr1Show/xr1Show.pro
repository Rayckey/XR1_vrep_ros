#-------------------------------------------------
#
# Project created by QtCreator 2018-04-28T11:36:20
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets


TARGET = xr1Show
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS \
            IMU_ENABLE

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h\
        xr1define.h

FORMS    += mainwindow.ui

#win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../Actuator/Ethernet2CAN_sdk_v2.0.0/sdk/lib/linux_x86_64/release/ -lactuatorController
#else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../Actuator/Ethernet2CAN_sdk_v2.0.0/sdk/lib/linux_x86_64/debug/ -lactuatorControllerd
#else:unix: LIBS += -L$$PWD/../Actuator/Ethernet2CAN_sdk_v2.0.0/sdk/lib/linux_x86_64/ -lactuatorControllerd

#INCLUDEPATH += $$PWD/../Actuator/Ethernet2CAN_sdk_v2.0.0/sdk/include
#DEPENDPATH += $$PWD/../Actuator/Ethernet2CAN_sdk_v2.0.0/sdk/include

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../Actuator/serialPort_Ethernet2CAN_sdk_v2.0.0/sdk/lib/linux_x86_64/release/ -lactuatorController
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../Actuator/serialPort_Ethernet2CAN_sdk_v2.0.0/sdk/lib/linux_x86_64/debug/ -lactuatorController
else:unix: LIBS += -L$$PWD/../Actuator/serialPort_Ethernet2CAN_sdk_v2.0.0/sdk/lib/linux_x86_64/ -lactuatorController

INCLUDEPATH += $$PWD/../Actuator/serialPort_Ethernet2CAN_sdk_v2.0.0/sdk/include
DEPENDPATH += $$PWD/../Actuator/serialPort_Ethernet2CAN_sdk_v2.0.0/sdk/include


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../build-QXboxController-Desktop_Qt_5_7_1_GCC_64bit-Debug/release/ -lQXboxController
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../build-QXboxController-Desktop_Qt_5_7_1_GCC_64bit-Debug/debug/ -lQXboxController
else:unix: LIBS += -L$$PWD/../build-QXboxController-Desktop_Qt_5_7_1_GCC_64bit-Debug/ -lQXboxController

INCLUDEPATH += $$PWD/../QXboxController
DEPENDPATH += $$PWD/../QXboxController


INCLUDEPATH += $$PWD/../Eigen3
DEPENDPATH += $$PWD/../Eigen3
