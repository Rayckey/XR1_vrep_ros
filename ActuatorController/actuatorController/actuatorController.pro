#-------------------------------------------------
#
# Project created by QtCreator 2018-01-04T18:34:48
#
#-------------------------------------------------

QT       -= gui
QT += network
QT += core
CONFIG += c++11
CONFIG(debug,debug|release){
TARGET = actuatorControllerd
}else{
TARGET = actuatorController
}

TEMPLATE = lib
#CONFIG -= app_bundle

DEFINES += ACTUATORCONTROLLER_LIBRARY
# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS
DEFINES += QT_NO_DEBUG_OUTPUT
#自定义
DEFINES += USING_VECTOR
DEFINES += IMU_ENABLE
# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    communication.cpp \
    dataUtil.cpp \
    idata.cpp \
    innfosproxy.cpp \
    mediator.cpp \
    proxyparser.cpp \
    communicateunit.cpp \
    SerialAutoRecognize.cpp \
    EthernetCommunicateUnit.cpp \
    EthernetAutoRecognize.cpp \
    SerialCommunicateUnit.cpp \
    actuatordata.cpp \
    actuatorcontroller.cpp \
    EthernetCommunication.cpp \
    SerialCommunication.cpp \
    AbstractAutoRecognize.cpp

HEADERS +=\
    communication.h \
    dataUtil.h \
    idata.h \
    innfosproxy.h \
    mediator.h \
    proxyparser.h \
    communicateunit.h \
    CSignal.hpp \
    SerialAutoRecognize.h \
    EthernetCommunicateUnit.h \
    EthernetAutoRecognize.h \
    SerialCommunicateUnit.h \
    actuatordata.h \
    actuatorcontroller_global.h \
    actuatorcontroller.h \
    actuatordefine.h \
    EthernetCommunication.h \
    SerialCommunication.h \
    AbstractAutoRecognize.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}

win32:CONFIG(debug,debug|release):DESTDIR = ../../serialPort_Ethernet2CAN_sdk_v2.0.0/sdk/lib/windows_x64/debug
else:win32:CONFIG(release,debug|release):DESTDIR = ../../serialPort_Ethernet2CAN_sdk_v2.0.0/sdk/lib/windows_x64/release
else:unix:DESTDIR = ../../serialPort_Ethernet2CAN_sdk_v2.0.0/sdk/lib/linux_x86_64
