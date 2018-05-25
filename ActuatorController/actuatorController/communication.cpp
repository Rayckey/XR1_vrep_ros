#include "communication.h"
#include <QIODevice>
#include <QDebug>
#include <QTime>
#include <QElapsedTimer>
#include <QFile>
#include <QTextStream>
#include <chrono>
#include <thread>
#include "innfosproxy.h"
#include <QThread>
#include <QRegExp>
#include "communicateunit.h"
#include "EthernetCommunication.h"
#include "SerialCommunication.h"


Communication *Communication::getInstance(int nType)
{
    Communication * pInstance = nullptr;
    switch (nType) {
    case Actuator::Via_Ethernet:
        pInstance = new EthernetCommunication();
        break;
    case Actuator::Via_Serialport:
        pInstance = new SerialCommunication();
    default:
        break;
    }
    return pInstance;
}

Communication::Communication(QObject *parent):
    QObject(parent)
{

}

//void Communication::sendDataBeforeDestroyed()
//{
//    if(m_pCommucation)
//    {
//#ifdef RELEASE_LOG
//    m_pCommucation->recordRemainCmds();
//#endif
//        while (true)
//        {
//            if(!m_pCommucation->hasDataWaitToSend())
//            {
//                //delete m_pCommucation;
//                break;
//            }
//            QThread::msleep(1);
//        }

//    }
//    //_pCommucation = nullptr;
//}

Communication::~Communication()
{
    stop();
}


void Communication::sendData(quint8 nDeviceId, const QByteArray &data)
{
    if(nDeviceId == 0)//send to all
    {
        for(int i=0;i<m_lUnits.size();++i)
        {
            m_lUnits[i]->sendData(data);
        }
    }
    else
    {
        for(int i=0;i<m_lUnits.size();++i)
        {
            if(m_lUnits[i]->isContainsRelateId(nDeviceId))
            {
                m_lUnits[i]->sendData(data);
                break;
            }
        }
    }
}

bool Communication::hasDataWaitToSend()
{
    for(int i=0;i<m_lUnits.size();++i)
    {
        if(m_lUnits[i]->hasDataWaiting())
        {
            return true;
        }
    }
    return false;
}

void Communication::stop()
{
    //todo
    for(int i=0;i<m_lUnits.size();++i)
    {
        m_lUnits[i]->stopCommunication();
    }
    m_lUnits.clear();
}

void Communication::recordRemainCmds()
{
//    if(m_waitSendDatas.size() > 0)
//    {
//        QFile * pFile = new QFile("waitSend.txt",this);
//        if(pFile->open(QFile::WriteOnly | QFile::Text | QFile::Truncate))
//        {
//            QTextStream stream2(pFile);
//            stream2 << m_waitSendDatas.size();
//            endl(stream2);
//            for(int i=0;i<m_waitSendDatas.size(); ++i)
//            {
//                QTextStream stream(pFile);
//                stream << m_waitSendDatas.at(i).toHex();
//                endl(stream);
//            }
//        }
//        pFile->close();
    //    }
}

void Communication::addRelateIdToUnit(quint32 nUnitId, quint8 nRelateId)
{
    for(int i=0;i<m_lUnits.size();++i)
    {
        CommunicateUnit * pUnit = m_lUnits[i];
        if(pUnit->getUnitId() == nUnitId)
        {
            pUnit->addRelateId(nRelateId);
        }
    }
}

void Communication::changeUnitRelateId(quint8 nOldId, quint8 nNewId)
{
    for(int i=0;i<m_lUnits.size();++i)
    {
        CommunicateUnit * pUnit = m_lUnits[i];
        if(pUnit->isContainsRelateId(nOldId))
        {
            pUnit->switchOldRelateIdToNewRelateId(nOldId,nNewId);
            qDebug() << "Unit";
        }
    }
}

void Communication::setUnitConnectionStatus(quint32 nUnitId, quint8 nStatus)
{
    for(int i=0;i<m_lUnits.size();++i)
    {
        CommunicateUnit * pUnit = m_lUnits[i];
        if(pUnit->getUnitId() == nUnitId)
        {
            pUnit->setConnectionStatus(nStatus);
            break;
        }
    }
}

void Communication::unitFinished(quint8 unitId)
{
    for(int i=0;i<m_lUnits.size();++i)
    {
        CommunicateUnit * pUnit = m_lUnits[i];
        if(pUnit->getUnitId() == unitId)
        {
            m_lUnits.removeAt(i);
            pUnit->deleteLater();
            break;
        }
    }
}
