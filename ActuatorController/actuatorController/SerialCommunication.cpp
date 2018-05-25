#include "SerialCommunication.h"
#include "SerialCommunicateUnit.h"
#include <QThread>
#include <QDebug>

SerialCommunication::SerialCommunication(QObject *parent):
    Communication(parent)
{

}

int SerialCommunication::addCommunication(const QString &str, const quint32 num)
{

    QRegExp rx("\\d+");
    int pos = rx.indexIn(str);
    if(pos > -1)
    {
        quint8 nPortNum = rx.cap(0).toUInt();
        CommunicateUnit * pUnit = nullptr;

        pUnit = new SerialCommunicateUnit(nPortNum,str,num);

        if(pUnit != nullptr)
        {
            QThread * pThread = new QThread();
            pUnit->moveToThread(pThread);
            m_lUnits.append(pUnit);
            connect(pUnit,&CommunicateUnit::finished,pUnit,&CommunicateUnit::deleteLater);
            connect(pUnit,&CommunicateUnit::finished,pThread,&QThread::quit);
            connect(pThread,&QThread::finished,pThread,&QThread::deleteLater);

            connect(pUnit,&CommunicateUnit::response,this,&Communication::response);
            connect(pThread,&QThread::started,pUnit,&CommunicateUnit::progress);

            pThread->start();
            return pUnit->getUnitId();
        }
    }

    return -1;
}

void SerialCommunication::removeUnAvailablePorts()
{
    qDebug() << "Serial remove" << m_lUnits.size();

    for(int i=m_lUnits.size();--i>=0;)
    {
        if(!m_lUnits[i]->isAvailable())
        {
            if(m_lUnits[i]->getConnectionStatus()&Actuator::CAN_CONNECTED)
            {
                QString str = tr("ip : %1 connection error, no servo can be connected!").arg(m_lUnits[i]->getCommunicationUnitName());
                emit connectionError(m_lUnits[i]->getUnitId(),Actuator::ERR_ACTUATOR_DISCONNECTION,str);
            }
            else
            {
                m_lUnits.removeAt(i);
            }

        }
    }
    if(m_lUnits.size() == 0)
        emit connectionError(0,Actuator::ERR_CAN_DISCONNECTION,"no CAN can be connected!");
}
