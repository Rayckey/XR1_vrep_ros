#include "EthernetCommunication.h"
#include "EthernetCommunicateUnit.h"
#include <QThread>


EthernetCommunication::EthernetCommunication(QObject *parent):
    Communication(parent)
{

}

int EthernetCommunication::addCommunication(const QString &str, const quint32 num)
{

    //quint8 nPortNum = rx.cap(0).toUInt();
    CommunicateUnit * pUnit = nullptr;

    pUnit = new EthernetCommunicateUnit(num,str,num);//id与端口一致，现在端口都不一样

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

    return -1;
}

void EthernetCommunication::removeUnAvailablePorts()
{
    qDebug() << "Ethernet remove" << m_lUnits.size();
    if(m_lUnits.size() == 0)
        emit connectionError(0,Actuator::ERR_IP_ADDRESS_NOT_FOUND,"No available ip address!");
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
                QString str = tr("ip : %1 connection error, no CAN can be connected!").arg(m_lUnits[i]->getCommunicationUnitName());
                emit connectionError(m_lUnits[i]->getUnitId(),Actuator::ERR_CAN_DISCONNECTION,str);
//                m_lUnits.removeAt(i);
            }

        }
    }
}
