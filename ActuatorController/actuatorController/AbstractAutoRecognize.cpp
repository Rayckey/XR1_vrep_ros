#include "AbstractAutoRecognize.h"
#include "mediator.h"
#include "EthernetAutoRecognize.h"
#include "SerialAutoRecognize.h"

AbstractAutoRecognize *AbstractAutoRecognize::getInstance(int nType)
{
    AbstractAutoRecognize * pInstance = nullptr;
    if(nType == Actuator::Via_Ethernet)
    {
        pInstance = new EthernetAutoRecognize();
    }
    else
    {
        pInstance = new SerialAutoRecognize();
    }
    return pInstance;
}

AbstractAutoRecognize::AbstractAutoRecognize(QObject *parent) : QObject(parent)
{

}

void AbstractAutoRecognize::addMototInfo(quint8 nDeviceId, quint32 nDeviceMac)
{
    m_motorsInfo.insert(nDeviceId,nDeviceMac);
}
