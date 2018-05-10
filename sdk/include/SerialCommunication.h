#ifndef SERIALCOMMUNICATION_H
#define SERIALCOMMUNICATION_H
#include "communication.h"

class SerialCommunication : public Communication
{
public:
    SerialCommunication(QObject * parent = nullptr);
    virtual int addCommunication(const QString &str,const quint32 num);
    virtual void removeUnAvailablePorts();
};


#endif // SERIALCOMMUNICATION_H
