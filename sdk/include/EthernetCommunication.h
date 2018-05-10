#ifndef EHTERCOMMUNICATION_H
#define EHTERCOMMUNICATION_H
#include "communication.h"

class EthernetCommunication : public Communication
{
public:
    EthernetCommunication(QObject * parent = nullptr);
    virtual int addCommunication(const QString &str,const quint32 num);
    virtual void removeUnAvailablePorts();
};

#endif // EHTERCOMMUNICATION_H
