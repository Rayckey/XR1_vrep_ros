#ifndef COMMUNICATION_H
#define COMMUNICATION_H
#include <QList>
#include <QMap>
#include <QObject>
#include "actuatordefine.h"

class CommunicateUnit;

class Communication : public QObject
{
    Q_OBJECT
public:
    static Communication *getInstance(int nType);
    explicit Communication(QObject * parent = nullptr);
    //void sendDataBeforeDestroyed();
    ~ Communication();
    virtual int addCommunication(const QString &str,const quint32 num)=0;
    void sendData(quint8 nDeviceId,const QByteArray & data);
    bool hasDataWaitToSend();
    void stop();
    void recordRemainCmds();
    void addRelateIdToUnit(quint32 nUnitId,quint8 nRelateId);
    virtual void removeUnAvailablePorts()=0;
    void changeUnitRelateId(quint8 nOldId,quint8 nNewId);
    void setUnitConnectionStatus(quint32 nUnitId,quint8 nStatus);
public slots:
    void unitFinished(quint8 unitId);
protected:

signals:
    void response(quint32 unitId,const QByteArray response);
    void request(const QByteArray &request);
    void connectionError(quint8 unitId,quint16 errorId,QString errorStr);
private:
protected:
    QList<CommunicateUnit *> m_lUnits;
public:
};

#endif // COMMUNICATION_H
