#ifndef INNFOSPROXY_H
#define INNFOSPROXY_H


#include <QObject>
#include <QVector>
#include "idata.h"
#include <QTime>
#include <QByteArray>
#include <QMutex>
#include "actuatordefine.h"

class QFile;
class QTextStream;


//串口协议(有数据有校验)
class InnfosProxy : public QObject
{
    Q_OBJECT
public:
    IData * m_pData;
public slots:
    void Send();
public:
    InnfosProxy(const int nId,const quint8 nDeviceId);
    virtual ~InnfosProxy();
    static void decode(quint32 communicateUnitId,QByteArray &buf);
    const int GetIdx()const { return m_nId; }
    static void SendProxy(const quint8 nDeviceId,const int nIdx);
    static void SendProxy(const quint8 nDeviceId,const quint32 nDeviceMac,const int nIdx,const quint8 newDeviceId);
    template <class T>
    static void SendProxy(const quint8 nDeviceId,const int nIdx,T data){
        InnfosProxy proxy(nIdx,nDeviceId);
        proxy.m_pData->WriteShort(sizeof(T));
        proxy.m_pData->WriteData(data);
        proxy.m_pData->AddCRCCode(sizeof(T));
        proxy.m_pData->AddProxyEnd();
        proxy.Send();
    }
    static void SendProxyScale(const quint8 nDeviceId,const int nIdx,qreal data,int nScale=(1<<24)){//actural vaue will be scale 1<<24
        int nScaleValue = data*nScale;
        SendProxy(nDeviceId,nIdx,nScaleValue);
    }
    static void SendQrealProxy(const quint8 nDeviceId,const int nIdx,qreal data);//data is qreal,will be scale
    static QByteArray getProxyContent(const quint8 nDeviceId,const int nProxyIdx);
protected:
private:
    enum TMP_DIRECTIVES{
        T_D_READ_QUATERNION=0x01,//读取四元数
        T_D_READ_ALL_QUATERNION=0x02,
    };

    static void decodeTmpCmd(quint32 communicateUnitId,QByteArray &buf);//解析与中间板的协议指令，指令内容里还有指令符
private:
    int m_nId;//协议id
protected:
};

//无数据协议，主要是为了获取一些数据
class NoDataProxy : public InnfosProxy
{
    Q_OBJECT
public slots:
public:
    NoDataProxy(const int nId, const quint8 nDeviceId);
    static void decode(quint32 communicateUnitId,QByteArray &buf);
protected:
private:
};

//有数据无校验
class NoCrcProxy : public InnfosProxy
{
    Q_OBJECT
public slots:
public:
    NoCrcProxy(const int nId, const quint8 nDeviceId);
    static void decode(quint32 communicateUnitId,QByteArray &buf);
protected:
private:
};

//proxy watcher,use to log those proxys without response
class ProxyWatcher : public QObject
{
    Q_OBJECT
public:
    static ProxyWatcher * getInstance();
    static void autoDestroy();
    void addSendItem(const QByteArray & buf);
    void reciveItem(const QByteArray & buf);
    virtual ~ProxyWatcher();
protected:
    void checkTimeout();
    ProxyWatcher(QObject *parent=Q_NULLPTR);
    struct logStruct{
        QTime sendTime;
        QByteArray sendContent;
    };
private:
    static ProxyWatcher * m_pWatcher;
    QVector<logStruct *> m_sendContents;
    QFile * m_pLogFile;
    QTextStream * m_pLogStream;
    QMutex m_qmMutex;
};
#endif // INNFOSPROXY_H
