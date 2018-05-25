﻿#include "proxyparser.h"
#include "innfosproxy.h"
#include "mediator.h"
#include <QDebug>

ProxyParser * ProxyParser::m_pParser = nullptr;

ProxyParser *ProxyParser::getInstance()
{
    if(!m_pParser)
    {
        m_pParser = new ProxyParser();
    }
    return m_pParser;
}


void ProxyParser::parse(quint32 communicateUnitId, const QByteArray &buf)
{
    m_remainData.append(buf);
    qDebug() << "parse" << buf.toHex();
    while (m_remainData.size() > 5) {
        if(!headCheck(m_remainData))
            handleError();
        if(m_remainData.size() > 5)
        {
            QByteArray byteData = m_remainData.left(5);
            m_remainData = m_remainData.right(m_remainData.size()-5);
            IData tmp(byteData);
            tmp.Skip(3);
            quint16 nLen = tmp.ReadShort();
            int nRemainCnt = 0;// data bytes remain to read
            if (nLen == 0)
            {
                nRemainCnt = 1;
            }
            else
            {
                if (nLen == 1 && byteData.at(2) != '\2' && byteData.at(2) != Actuator::D_CAN_CONNECT)//暂时处理
                {
                    nRemainCnt = 2;//
                }
                else
                {
                    nRemainCnt = nLen + 3;
                }
            }
            if (m_remainData.size() < nRemainCnt)// data not enough
            {
                m_remainData.prepend(byteData);
                return;
            }
            byteData.append(m_remainData.left(nRemainCnt));
            m_remainData = m_remainData.right(m_remainData.size()-nRemainCnt);
            if(!dataCheck(byteData))
            {
                handleError();
            }
            else
            {
                dispatchData(communicateUnitId,byteData);
            }
        }
        else
        {
            return;
        }

    }
}

ProxyParser::ProxyParser(QObject *parent) : QObject(parent)
{

}

void ProxyParser::handleError()
{
    QByteArray tag;
    tag.resize(1);
    tag[0] = (char)0xEE;
    int nIdx = m_remainData.indexOf(tag, 1);
    if (nIdx >= 0)
    {
        qDebug() << "ParseError" << m_remainData.toHex();
        m_remainData = m_remainData.right(m_remainData.size()-nIdx);
    }
    else
    {
        qDebug() << "ParseError" << m_remainData;
        m_remainData.clear();
    }
}

bool ProxyParser::headCheck(const QByteArray &data)
{
    if (data.size() == 0)
        return false;

    QByteArray tag;
    tag.resize(1);
    tag[0] = (char)0xEE;
    int nIdx = data.indexOf(tag, 0);
    if(nIdx != 0)
        return false;

    return true;
}

bool ProxyParser::dataCheck(const QByteArray &data)
{
    if(data.size() < 7)
        return false;

    QByteArray tag;
    tag.resize(1);
    tag[0] = (char)0xED;
    int nIdx = data.indexOf(tag, data.size()-1);
    if(nIdx != data.size()-1)
        return false;
    IData tmp(data);
    tmp.Skip(3);
    quint16 nLen = tmp.ReadShort();

    if(nLen+6 < data.size())// has crc check code
    {
        return tmp.CheckData();
    }

    return true;
}

void ProxyParser::dispatchData(quint32 communicateUnitId,QByteArray &buf)
{
#ifdef TEST_DEBUG
    ProxyWatcher::getInstance()->reciveItem(buf);
#endif

    quint8 nTag = buf.at(2);
    switch (nTag)
    {
    case Actuator::D_HANDSHAKE:
        NoCrcProxy::decode(communicateUnitId,buf);
        break;
    case Actuator::D_SET_POSITION_P:
    case Actuator::D_SET_POSITION_I:
    case Actuator::D_SET_POSITION_D:
    case Actuator::D_SET_CURRENT_P:
    case Actuator::D_SET_CURRENT_I:
    case Actuator::D_SET_VELOCITY_P:
    case Actuator::D_SET_VELOCITY_I:
    case Actuator::D_CLEAR_ERROR:
    case Actuator::D_CLEAR_HOMING:
    case Actuator::D_SET_PAIRS:
    case Actuator::D_SET_CURRENT:
    case Actuator::D_SET_POSITION:
    case Actuator::D_SET_VELOCITY:
    case Actuator::D_CHART_DATA_STATR:
    case Actuator::D_CAN_CONNECT:
    case Actuator::D_SAVE_PARAM:
    case Actuator::D_SET_DEVICE_ID:
    case Actuator::D_CHART_OPEN:
    case Actuator::D_CHART_CLOSE:
    case Actuator::D_SET_MODE:
    case Actuator::D_READ_MOTORS_SWITCH://暂时处理
    case Actuator::D_SET_SWITCH_MOTORS:
    case Actuator::D_READ_LAST_STATE:
    case Actuator::D_SET_CURRENT_ID:
    case Actuator::D_SET_PROFILE_POS_MAX_SPEED:
    case Actuator::D_SET_PROFILE_POS_ACC:
    case Actuator::D_SET_PROFILE_POS_DEC:
    case Actuator::D_SET_PROFILE_VEL_MAX_SPEED:
    case Actuator::D_SET_PROFILE_VEL_ACC:
    case Actuator::D_SET_PROFILE_VEL_DEC:
    case Actuator::D_SET_CURRENT_MAXSPEED:
    case Actuator::D_SET_CURRENT_PID_MIN:
    case Actuator::D_SET_CURRENT_PID_MAX:
    case Actuator::D_SET_VELOCITY_PID_MIN:
    case Actuator::D_SET_VELOCITY_PID_MAX:
    case Actuator::D_SET_POSITION_PID_MIN:
    case Actuator::D_SET_POSITION_PID_MAX:
    case Actuator::D_SET_CHART_THRESHOLD:
    case Actuator::D_SET_CHART_FREQUENCY:
    case Actuator::D_SET_TEMP_PROTECT:
    case Actuator::D_SET_TEMP_RECOVERY:
    case Actuator::D_SET_INVERTER_TEMP_PROTECT:
    case Actuator::D_SET_INVERTER_TEMP_RECOVERY:
    case Actuator::D_SET_FILTER_C_STATUS:
    case Actuator::D_SET_FILTER_C_VALUE:
    case Actuator::D_SET_FILTER_V_STATUS:
    case Actuator::D_SET_FILTER_V_VALUE:
    case Actuator::D_SET_FILTER_P_STATUS:
    case Actuator::D_SET_FILTER_P_VALUE:
    case Actuator::D_SET_INERTIA:
    case Actuator::D_SET_LOCK_ENERGY:
    case Actuator::D_SET_MAX_POS:
    case Actuator::D_SET_MIN_POS:
    case Actuator::D_SET_HOMING_POS:
    case Actuator::D_SET_POS_OFFSET:
    case Actuator::D_SET_HOMING_LIMIT:
    case Actuator::D_SET_HOMING_OPERATION:
    case Actuator::D_SET_HOMING_MIN:
    case Actuator::D_SET_HOMING_MAX:
    case Actuator::D_SET_HOMING_CUR_MIN:
    case Actuator::D_SET_HOMING_CUR_MAX:
    case Actuator::D_SET_CALIBRATION_ANGLE:
        NoCrcProxy::decode(communicateUnitId,buf);
        break;
    default:
        InnfosProxy::decode(communicateUnitId,buf);
        break;
    }
}

