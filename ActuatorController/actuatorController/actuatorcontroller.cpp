#include "actuatorcontroller.h"
#include <QDebug>
#include "innfosproxy.h"
#include <QTimer>

QCoreApplication * ActuatorController::m_pQtCore = nullptr;
ActuatorController * ActuatorController::m_pInstance = nullptr;

ActuatorController::ActuatorController():
    QObject()
{
    m_lConnectionIds.push_back(mediator->m_sRequestBack.connect_member(this,&ActuatorController::onRequestCallback));
    m_lConnectionIds.push_back(mediator->m_sRecognizeFinished.connect_member(this,&ActuatorController::finishRecognizeCallback));
    m_lConnectionIds.push_back(mediator->m_sError.connect_member(this,&ActuatorController::errorOccur));
    m_lConnectionIds.push_back(mediator->m_sMotorAttrChanged.connect_member(this,&ActuatorController::motorAttrChanged));
    m_lConnectionIds.push_back(mediator->m_sChartValueChange.connect_member(this,&ActuatorController::chartValueChange));
    m_lConnectionIds.push_back(mediator->m_sNewChartStart.connect_member(this,&ActuatorController::startNewChart));
#ifdef IMU_ENABLE
    m_lConnectionIds.push_back(mediator->m_sQuaternion.connect_member(this,&ActuatorController::receiveQuaternion));
#endif
}

void ActuatorController::initController(int &argc, char **argv, int nCommunicationType)
{
    if(m_pInstance != nullptr)
    {
        return;
    }
    mediator->initCommunication(nCommunicationType);
    if(QCoreApplication::instance() == nullptr)
        m_pQtCore = new QCoreApplication(argc,argv);
    m_pInstance= new ActuatorController();
}

ActuatorController *ActuatorController::getInstance()
{
    return m_pInstance;
}

void ActuatorController::processEvents()
{
    QCoreApplication::processEvents();
}

ActuatorController::~ActuatorController()
{
    mediator->m_sRequestBack.s_Disconnect(m_lConnectionIds);
    mediator->m_sRecognizeFinished.s_Disconnect(m_lConnectionIds);
    mediator->m_sError.s_Disconnect(m_lConnectionIds);
    mediator->m_sMotorAttrChanged.s_Disconnect(m_lConnectionIds);
    mediator->m_sChartValueChange.s_Disconnect(m_lConnectionIds);
    mediator->m_sNewChartStart.s_Disconnect(m_lConnectionIds);
#ifdef IMU_ENABLE
    mediator->m_sQuaternion.s_Disconnect(m_lConnectionIds);
#endif
    delete m_pQtCore;
    //Mediator::destroyAllStaticObjects();
}

void ActuatorController::autoRecoginze()
{
    mediator->autoRecognize();
}

bool ActuatorController::hasAvailableActuator() const
{
    return getActuatorIdArray().size() > 0;
}

void ActuatorController::finishRecognizeCallback()
{
    qDebug()<<"finished";
    QTimer::singleShot(100,[=]{
        qDebug() << "really finished";
        m_sOperationFinished.s_Emit(0,Actuator::Recognize_Finished);
    }); //delay to insure all requests have acknowledges
}

void ActuatorController::onRequestCallback(uint8_t nDeviceId, uint8_t nProxyId, double value)
{
    switch (nProxyId) {

    case Actuator::D_SAVE_PARAM:
        if(value > 0)
            m_sOperationFinished.s_Emit(nDeviceId,Actuator::Save_Params_Finished);
        else
            m_sOperationFinished.s_Emit(nDeviceId,Actuator::Save_Params_Failed);
        break;
    case Actuator::D_SET_SWITCH_MOTORS:
        if((int)value == Actuator::ACTUATOR_SWITCH_ON)
        {
            QTimer::singleShot(3500,[=]{//等待3.5s,开机动作才完成
                m_sOperationFinished.s_Emit(nDeviceId,Actuator::Launch_Finished);
            });
        }
        else
        {
            m_sOperationFinished.s_Emit(nDeviceId,Actuator::Close_Finished);
        }
        break;
    default:
        m_sRequestBack.s_Emit(nDeviceId,nProxyId,value);
        break;
    }
}

void ActuatorController::errorOccur(uint8_t nDeviceId, uint16_t errorId, string errorStr)
{
    m_sError.s_Emit(nDeviceId,errorId,errorStr);
}

void ActuatorController::motorAttrChanged(uint8_t nDeviceId, uint8_t attrId, double value)
{
    m_sActuatorAttrChanged.s_Emit(nDeviceId,attrId,value);
}

void ActuatorController::startNewChart()
{
    m_sNewChartStart.s_Emit();
}

void ActuatorController::chartValueChange(uint8_t channelId, double value)
{
    m_sChartValueChange.s_Emit(channelId,value);
}

vector<uint8_t> ActuatorController::getActuatorIdArray() const
{
    QList<quint8> motorKeyList = motorDataMgrInstance->getMotorsMap().uniqueKeys();
    return convertQListToVector(motorKeyList);
}

void ActuatorController::activeActuatorMode(vector<uint8_t> idArray, const Actuator::ActuatorMode nMode)
{
    motorDataMgrInstance->activeMotorModeInBatch(convertVectorToQList(idArray),nMode);
}

void ActuatorController::launchAllActuators()
{
    QList<quint8> motorKeyList = motorDataMgrInstance->getMotorsMap().uniqueKeys();
    for(int i=0;i<motorKeyList.size();++i)
    {
        motorDataMgrInstance->setMotorDataAttrByUser(motorKeyList[i],Actuator::ACTUATOR_SWITCH,Actuator::ACTUATOR_SWITCH_ON);
    }

}

void ActuatorController::closeAllActuators()
{
    QList<quint8> motorKeyList = motorDataMgrInstance->getMotorsMap().uniqueKeys();
    for(int i=0;i<motorKeyList.size();++i)
    {
        motorDataMgrInstance->setMotorDataAttrByUser(motorKeyList[i],Actuator::ACTUATOR_SWITCH,Actuator::ACTUATOR_SWITCH_OFF);
    }
}

void ActuatorController::launchActuator(uint8_t id)
{
    motorDataMgrInstance->setMotorDataAttrByUser(id,Actuator::ACTUATOR_SWITCH,Actuator::ACTUATOR_SWITCH_ON);
}

void ActuatorController::closeActuator(uint8_t id)
{
    motorDataMgrInstance->setMotorDataAttrByUser(id,Actuator::ACTUATOR_SWITCH,Actuator::ACTUATOR_SWITCH_OFF);
}

void ActuatorController::switchAutoRefresh(uint8_t id, bool bOpen)
{
    motorDataMgrInstance->switchAutoRequestActual(id,bOpen);
}

void ActuatorController::setAutoRefreshInterval(uint8_t id, uint32_t mSec)
{
    motorDataMgrInstance->setAutoRequestInterval(id,mSec);
}

void ActuatorController::setPosition(uint8_t id, double pos)
{
    motorDataMgrInstance->setMotorDataAttrByUser(id,Actuator::POS_SETTING,pos);
}

void ActuatorController::setVelocity(uint8_t id, double vel)
{
    motorDataMgrInstance->setMotorDataAttrByUser(id,Actuator::VEL_SETTING,vel);
}

void ActuatorController::setCurrent(uint8_t id, double current)
{
    motorDataMgrInstance->setMotorDataAttrByUser(id,Actuator::CUR_IQ_SETTING,current);
}

double ActuatorController::getPosition(uint8_t id, bool bRefresh) const
{
    if(bRefresh)
        motorDataMgrInstance->regainData(id,Actuator::ACTUAL_POSITION);
    return motorDataMgrInstance->getMotorDataAttrValueAsDouble(id,Actuator::ACTUAL_POSITION);
}

double ActuatorController::getVelocity(uint8_t id, bool bRefresh) const
{
    if(bRefresh)
        motorDataMgrInstance->regainData(id,Actuator::ACTUAL_VELOCITY);
    return motorDataMgrInstance->getMotorDataAttrValueAsDouble(id,Actuator::ACTUAL_VELOCITY);
}

double ActuatorController::getCurrent(uint8_t id, bool bRefresh) const
{
    if(bRefresh)
        motorDataMgrInstance->regainData(id,Actuator::ACTUAL_CURRENT);
    return motorDataMgrInstance->getMotorDataAttrValueAsDouble(id,Actuator::ACTUAL_CURRENT);
}

void ActuatorController::setActuatorAttribute(uint8_t id, Actuator::ActuatorAttribute attrId, double value)
{
    motorDataMgrInstance->setMotorDataAttrByUser(id,attrId,value);
}

double ActuatorController::getActuatorAttribute(uint8_t id, Actuator::ActuatorAttribute attrId) const
{
    return motorDataMgrInstance->getMotorDataAttrValueAsDouble(id,attrId);
}

void ActuatorController::saveAllParams(uint8_t id)
{
    motorDataMgrInstance->saveAllParams(id);
}

void ActuatorController::clearHomingInfo(uint8_t id)
{
    motorDataMgrInstance->clearHomingInfo(id);
}

void ActuatorController::setHomingOperationMode(uint8_t id, uint8_t nMode)
{
    motorDataMgrInstance->setHomingOperationMode(id,nMode);
}

void ActuatorController::setMinPosLimit(uint8_t id)
{
    motorDataMgrInstance->sendCmd(id,Actuator::D_SET_HOMING_MIN);
}

void ActuatorController::setMinPosLimit(uint8_t id, double posValue)
{
    motorDataMgrInstance->setMotorDataAttrByUser(id,Actuator::POS_LIMITATION_MINIMUM,posValue);
}

void ActuatorController::setMaxPosLimit(uint8_t id)
{
    motorDataMgrInstance->sendCmd(id,Actuator::D_SET_HOMING_MAX);
}

void ActuatorController::setMaxPosLimit(uint8_t id, double posValue)
{
    motorDataMgrInstance->setMotorDataAttrByUser(id,Actuator::POS_LIMITATION_MAXIMUM,posValue);
}

void ActuatorController::setHomingPosition(uint8_t id, double posValue)
{
    motorDataMgrInstance->setMotorDataAttrByUser(id,Actuator::HOMING_POSITION,posValue);
}

void ActuatorController::openChartChannel(uint8_t id, uint8_t nChannelId)
{
    motorDataMgrInstance->openChartChannel(id,nChannelId);
}

void ActuatorController::closeChartChannel(uint8_t id, uint8_t nChannelId)
{
    motorDataMgrInstance->closeChartChannel(id,nChannelId);
}

void ActuatorController::switchChartAllChannel(uint8_t id, bool bOn)
{
    motorDataMgrInstance->switchChartAllChannel(id,bOn);
}

void ActuatorController::setCurrentChartMode(uint8_t id, uint8_t mode)
{
    motorDataMgrInstance->sendCmd(id,Actuator::D_SET_CUR_TRIGGER_MODE,mode);
}


void ActuatorController::regainAttrbute(uint8_t id, uint8_t attrId)
{
    motorDataMgrInstance->regainData(id,(Actuator::ActuatorAttribute)attrId);
}

vector<uint16_t> ActuatorController::getErrorHistory(uint8_t id)
{
    QVector<uint16_t> errors = motorDataMgrInstance->motorErrorHistory(id);
    vector<uint16_t> history;
    for(int i=0;i<errors.size();++i)
    {
        history.push_back(errors.at(i));
    }
    return history;
}
void ActuatorController::reconnect(uint8_t id)
{
    motorDataMgrInstance->reconnect(id);
}
void ActuatorController::clearError(uint8_t id)
{
    motorDataMgrInstance->clearError(id);
}

string ActuatorController::versionString() const
{
    return mediator->versionString().toStdString();
}

#ifdef IMU_ENABLE
void ActuatorController::requestAllQuaternions()
{
    mediator->requestQuaternion(2);
}

void ActuatorController::requestSingleQuaternion()
{
    mediator->requestQuaternion(1);
}

void ActuatorController::receiveQuaternion(quint8 nIMUId, double w, double x, double y, double z)
{
    m_sQuaternion.s_Emit(nIMUId,w,x,y,z);
}
#endif

vector<uint8_t> ActuatorController::convertQListToVector(const QList<quint8> &qList) const
{
    vector<uint8_t> idVector;
    for(int i=0;i<qList.size();++i)
    {
        idVector.push_back(qList[i]);
    }
    return idVector;
}

QList<quint8> ActuatorController::convertVectorToQList(const vector<uint8_t> &cVector) const
{
    QList<quint8> qList;
    for(int i=0;i<cVector.size();++i)
    {
        qList.push_back(cVector[i]);
    }
    return qList;
}
