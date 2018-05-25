#include "dataUtil.h"
#include "actuatordefine.h"
DataUtil * DataUtil::m_pInstance = nullptr;
using namespace Actuator;

Actuator::Directives DataUtil::convertToReadProxyId(Actuator::ActuatorAttribute id)
{
    if(!m_pInstance)
        m_pInstance = new DataUtil();
    QList<Directives> keys = m_pInstance->readDataMap.keys(id);

    if(keys.size() > 0)
        return keys.at(0);

    return DIRECTIVES_INVALID;
}

Actuator::Directives DataUtil::convertToSetProxyId(Actuator::ActuatorAttribute id)
{
    if(!m_pInstance)
        m_pInstance = new DataUtil();
    QList<Directives> keys = m_pInstance->setDataMap.keys(id);

    if(keys.size() > 0)
        return keys.at(0);

    return DIRECTIVES_INVALID;
}


Actuator::ActuatorAttribute DataUtil::convertToMotorDataId(Actuator::Directives id)
{
    if(!m_pInstance)
        m_pInstance = new DataUtil();

    if(m_pInstance->readDataMap.contains(id))
        return m_pInstance->readDataMap.value(id);

    if(m_pInstance->setDataMap.contains(id))
        return m_pInstance->setDataMap.value(id);

    return DATA_INVALID;
}

DataUtil::DataUtil()
{
    readDataMap.insert(D_READ_CUR_CURRENT,ACTUAL_CURRENT);
    readDataMap.insert(D_READ_CUR_VELOCITY,ACTUAL_VELOCITY);
    readDataMap.insert(D_READ_CUR_POSITION,ACTUAL_POSITION);

    readDataMap.insert(D_READ_CUR_P,CUR_PROPORTIONAL);
    readDataMap.insert(D_READ_CUR_I,CUR_INTEGRAL);
    readDataMap.insert(D_READ_VEL_P,VEL_PROPORTIONAL);
    readDataMap.insert(D_READ_VEL_I,VEL_INTEGRAL);
    readDataMap.insert(D_READ_POS_P,POS_PROPORTIONAL);
    readDataMap.insert(D_READ_POS_I,POS_INTEGRAL);
    readDataMap.insert(D_READ_POS_D,POS_DIFFERENTIAL);

    readDataMap.insert(D_READ_PROFILE_POS_MAX_SPEED,PROFILE_POS_MAX_SPEED);
    readDataMap.insert(D_READ_PROFILE_POS_ACC,PROFILE_POS_ACC);
    readDataMap.insert(D_READ_PROFILE_POS_DEC,PROFILE_POS_DEC);

    readDataMap.insert(D_READ_PROFILE_VEL_MAX_SPEED,PROFILE_VEL_MAX_SPEED);
    readDataMap.insert(D_READ_PROFILE_VEL_ACC,PROFILE_VEL_ACC);
    readDataMap.insert(D_READ_PROFILE_VEL_DEC,PROFILE_VEL_DEC);

    readDataMap.insert(D_READ_CURRENT_PID_MIN,CUR_MINIMUM);
    readDataMap.insert(D_READ_CURRENT_PID_MAX,CUR_MAXIMUM);
    readDataMap.insert(D_READ_VELOCITY_PID_MIN,VEL_OUTPUT_LIMITATION_MINIMUM);
    readDataMap.insert(D_READ_VELOCITY_PID_MAX,VEL_OUTPUT_LIMITATION_MAXIMUM);
    readDataMap.insert(D_READ_POSITION_PID_MIN,POS_OUTPUT_LIMITATION_MINIMUM);
    readDataMap.insert(D_READ_POSITION_PID_MAX,POS_OUTPUT_LIMITATION_MAXIMUM);

    setDataMap.insert(D_SET_CURRENT,CUR_IQ_SETTING);
    setDataMap.insert(D_SET_VELOCITY,VEL_SETTING);
    setDataMap.insert(D_SET_POSITION,POS_SETTING);
    setDataMap.insert(D_SET_CURRENT_ID,CUR_ID_SETTING);
    setDataMap.insert(D_SET_CURRENT_P,CUR_PROPORTIONAL);
    setDataMap.insert(D_SET_CURRENT_I,CUR_INTEGRAL);
    setDataMap.insert(D_SET_VELOCITY_P,VEL_PROPORTIONAL);
    setDataMap.insert(D_SET_VELOCITY_I,VEL_INTEGRAL);
    setDataMap.insert(D_SET_POSITION_P,POS_PROPORTIONAL);
    setDataMap.insert(D_SET_POSITION_I,POS_INTEGRAL);
    setDataMap.insert(D_SET_POSITION_D,POS_DIFFERENTIAL);

    setDataMap.insert(D_SET_PROFILE_POS_MAX_SPEED,PROFILE_POS_MAX_SPEED);
    setDataMap.insert(D_SET_PROFILE_POS_ACC,PROFILE_POS_ACC);
    setDataMap.insert(D_SET_PROFILE_POS_DEC,PROFILE_POS_DEC);

    setDataMap.insert(D_SET_PROFILE_VEL_MAX_SPEED,PROFILE_VEL_MAX_SPEED);
    setDataMap.insert(D_SET_PROFILE_VEL_ACC,PROFILE_VEL_ACC);
    setDataMap.insert(D_SET_PROFILE_VEL_DEC,PROFILE_VEL_DEC);

    setDataMap.insert(D_SET_CURRENT_PID_MIN,CUR_MINIMUM);
    setDataMap.insert(D_SET_CURRENT_PID_MAX,CUR_MAXIMUM);
    setDataMap.insert(D_SET_VELOCITY_PID_MIN,VEL_OUTPUT_LIMITATION_MINIMUM);
    setDataMap.insert(D_SET_VELOCITY_PID_MAX,VEL_OUTPUT_LIMITATION_MAXIMUM);
    setDataMap.insert(D_SET_POSITION_PID_MIN,POS_OUTPUT_LIMITATION_MINIMUM);
    setDataMap.insert(D_SET_POSITION_PID_MAX,POS_OUTPUT_LIMITATION_MAXIMUM);

    setDataMap.insert(D_SET_CHART_FREQUENCY,CHART_FREQUENCY);
    setDataMap.insert(D_SET_CHART_THRESHOLD,CHART_THRESHOLD);

    readDataMap.insert(D_READ_CHART_FREQUENCY,CHART_FREQUENCY);
    readDataMap.insert(D_READ_CHART_THRESHOLD,CHART_THRESHOLD);

    readDataMap.insert(D_READ_CHANNEL_2,DATA_CHART);
    readDataMap.insert(D_READ_CHANNEL_3,DATA_CHART);
    readDataMap.insert(D_READ_CHANNEL_4,DATA_CHART);

    readDataMap.insert(D_READ_MIN_POS,POS_LIMITATION_MINIMUM);
    readDataMap.insert(D_READ_MAX_POS,POS_LIMITATION_MAXIMUM);

    setDataMap.insert(D_SET_MIN_POS,POS_LIMITATION_MINIMUM);
    setDataMap.insert(D_SET_MAX_POS,POS_LIMITATION_MAXIMUM);

    setDataMap.insert(D_SET_HOMING_POS,HOMING_POSITION);

    setDataMap.insert(D_SET_POS_OFFSET,POS_OFFSET);
    readDataMap.insert(D_READ_POS_OFFSET,POS_OFFSET);

    readDataMap.insert(D_READ_VOLTAGE,VOLTAGE);

    readDataMap.insert(D_READ_HOMING_LIMIT,POS_LIMITATION_SWITCH);
    setDataMap.insert(D_SET_HOMING_LIMIT,POS_LIMITATION_SWITCH);

    setDataMap.insert(D_SET_HOMING_CUR_MIN,HOMING_CUR_MINIMUM);
    readDataMap.insert(D_READ_HOMING_CUR_MIN,HOMING_CUR_MINIMUM);
    setDataMap.insert(D_SET_HOMING_CUR_MAX,HOMING_CUR_MAXIMUM);
    readDataMap.insert(D_READ_HOMING_CUR_MAX,HOMING_CUR_MAXIMUM);

    readDataMap.insert(D_READ_CURRENT_SCALE,CURRENT_SCALE);

    setDataMap.insert(D_SET_FILTER_C_STATUS,FILTER_C_STATUS);
    readDataMap.insert(D_READ_FILTER_C_STATUS,FILTER_C_STATUS);
    setDataMap.insert(D_SET_FILTER_C_VALUE,FILTER_C_VALUE);
    readDataMap.insert(D_READ_FILTER_C_VALUE,FILTER_C_VALUE);

    setDataMap.insert(D_SET_FILTER_V_STATUS,FILTER_V_STATUS);
    readDataMap.insert(D_READ_FILTER_V_STATUS,FILTER_V_STATUS);
    setDataMap.insert(D_SET_FILTER_V_VALUE,FILTER_V_VALUE);
    readDataMap.insert(D_READ_FILTER_V_VALUE,FILTER_V_VALUE);

    setDataMap.insert(D_SET_FILTER_P_STATUS,FILTER_P_STATUS);
    readDataMap.insert(D_READ_FILTER_P_STATUS,FILTER_P_STATUS);
    setDataMap.insert(D_SET_FILTER_P_VALUE,FILTER_P_VALUE);
    readDataMap.insert(D_READ_FILTER_P_VALUE,FILTER_P_VALUE);

    setDataMap.insert(D_SET_INERTIA,INERTIA);
    readDataMap.insert(D_READ_INERTIA,INERTIA);

    setDataMap.insert(D_SET_LOCK_ENERGY,LOCK_ENERGY);
    readDataMap.insert(D_READ_LOCK_ENERGY,LOCK_ENERGY);

    readDataMap.insert(D_READ_TEMP_MOTOR,ACTUATOR_TEMPERATURE);
    readDataMap.insert(D_READ_TEMP_INVERTER,INVERTER_TEMPERATURE);
    readDataMap.insert(D_READ_TEMP_PROTECT,ACTUATOR_PROTECT_TEMPERATURE);
    readDataMap.insert(D_READ_TEMP_RECOVERY,ACTUATOR_RECOVERY_TEMPERATURE);

    setDataMap.insert(D_SET_TEMP_PROTECT,ACTUATOR_PROTECT_TEMPERATURE);
    setDataMap.insert(D_SET_TEMP_RECOVERY,ACTUATOR_RECOVERY_TEMPERATURE);

    readDataMap.insert(D_READ_INVERTER_TEMP_PROTECT,INVERTER_PROTECT_TEMPERATURE);
    readDataMap.insert(D_READ_INVERTER_TEMP_RECOVERY,INVERTER_RECOVERY_TEMPERATURE);
    setDataMap.insert(D_SET_INVERTER_TEMP_PROTECT,INVERTER_PROTECT_TEMPERATURE);
    setDataMap.insert(D_SET_INVERTER_TEMP_RECOVERY,INVERTER_RECOVERY_TEMPERATURE);

    setDataMap.insert(D_SWITCH_CALIBRATION,CALIBRATION_SWITCH);
    setDataMap.insert(D_SET_CALIBRATION_ANGLE,CALIBRATION_ANGLE);
    readDataMap.insert(D_READ_CALIBRATION_SWITCH,CALIBRATION_SWITCH);
    readDataMap.insert(D_READ_CALIBRATION_ANGLE,CALIBRATION_ANGLE);

    setDataMap.insert(D_SET_SWITCH_MOTORS,ACTUATOR_SWITCH);
    readDataMap.insert(D_READ_MOTORS_SWITCH,ACTUATOR_SWITCH);

    readDataMap.insert(D_READ_VERSION,FIRMWARE_VERSION);
	
	
    setDataMap.insert(D_SET_MODE,MODE_ID);
    setDataMap.insert(D_SET_DEVICE_ID,DEVICE_ID);
    readDataMap.insert(D_CHECK_ERROR,ERROR_ID);
    setDataMap.insert(D_CHECK_ERROR,ERROR_ID);

    readDataMap.insert(D_READ_CURRENT_MAXSPEED,CUR_MAXSPEED);
    setDataMap.insert(D_SET_CURRENT_MAXSPEED,CUR_MAXSPEED);

    readDataMap.insert(D_READ_CUMULATIVE_TIME,CUMULATIVE_TIME);

    readDataMap.insert(D_READ_MOTOR_MODE,MODE_ID);
    //这个比较特殊 两条协议对应一个属性，处理时候要判断是开还是关
    setDataMap.insert(D_CHART_OPEN,CHART_SWITCH);
    setDataMap.insert(D_CHART_CLOSE,CHART_SWITCH);

    readDataMap.insert(D_READ_RESERVE_0,RESERVE_0);
    readDataMap.insert(D_READ_RESERVE_1,RESERVE_1);
    readDataMap.insert(D_READ_RESERVE_2,RESERVE_2);
    readDataMap.insert(D_READ_RESERVE_3,RESERVE_3);
}


