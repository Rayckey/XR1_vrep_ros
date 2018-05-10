﻿#ifndef ACTUATORDEFINE_H
#define ACTUATORDEFINE_H

namespace Actuator {

//连接状态，用于执行器和CAN的连接状态判断：
    enum ConnectStatus
    {//connect status
        NO_CONNECT,
        CAN_CONNECTED=0x02,
        ACTUATOR_CONNECTED=0x04,
    };

    //通道ID,用于标识执行器图表数据的通道索引：
    enum Channel_ID
    {
        channel_1=0,
        channel_2,
        channel_3,
        channel_4,
        channel_cnt
    };

//错误类型定义，定义了执行器内部和连接等错误代码：
    enum ErrorsDefine
    {
        ERR_NONE = 0,
    //执行器过压错误
        ERR_ACTUATOR_OVERVOLTAGE=0x01,
    //执行器欠压错误
        ERR_ACTUATOR_UNDERVOLTAGE=0x02,
    //执行器堵转错误
        ERR_ACTUATOR_LOCKED_ROTOR=0x04,
    //执行器过温错误
        ERR_ACTUATOR_OVERHEATING=0x08,
    //执行器读写错误
        ERR_ACTUATOR_READ_OR_WIRTE=0x10,
    //执行器多圈计数错误
        ERR_ACTUATOR_MULTI_TURN=0x20,
    //执行器逆变器温度器错误
        ERR_INVERTOR_TEMPERATURE_SENSOR=0x40,
    //执行器CAN通信错误
        ERR_CAN_COMMUNICATION=0x80,
    //执行器温度传感器错误
        ERR_ACTUATOR_TEMPERATURE_SENSOR=0x100,
    //执行器DRV保护
        ERR_DRV_PROTECTION=0x400,
    //执行器ID不唯一错误
        ERR_ID_UNUNIQUE=0x800,
    //执行器未连接错误
        ERR_ACTUATOR_DISCONNECTION=0x801,
    //CAN通信转换板未连接错误
        ERR_CAN_DISCONNECTION=0x802,
    //无可用ip地址错误
        ERR_IP_ADDRESS_NOT_FOUND=0x803,
    //执行器非正常关机错误
        ERR_ABNORMAL_SHUTDOWN=0x804,
    //执行器关机时参数保存错误
        ERR_SHUTDOWN_SAVING=0x805,
        ERR_UNKOWN=0xffff
    };

//在线状态，用于标识执行器是否处于连接状态：
    enum OnlineStatus
    {
        Status_Online=0x00,
        Status_Offline=0x01,
    };

//开关状态，标识执行器的开关机状态：
    enum SwitchStatus
    {
        ACTUATOR_SWITCH_OFF=0,
        ACTUATOR_SWITCH_ON=1,
    };

//图表开关，用于标识执行器图表功能的开启或关闭：
    enum ChartSwitchStatus
    {
        CHART_SWITCH_OFF=0,
        CHART_SWITCH_ON=1,
    };

//电流环图表索引，用于标识电流图表是IQ值还是ID值
    enum CurrnetChart
    {
        IQ_CHART=0,
        ID_CHART=1,
    };
//归零模式，分为手动和自动两种
    enum HomingOperationMode
    {
        Homing_Auto=0,
        Homing_Manual,
    };

//通信方式，可通过以太网或者串口两种方式与执行器通信，初始化执行器控制器时候要指定方式，默认为以太网通信：
    enum CommunicationType
    {
        Via_Ethernet,
        Via_Serialport,
    };


    //操作标识，标识操作完成，可用于判断执行器控制器的指令执行状态：
    enum OperationFlags
    {
    //自动识别完成
        Recognize_Finished,
    //执行器启动完成（如果连接的是多个执行器，会触发多次启动完成信号）
        Launch_Finished,
    //执行器关闭完成（如果连接的是多个执行器，会触发多次关闭完成信号）
        Close_Finished,
    //执行器参数保存完成（如果连接的是多个执行器，会触发多次参数保存完成信号）
        Save_Params_Finished,
        //执行器参数保存失败
        Save_Params_Failed,
    //暂未实现
        Attribute_Change_Finished,
    };

//执行器模式，标识当前执行器的模式：
    enum ActuatorMode
    {
        Mode_None,
        Mode_Cur,//电流模式
        Mode_Vel,//速度模式
        Mode_Pos,//位置模式
        Mode_Teaching,//暂未实现
        Mode_Profile_Pos=6,//profile位置模式，比较于位置模式，该模式有加速减速过程
        Mode_Profile_Vel, //profile速度模式，比较于速度模式，该模式有加速减速过程

        Mode_Homing,//归零模式
    };

//执行器属性，标识了执行器所有相关属性：
    enum ActuatorAttribute
    {
        CUR_IQ_SETTING,//电流IQ值
        CUR_PROPORTIONAL,//电流比例
        CUR_INTEGRAL,//电流积分
        CUR_ID_SETTING,//电流ID值
        CUR_MINIMUM,//预留
        CUR_MAXIMUM,//预留
        CUR_NOMINAL,//预留
        CUR_OUTPUT,//预留
        CUR_MAXSPEED,//电流环最大速度
        ACTUAL_CURRENT,//当前电流值
        VEL_SETTING,//速度设置
        VEL_PROPORTIONAL,//速度比例
        VEL_INTEGRAL,//速度积分
        VEL_OUTPUT_LIMITATION_MINIMUM,//速度环输出最小电流比例
        VEL_OUTPUT_LIMITATION_MAXIMUM,//速度环输出最大电流比例
        ACTUAL_VELOCITY,//速度值
        POS_SETTING,//位置设置
        POS_PROPORTIONAL,//位置比例
        POS_INTEGRAL,//位置积分
        POS_DIFFERENTIAL,//位置微分
        POS_OUTPUT_LIMITATION_MINIMUM,//位置环输出最小速度比例
        POS_OUTPUT_LIMITATION_MAXIMUM,//位置环输出最大速度比例
        POS_LIMITATION_MINIMUM,//最小位置限制
        POS_LIMITATION_MAXIMUM,//最大位置限制
        HOMING_POSITION,//归零位置
        ACTUAL_POSITION,//当前位置
        PROFILE_POS_MAX_SPEED,//profile position模式最大速度
        PROFILE_POS_ACC,//profile position模式加速度
        PROFILE_POS_DEC,//profile position模式减速速度
        PROFILE_VEL_MAX_SPEED,//profile velocity模式最大速度
        PROFILE_VEL_ACC,//profile velocity模式加速度
        PROFILE_VEL_DEC,//profile velocity模式减速速度
        CHART_FREQUENCY,//图像频率
        CHART_THRESHOLD,//图像阈值
        CHART_SWITCH,//图像开关
        POS_OFFSET,//位置偏移
        VOLTAGE,//电压
        POS_LIMITATION_SWITCH,//开启或关闭位置限制
        HOMING_CUR_MAXIMUM,//归零最大电流
        HOMING_CUR_MINIMUM,//归零最小小电流
        CURRENT_SCALE,//物理最大电流值
        VELOCITY_SCALE,//速度最大电流值
        FILTER_C_STATUS,//电流环滤波是否开启
        FILTER_C_VALUE,//电流环滤波值
        FILTER_V_STATUS,//速度环滤波是否开启
        FILTER_V_VALUE,//速度环滤波值
        FILTER_P_STATUS,//位置环滤波是否开启
        FILTER_P_VALUE,//位置环滤波值
        INERTIA,//惯量
        LOCK_ENERGY,//堵转保护能量
        ACTUATOR_TEMPERATURE,//执行器温度
        INVERTER_TEMPERATURE,//逆变器温度
        ACTUATOR_PROTECT_TEMPERATURE,//执行器保护温度
        ACTUATOR_RECOVERY_TEMPERATURE,//执行器恢复温度
        INVERTER_PROTECT_TEMPERATURE,//逆变器保护温度
        INVERTER_RECOVERY_TEMPERATURE,//逆变器恢复温度
        CALIBRATION_SWITCH,//预留
        CALIBRATION_ANGLE,//预留
        ACTUATOR_SWITCH,//执行器开关机
        FIRMWARE_VERSION,//执行器固件版本
        ONLINE_STATUS,//执行器是否在线
        DEVICE_ID,//执行器Id
        SN_ID,//执行器SN号
        MODE_ID,//执行器当前模式
        ERROR_ID,//错误代码
        RESERVE_0,
        RESERVE_1,
        RESERVE_2,
        RESERVE_3,
        DATA_CNT,
        DATA_CHART,//预留
        DATA_INVALID,
    };

    enum Directives
    {
        D_HANDSHAKE=0x00,
        D_READ_VERSION=0x01,
        D_READ_ADDRESS=0x02,
        D_READ_CONFIG=0x03,
        D_READ_CUR_CURRENT=0x04,
        D_READ_CUR_VELOCITY=0x05,
        D_READ_CUR_POSITION=0x06,
        D_SET_MODE=0x07,
        D_SET_CURRENT=0x08,//设置当前q轴电流
        D_SET_VELOCITY=0x09,
        D_SET_POSITION=0x0a,
        D_SET_PAIRS=0x0b,
        D_SET_CURRENT_ID=0x0c,//设置当前d轴电流
        D_SAVE_PARAM=0x0d,//
        D_SET_CURRENT_P=0x0e,//电流环的p
        D_SET_CURRENT_I=0x0f,
        D_SET_VELOCITY_P=0x10,
        D_SET_VELOCITY_I=0x11,
        D_SET_POSITION_P=0x12,
        D_SET_POSITION_I=0x13,
        D_SET_POSITION_D=0x14,
        D_READ_CUR_P=0x15,
        D_READ_CUR_I=0x16,
        D_READ_VEL_P=0x17,
        D_READ_VEL_I=0x18,
        D_READ_POS_P=0x19,
        D_READ_POS_I=0x1a,
        D_READ_POS_D=0x1b,
        D_READ_PROFILE_POS_MAX_SPEED=0x1c,
        D_READ_PROFILE_POS_ACC=0x1d,
        D_READ_PROFILE_POS_DEC=0x1e,
        D_SET_PROFILE_POS_MAX_SPEED=0x1f,
        D_SET_PROFILE_POS_ACC=0x20,
        D_SET_PROFILE_POS_DEC=0x21,
        D_READ_PROFILE_VEL_MAX_SPEED=0x22,
        D_READ_PROFILE_VEL_ACC=0x23,
        D_READ_PROFILE_VEL_DEC=0x24,
        D_SET_PROFILE_VEL_MAX_SPEED=0x25,
        D_SET_PROFILE_VEL_ACC=0x26,
        D_SET_PROFILE_VEL_DEC=0x27,
        D_READ_CURRENT_MAXSPEED=0x28,
        D_SET_CURRENT_MAXSPEED=0x29,
        D_SET_SWITCH_MOTORS=0x2a,
        D_READ_MOTORS_SWITCH=0x2b,
        D_SET_CURRENT_PID_MIN = 0x2e,//设置电流环的pid的上下限
        D_SET_CURRENT_PID_MAX=0x2f,
        D_SET_VELOCITY_PID_MIN=0x30,
        D_SET_VELOCITY_PID_MAX=0x31,
        D_SET_POSITION_PID_MIN=0x32,
        D_SET_POSITION_PID_MAX=0x33,
        D_READ_CURRENT_PID_MIN=0x34,//读取电流环的pid的上下限
        D_READ_CURRENT_PID_MAX=0x35,
        D_READ_VELOCITY_PID_MIN=0x36,
        D_READ_VELOCITY_PID_MAX=0x37,
        D_READ_POSITION_PID_MIN=0x38,
        D_READ_POSITION_PID_MAX=0x39,
        D_READ_CHANNEL_2=0x3a,
        D_READ_CHANNEL_3=0x3b,
        D_READ_CHANNEL_4=0x3c,
        D_SET_DEVICE_ID=0x3d,
        D_SOFTWARE_CLOSE=0x3e,
        D_SET_CHART_THRESHOLD=0x3f,
        D_SET_CHART_FREQUENCY=0x40,
        D_READ_CHART_THRESHOLD=0x41,
        D_READ_CHART_FREQUENCY=0x42,
        D_CHART_DATA_STATR=0x43,
        D_CAN_CONNECT=0x44,
        D_READ_VOLTAGE=0x45,
        D_CHART_OPEN=0x46,
        D_CHART_CLOSE=0x47,
        D_CHANNEL2_OPEN=0x48,//
        D_CHANNEL2_CLOSE=0x49,//
        D_CHANNEL3_OPEN=0x4a,//
        D_CHANNEL3_CLOSE=0x4b,//
        D_CHANNEL4_OPEN=0x4c,//
        D_CHANNEL4_CLOSE=0x4d,//
        D_READ_CHANNEL_1=0x4e,
        D_SET_VOLTAGE=0x4f,//
        D_CRC_ERROR=0x50,
        D_CHANNEL1_OPEN=0x51,
        D_CHANNEL1_CLOSE=0x52,
        D_READ_CURRENT_SCALE=0x53,
        D_SET_CUR_TRIGGER_MODE=0x54,//
        D_READ_MOTOR_MODE=0x55,
        D_READ_TEMP_MOTOR=0x5f,
        D_READ_TEMP_INVERTER=0x60,
        D_SET_TEMP_PROTECT=0x6b,
        D_READ_TEMP_PROTECT=0x6c,
        D_SET_TEMP_RECOVERY=0x6d,
        D_READ_TEMP_RECOVERY=0x6e,
        D_SET_INVERTER_TEMP_PROTECT=0x61,
        D_READ_INVERTER_TEMP_PROTECT=0x62,
        D_SET_INVERTER_TEMP_RECOVERY=0x63,
        D_READ_INVERTER_TEMP_RECOVERY=0x64,

        D_SET_FILTER_C_STATUS=0x70,
        D_READ_FILTER_C_STATUS=0x71,
        D_SET_FILTER_C_VALUE=0x72,
        D_READ_FILTER_C_VALUE=0x73,
        D_SET_FILTER_V_STATUS=0x74,
        D_READ_FILTER_V_STATUS=0x75,
        D_SET_FILTER_V_VALUE=0x76,
        D_READ_FILTER_V_VALUE=0x77,
        D_SET_FILTER_P_STATUS=0x78,
        D_READ_FILTER_P_STATUS=0x79,
        D_SET_FILTER_P_VALUE=0x7a,
        D_READ_FILTER_P_VALUE=0x7b,
        D_SET_INERTIA = 0x7c,
        D_READ_INERTIA = 0x7d,
        D_SET_LOCK_ENERGY=0x7e,
        D_READ_LOCK_ENERGY=0x7f,

        D_SET_MAX_POS=0x83,//上下限
        D_SET_MIN_POS=0x84,
        D_READ_MAX_POS=0x85,
        D_READ_MIN_POS=0x86,
        D_SET_HOMING_POS=0x87,
        D_CLEAR_HOMING=0x88,//清除homing相关信息
        D_SET_POS_OFFSET=0x89,
        D_READ_POS_OFFSET=0x8a,
        D_READ_HOMING_LIMIT=0x8b,
        D_SET_HOMING_LIMIT=0x8c,
        D_SET_HOMING_OPERATION=0x8d,
        D_SET_HOMING_MIN=0x8e,
        D_SET_HOMING_MAX=0x8f,
        D_SET_HOMING_CUR_MIN=0x90,
        D_SET_HOMING_CUR_MAX=0x91,
        D_READ_HOMING_CUR_MIN=0x92,
        D_READ_HOMING_CUR_MAX=0x93,
        D_SWITCH_CALIBRATION=0xa0,
        D_READ_CALIBRATION_SWITCH=0xa1,
        D_START_CALIBRATION=0xa2,
        D_SET_CALIBRATION_ANGLE=0xa3,
        D_READ_CALIBRATION_ANGLE=0xa4,
        D_SWITCH_CALIBRATION_VEL=0xa5,

        D_READ_RESERVE_0=0xd0,
        D_READ_RESERVE_1=0xd1,
        D_READ_RESERVE_2=0xd2,
        D_READ_RESERVE_3=0xd3,
        D_READ_LAST_STATE=0xb0,//读取上一次状态（是否正常关机）

        D_IP_BROADCAST=0xc0,//广播查找ip地址
        D_TMP_COMMAND=0xc1,//与中间板通信的协议指令

        D_CLEAR_ERROR=0xfe,//清理错误
        D_CHECK_ERROR=0xff,//错误提示
        DIRECTIVES_INVALID,
    };

}
#endif // ACTUATORDEFINE_H
