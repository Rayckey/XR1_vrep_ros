/*
*自动识别已连接执行器
*/

#include <iostream>
#include "actuatorcontroller.h"
#include <thread>
#include <signal.h>
using namespace std;

bool bExit = false;
void processSignal(int sign)
{
    ActuatorController::getInstance()->closeAllActuators();
    //延时保证关机成功
    this_thread::sleep_for(std::chrono::milliseconds(200));
    bExit = true;
}

int main(int argc, char *argv[])
{    
    //初始化控制器
    ActuatorController::initController(argc,argv,Actuator::Via_Ethernet);
    ActuatorController * pController = ActuatorController::getInstance();
    signal(SIGINT,processSignal);
    //关联控制器的操作信号
    int nOperationConnection = pController->m_sOperationFinished.s_Connect([=](uint8_t nDeviceId,uint8_t operationType){
        switch (operationType) {
        case Actuator::Recognize_Finished://自动识别完成
            if(pController->hasAvailableActuator())
            {
                vector<uint8_t> idArray = pController->getActuatorIdArray();
                cout << "Number of connected actuators:" << idArray.size() << endl;
            }
            break;
        default:
            break;
        }
    });

    //关联错误信号
    int nErrorConnection = pController->m_sError.s_Connect([=](uint8_t nDeviceId,uint16_t nErrorType,string errorInfo){
        if(nDeviceId==0)
        {
            cout << "Error: " << (int)nErrorType << " " << errorInfo << endl;
        }
        else
        {
            cout << "Actuator " << (int)nDeviceId << " " <<"error " << (int)nErrorType << " " << errorInfo << endl;
        }
    });
    //自动识别已连接执行器
    pController->autoRecoginze();
    //执行控制器事件循环
    while (!bExit)
    {
        ActuatorController::processEvents();
    }
    //断开信号连接
    pController->m_sOperationFinished.s_Disconnect(nOperationConnection);
    pController->m_sError.s_Disconnect(nErrorConnection);

    return 0;
}
