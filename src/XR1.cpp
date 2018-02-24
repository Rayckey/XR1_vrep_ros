#include "XR1.h"
#include <tf2>
#include "ros.h"
#include "ros/ros.h"
#include "vrep_test/JointAngles.h"

XR1::XR1(); :
{
    LeftArmDHParameters[0] = 
}

XR1::~XR1()
{
    delete this;
}


void XR1::tfConversion(std::vector<double> JointAngles){

}