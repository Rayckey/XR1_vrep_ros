#ifndef XR1_H
#define XR1_H

#include <tf2>
#include "ros.h"
#include "ros/ros.h"
#include "vrep_test/JointAngles.h"

namespace vrep_test {
class XR1;
}

class XR1 :
{

public:
    XR1();
    ~XR1();
    void tfConversion(std::vector<double> JointAngles)
private:
    double LeftArmDHParameters [7];
    double RightArmDHParameters [7];
    double BodyDHParameters [7];
};

#endif // MAINWINDOW_H
