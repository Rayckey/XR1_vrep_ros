#ifndef XR1_H
#define XR1_H
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "ros/ros.h"
#include "vrep_test/JointAngles.h"



class XR1
{

public:
    XR1();
    ~XR1();

    void subscribeJointCurrentPosition(vrep_test::JointAngles msg);
    // void ActivateSubscriber(ros::NodeHandle nh);
private:
    std::vector<geometry_msgs::TransformStamped> tfConversion(double JointAngles [27]);
    double DHTableLookUp(const int row, const int col);
    geometry_msgs::TransformStamped DH2tf(int row, double angle);	
    // std::vector<double> DHTable[22*4];
};
#endif // MAINWINDOW_H
