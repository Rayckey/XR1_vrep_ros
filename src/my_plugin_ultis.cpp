#include "my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include "ros/ros.h"
#include "XR1.h"
#include <QTimer>
#include <QMessageBox>
#include <QLabel>
#include <QGroupBox>
#include <QFile>
#include <QTime>
#include <QTextStream>



#define PI 3.141592654
namespace vrep_test {


void MyPlugin::setup_main_joint_limit() {

    joint_lower_limit.push_back(-0.691);
    joint_upper_limit.push_back( 0.691);

    joint_lower_limit.push_back(-1.57);
    joint_upper_limit.push_back(1.57);

    joint_lower_limit.push_back(-0.691);
    joint_upper_limit.push_back( 0.691);

    joint_lower_limit.push_back(-1.01);
    joint_upper_limit.push_back( 1.01);

    joint_lower_limit.push_back(-1.57);
    joint_upper_limit.push_back(1.57);

    joint_lower_limit.push_back(-1.01);
    joint_upper_limit.push_back(0.628);

    joint_lower_limit.push_back(-1.26);
    joint_upper_limit.push_back(1.26);
}

void MyPlugin::setup_arm_joint_limit() {

    joint_lower_limit.push_back(-2.26);
    joint_upper_limit.push_back( 2.26);

    joint_lower_limit.push_back(-0.188);
    joint_upper_limit.push_back(1.88);

    joint_lower_limit.push_back(-3.14);
    joint_upper_limit.push_back( 3.14);

    joint_lower_limit.push_back(-1.57);
    joint_upper_limit.push_back( 0.1);

    joint_lower_limit.push_back(-3.14);
    joint_upper_limit.push_back( 3.14);

    joint_lower_limit.push_back(-1.07);
    joint_upper_limit.push_back( 1.13);

    joint_lower_limit.push_back(-0.88);
    joint_upper_limit.push_back( 0.754);

    joint_lower_limit.push_back(-2.26);
    joint_upper_limit.push_back( 2.26);

    joint_lower_limit.push_back(-1.88);
    joint_upper_limit.push_back(0.188);

    joint_lower_limit.push_back(-3.14);
    joint_upper_limit.push_back( 3.14);

    joint_lower_limit.push_back(-1.57);
    joint_upper_limit.push_back( 0.1);

    joint_lower_limit.push_back(-3.14);
    joint_upper_limit.push_back( 3.14);
 
    joint_lower_limit.push_back(-1.13);
    joint_upper_limit.push_back(1.07);

    joint_lower_limit.push_back(-0.88);
    joint_upper_limit.push_back( 0.754);

}

void MyPlugin::setup_hand_joint_limit() {

    hand_joint_lower_limit.push_back( 0.0);
    hand_joint_upper_limit.push_back( 0.1);

    hand_joint_lower_limit.push_back( 0.0);
    hand_joint_upper_limit.push_back( 0.1);

    hand_joint_lower_limit.push_back( 0.0);
    hand_joint_upper_limit.push_back( 0.1);

    hand_joint_lower_limit.push_back( 0.0);
    hand_joint_upper_limit.push_back( 0.1);

    hand_joint_lower_limit.push_back( 0.0);
    hand_joint_upper_limit.push_back( 0.1);

}



void MyPlugin::subscribeJointCurrentPosition(const vrep_test::JointAngles& msg) {
  // Can be repalced by joint state publisher
  currentPosition[0] = msg.Knee;

  currentPosition[1] = msg.Back_Z;

  currentPosition[2] = msg.Back_X;

  currentPosition[3] = msg.Back_Y;

  currentPosition[4] = msg. Neck_Z;

  currentPosition[5] = msg. Neck_X;

  currentPosition[6] = msg. Head;

  currentPosition[7] = msg. Left_Shoulder_X;

  currentPosition[8] = msg. Left_Shoulder_Y;

  currentPosition[9] = msg. Left_Elbow_Z;

  currentPosition[10] = msg. Left_Elbow_X;

  currentPosition[11] = msg. Left_Wrist_Z;

  currentPosition[12] = msg. Left_Wrist_Y;

  currentPosition[13] = msg. Left_Wrist_X;

  currentPosition[14] = msg. Right_Shoulder_X;

  currentPosition[15] = msg. Right_Shoulder_Y;

  currentPosition[16] = msg. Right_Elbow_Z;

  currentPosition[17] = msg. Right_Elbow_X;

  currentPosition[18] = msg. Right_Wrist_Z;

  currentPosition[19] = msg. Right_Wrist_Y;

  currentPosition[20] = msg. Right_Wrist_X;


  // ptr_XR1->subscribeJointCurrentPosition(msg);
}




vrep_test::JointAngles MyPlugin::ConvertJointAnglesMsgs(std::vector<double> targetPosition) {

  // This is obviously a lot less efficient than just using joint state publisher, it only serves as a demonstration for
  // how to use custom ros messages for vrep communication
  vrep_test::JointAngles msg;

  msg.Knee = targetPosition[0];

  msg.Back_Z = targetPosition[1];

  msg.Back_X = targetPosition[2] ;

  msg.Back_Y = targetPosition[3];

  msg. Neck_Z = targetPosition[4];

  msg. Neck_X = targetPosition[5];

  msg. Head = targetPosition[6];

  msg. Left_Shoulder_X = targetPosition[7];

  msg. Left_Shoulder_Y = targetPosition[8];

  msg. Left_Elbow_Z = targetPosition[9];

  msg. Left_Elbow_X = targetPosition[10];

  msg. Left_Wrist_Z = targetPosition[11];

  msg. Left_Wrist_Y = targetPosition[12];

  msg. Left_Wrist_X = targetPosition[13];

  msg. Left_Shoulder_X = targetPosition[7];

  msg. Left_Shoulder_Y = targetPosition[8];

  msg. Left_Elbow_Z = targetPosition[9];

  msg. Left_Elbow_X = targetPosition[10];

  msg. Left_Wrist_Z = targetPosition[11];

  msg. Left_Wrist_Y = targetPosition[12];

  msg. Left_Wrist_X = targetPosition[13];

  msg. Right_Shoulder_X = targetPosition[14];

  msg. Right_Shoulder_Y = targetPosition[15];

  msg. Right_Elbow_Z = targetPosition[16];

  msg. Right_Elbow_X = targetPosition[17];

  msg. Right_Wrist_Z = targetPosition[18];

  msg. Right_Wrist_Y = targetPosition[19];

  msg. Right_Wrist_X = targetPosition[20];

  return msg;
}


vrep_test::IK_msg MyPlugin::ConvertIkMsgs(std::vector<double> IKtargetPosition) {

  vrep_test::IK_msg msg;
  msg.LPX = IKtargetPosition[0];

  msg.LPY = IKtargetPosition[1];

  msg.LPZ = IKtargetPosition[2] ;

  msg.LRX = IKtargetPosition[3];

  msg.LRY = IKtargetPosition[4];

  msg.LRZ = IKtargetPosition[5];

  msg.RPX = IKtargetPosition[6];

  msg.RPY = IKtargetPosition[7];

  msg.RPZ = IKtargetPosition[8];

  msg.RRX = IKtargetPosition[9];

  msg.RRY = IKtargetPosition[10];

  msg.RRZ = IKtargetPosition[11];

  return msg;
}
} // namespace


PLUGINLIB_DECLARE_CLASS(vrep_test, MyPlugin, vrep_test::MyPlugin, rqt_gui_cpp::Plugin)