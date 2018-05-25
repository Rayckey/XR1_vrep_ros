#include "XR1IMUmethods.h"


using namespace Eigen;
using namespace XR1IMU;


XR1IMUmethods::XR1IMUmethods(){

    PI = 3.141592654;
    while (JointAngles.size() < Actuator_Total)
        JointAngles.push_back(0.0);


    while (Init_qs.size() < XR1IMU::IMU_total)
    {
        Init_qs.push_back(Quaterniond::Identity());
        Raw_qs.push_back(Quaterniond::Identity());
        Cooked_qs.push_back(Quaterniond::Identity());;
        Cooked_vs.push_back(Vector3d(0,1,0));
    }


    LH_m = MatrixXd::Identity(3,3);
    RH_m = MatrixXd::Identity(3,3);

}


void XR1IMUmethods::Initialize()
{
    for(int i = XR1IMU::Ground_IMU ; i < XR1IMU::IMU_total ; i++)
        Init_qs[i] = Raw_qs[i];

}



std::vector<double> XR1IMUmethods::getJointAngles(){

    quaterion2joint();

    return JointAngles;

}

void XR1IMUmethods::quaternioncallback(u_int8_t id , double w, double x , double y , double z) {
  //Some random intermediate values , becuase Eigen does not allow .inverse() on products;
  Quaterniond raw_q(w, x, y, z);

  Quaterniond temp_q;
  Quaterniond temp_temp_q;
  Quaterniond new_q;

  Matrix3d rot;

  //Unit Vector that Eigen doesn't have or I can't find for some reason
  static Vector3d unit_y(0, 1, 0);
  static Vector3d unit_x(1, 0, 0);

  switch (id) {

  case Back_IMU:
    Raw_qs[Back_IMU] = raw_q;
    Cooked_qs[Back_IMU] = Init_qs[Back_IMU].inverse() * Raw_qs[Back_IMU];
    break;


  case Left_Shoulder_IMU:

    Raw_qs[Left_Shoulder_IMU] = raw_q;

    temp_q = Init_qs[Back_IMU].inverse() * Init_qs[Left_Shoulder_IMU];
    Cooked_qs[Left_Shoulder_IMU] = Raw_qs[Back_IMU].inverse() * Raw_qs[Left_Shoulder_IMU] * temp_q.inverse();

    temp_q = Raw_qs[Back_IMU].inverse() * Raw_qs[Left_Shoulder_IMU];
    Cooked_vs[Left_Shoulder_IMU] = temp_q.toRotationMatrix() * unit_y;
    break;


  case Left_Arm_IMU:

      Raw_qs[Left_Arm_IMU] = raw_q;
    temp_q = Init_qs[Back_IMU].inverse() * Init_qs[Left_Arm_IMU];
    Cooked_qs[Left_Arm_IMU] = Cooked_qs[Left_Shoulder_IMU].inverse() * Raw_qs[Back_IMU].inverse() * Raw_qs[Left_Arm_IMU] * temp_q.inverse();


    rot = EulerXYZ(JointAngles[Left_Shoulder_X] , 0 , JointAngles[Left_Shoulder_Y]);

    temp_q = Raw_qs[Back_IMU].inverse() * Raw_qs[Left_Arm_IMU];
    Cooked_vs[Left_Arm_IMU] = temp_q.toRotationMatrix() * unit_y;
    Cooked_vs[Left_Arm_IMU] = rot.transpose() * Cooked_vs[Left_Arm_IMU];
    break;


  case Left_Hand_IMU:
      Raw_qs[Left_Hand_IMU] = raw_q;
    temp_temp_q = Init_qs[Back_IMU].inverse() * Init_qs[Left_Hand_IMU];
    rot = EulerXYZ(-JointAngles[Left_Shoulder_X] , 0 , JointAngles[Left_Shoulder_Y]) * EulerZYX(-JointAngles[Left_Elbow_X] , JointAngles[Left_Elbow_Z], 0);

    temp_q = Raw_qs[Back_IMU].inverse() * Raw_qs[Left_Hand_IMU] ;
    LH_m =  rot.transpose() * temp_q.toRotationMatrix() * temp_temp_q.inverse().toRotationMatrix();
    break;

  case Left_Thumb_IMU:

      Raw_qs[Left_Thumb_IMU] = raw_q;

    temp_q = Init_qs[Left_Hand_IMU].inverse() * Init_qs[Left_Thumb_IMU];

    new_q = temp_q.inverse() * Init_qs[Left_Hand_IMU].inverse() * Raw_qs[Left_Hand_IMU] * temp_q;
    Cooked_qs[Left_Thumb_IMU] = new_q.inverse() * Init_qs[Left_Thumb_IMU].inverse() * Raw_qs[Left_Thumb_IMU];

    Cooked_vs[Left_Thumb_IMU] = Cooked_qs[Left_Thumb_IMU].toRotationMatrix() * unit_x;
    break;

  case Left_Index_IMU:

    Raw_qs[Left_Index_IMU] = raw_q;


    temp_q = Raw_qs[Left_Hand_IMU].inverse() * Raw_qs[Left_Index_IMU];
    Cooked_vs[Left_Index_IMU] = temp_q.toRotationMatrix() * unit_x;
    break;

  case Left_Middle_IMU:

    Raw_qs[Left_Middle_IMU] = raw_q;

    temp_q = Raw_qs[Left_Hand_IMU].inverse() * Raw_qs[Left_Middle_IMU];
    Cooked_vs[Left_Middle_IMU] = temp_q.toRotationMatrix() * unit_x;
    break;

  case Left_Ring_IMU:

    Raw_qs[Left_Ring_IMU] = raw_q;

    temp_q = Raw_qs[Left_Hand_IMU].inverse() * Raw_qs[Left_Ring_IMU];
    Cooked_vs[Left_Ring_IMU] = temp_q.toRotationMatrix() * unit_x;
    break;

  case Left_Pinky_IMU:

    Raw_qs[Left_Pinky_IMU] = raw_q;

    temp_q = Raw_qs[Left_Hand_IMU].inverse() * Raw_qs[Left_Pinky_IMU];
    Cooked_vs[Left_Pinky_IMU] = temp_q.toRotationMatrix() * unit_x;

    break;

  case Right_Shoulder_IMU:
    Raw_qs[Right_Shoulder_IMU] = raw_q;

    temp_q =   Init_qs[Back_IMU].inverse() * Init_qs[Right_Shoulder_IMU];
    Cooked_qs[Right_Shoulder_IMU] = Raw_qs[Back_IMU].inverse() * Raw_qs[Right_Shoulder_IMU] * temp_q.inverse();


    temp_q = Raw_qs[Back_IMU].inverse() * Raw_qs[Right_Shoulder_IMU];
    Cooked_vs[Right_Shoulder_IMU] = temp_q.toRotationMatrix() * unit_y;
    break;

  case Right_Arm_IMU:
    Raw_qs[Right_Arm_IMU] = raw_q;

    temp_q = Init_qs[Back_IMU].inverse() * Init_qs[Right_Arm_IMU];
    Cooked_qs[Right_Arm_IMU] = Cooked_qs[Right_Shoulder_IMU].inverse() * Raw_qs[Back_IMU].inverse() * Raw_qs[Right_Arm_IMU] * temp_q.inverse();


    rot = EulerXYZ(JointAngles[Right_Shoulder_X] , 0 , JointAngles[Right_Shoulder_Y]);

    temp_q = Raw_qs[Back_IMU].inverse() * Raw_qs[Right_Arm_IMU];
    Cooked_vs[Right_Arm_IMU] = temp_q.toRotationMatrix() * unit_y;
    Cooked_vs[Right_Arm_IMU] = rot.transpose() * Cooked_vs[Right_Arm_IMU];
    break;

  case Right_Hand_IMU:
    Raw_qs[Right_Hand_IMU] = raw_q;

    temp_temp_q = Init_qs[Back_IMU].inverse() * Init_qs[Right_Hand_IMU];

    rot = EulerXYZ(-JointAngles[Right_Shoulder_X] , 0 , JointAngles[Right_Shoulder_Y]) * EulerZYX(-JointAngles[Right_Elbow_X] , JointAngles[Right_Elbow_Z], 0);

    temp_q = Raw_qs[Back_IMU].inverse() * Raw_qs[Right_Hand_IMU] ;
    RH_m =  rot.transpose() * temp_q.toRotationMatrix() * temp_temp_q.inverse().toRotationMatrix();
    break;


  case Right_Thumb_IMU:

    Raw_qs[Right_Thumb_IMU] = raw_q;

    temp_q = Init_qs[Right_Hand_IMU].inverse() * Init_qs[Right_Thumb_IMU];
    new_q = temp_q.inverse() * Init_qs[Right_Hand_IMU].inverse() * Raw_qs[Right_Hand_IMU] * temp_q;
    Cooked_qs[Right_Thumb_IMU] = new_q.inverse() * Init_qs[Right_Thumb_IMU].inverse() * Raw_qs[Right_Thumb_IMU];

    Cooked_vs[Right_Thumb_IMU] = Cooked_qs[Right_Thumb_IMU].toRotationMatrix() * unit_x;
    break;

  case Right_Index_IMU:

    Raw_qs[Right_Index_IMU] = raw_q;

    temp_q = Raw_qs[Right_Hand_IMU].inverse() * Raw_qs[Right_Index_IMU];
    Cooked_vs[Right_Index_IMU] = temp_q.toRotationMatrix() * unit_x;
    break;
  case Right_Middle_IMU:

    Raw_qs[Right_Middle_IMU] = raw_q;

    temp_q = Raw_qs[Right_Hand_IMU].inverse() * Raw_qs[Right_Middle_IMU];
    Cooked_vs[Right_Middle_IMU] = temp_q.toRotationMatrix() * unit_x;
    break;

  case Right_Ring_IMU:

    Raw_qs[Right_Ring_IMU] = raw_q;

    temp_q = Raw_qs[Right_Hand_IMU].inverse() * Raw_qs[Right_Ring_IMU];
    Cooked_vs[Right_Ring_IMU] = temp_q.toRotationMatrix() * unit_x;
    break;

  case Right_Pinky_IMU:

    Raw_qs[Right_Pinky_IMU] = raw_q;

    temp_q = Raw_qs[Right_Hand_IMU].inverse() * Raw_qs[Right_Pinky_IMU];
    Cooked_vs[Right_Pinky_IMU] = temp_q.toRotationMatrix() * unit_x;
    break;

  default:
    break;
  }
}



void XR1IMUmethods::quaterion2joint() {

    // All intermediate results are stored in this 3d vector
    Vector3d temp;
  temp = Vector2XZ(Cooked_vs[Right_Shoulder_IMU]);

  JointAngles[Right_Shoulder_X] = EasyFilter(JointAngles[Right_Shoulder_X] , temp(0));
  if(JointAngles[Right_Shoulder_X] < -2.9) JointAngles[Right_Shoulder_X] = JointAngles[Right_Shoulder_X] + 2.0 * PI;
  JointAngles[Right_Shoulder_Y] = EasyFilter(JointAngles[Right_Shoulder_Y] , temp(2));


  temp = Vector2YX(Cooked_vs[Right_Arm_IMU]);
  JointAngles[Right_Elbow_Z] = EasyFilter(JointAngles[Right_Elbow_Z] , temp(1));
  JointAngles[Right_Elbow_X] = EasyFilter(JointAngles[Right_Elbow_X] , temp(0));


  temp = Matrix2YZX( RH_m);
  JointAngles[Right_Wrist_Z] = EasyFilter(JointAngles[Right_Wrist_Z] , temp(1));
  JointAngles[Right_Wrist_Y] = EasyFilter(JointAngles[Right_Wrist_Y] , temp(2));
  JointAngles[Right_Wrist_X] = EasyFilter(JointAngles[Right_Wrist_X] , temp(0));

  temp = FingerVector2YX( Cooked_vs[Right_Thumb_IMU]);
  JointAngles[Right_Thumb] = EasyFilter(JointAngles[Right_Thumb] , temp(1));


  temp = FingerVector2YX(Cooked_vs[Right_Index_IMU]);
  JointAngles[Right_Index] = EasyFilter(JointAngles[Right_Index] , temp(1));


  temp = FingerVector2YX( Cooked_vs[Right_Middle_IMU]);
  JointAngles[Right_Middle] = EasyFilter(JointAngles[Right_Middle] , temp(1));

  temp = FingerVector2YX( Cooked_vs[Right_Ring_IMU]);
  JointAngles[Right_Ring] = EasyFilter(JointAngles[Right_Ring] , temp(1));

  temp = FingerVector2YX( Cooked_vs[Right_Pinky_IMU]);
  JointAngles[Right_Pinky] = EasyFilter(JointAngles[Right_Pinky] , temp(1));

  temp = Vector2XZ(Cooked_vs[Left_Shoulder_IMU]);
  JointAngles[Left_Shoulder_X] = EasyFilter(JointAngles[Left_Shoulder_X] , temp(0));
    if(JointAngles[Left_Shoulder_X] < -2.9) JointAngles[Left_Shoulder_X] = JointAngles[Left_Shoulder_X] + 2.0 * PI;
  JointAngles[Left_Shoulder_Y] = EasyFilter(JointAngles[Left_Shoulder_Y] , temp(2));

  temp = Vector2YX(Cooked_vs[Left_Arm_IMU]);

  JointAngles[Left_Elbow_Z] = EasyFilter(JointAngles[Left_Elbow_Z] , temp(1));
  JointAngles[Left_Elbow_X] = EasyFilter(JointAngles[Left_Elbow_X] , temp(0));

  temp = Matrix2YZX( LH_m);

  JointAngles[Left_Wrist_Z] = EasyFilter(JointAngles[Left_Wrist_Z], temp(1));
  JointAngles[Left_Wrist_Y] = EasyFilter(JointAngles[Left_Wrist_Y], temp(2));
  JointAngles[Left_Wrist_X] = EasyFilter(JointAngles[Left_Wrist_X], temp(0));

  temp = FingerVector2YX( Cooked_vs[Left_Thumb_IMU]);
  JointAngles[Left_Thumb] = EasyFilter(JointAngles[Left_Thumb] , temp(1));

  temp = FingerVector2YX( Cooked_vs[Left_Index_IMU]);
  JointAngles[Left_Index] = EasyFilter(JointAngles[Left_Index] , temp(1));

  temp = FingerVector2YX( Cooked_vs[Left_Middle_IMU]);
  JointAngles[Left_Middle] = EasyFilter(JointAngles[Left_Middle] , temp(1));

  temp = FingerVector2YX( Cooked_vs[Left_Ring_IMU]);
  JointAngles[Left_Ring] = EasyFilter(JointAngles[Left_Ring] , temp(1));

  temp = FingerVector2YX( Cooked_vs[Left_Pinky_IMU]);
  JointAngles[Left_Pinky] = EasyFilter(JointAngles[Left_Pinky] , temp(1));

}

Eigen::Vector3d XR1IMUmethods::quaternion2ZYX(double qw, double qx, double qy, double qz) {

  double aSinInput = -2 * (qx * qz - qw * qy);
  if (aSinInput > 1) aSinInput = 1;
  if (aSinInput < -1) aSinInput = -1;

  Eigen::Vector3d res(atan2( 2 * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz ),
                      asin( aSinInput ),
                      atan2( 2 * (qy * qz + qw * qx), qw * qw - qx * qx - qy * qy + qz * qz ));

  return res;
}


Eigen::Vector3d XR1IMUmethods::quaternion2XYZ(double qw, double qx, double qy, double qz) {

  double aSinInput = 2 * (qx * qz + qy * qw);
  if (aSinInput > 1) aSinInput = 1;
  if (aSinInput < -1) aSinInput = -1;

  Eigen::Vector3d res( atan2( -2 * (qy * qz - qx * qw), qw * qw - qx * qx - qy * qy + qz * qz ),
                       asin( aSinInput ),
                       atan2( -2 * (qx * qy - qz * qw), qw * qw + qx * qx - qy * qy - qz * qz ));

  return res;
}

Eigen::Vector3d XR1IMUmethods::Matrix2ZYX(Eigen::Matrix3d input) {

  Eigen::Vector3d res( atan2( input(2, 1) , input(2, 2)),
                       asin(  input(1, 0)),
                       atan2( input(1, 0) , input(0, 0)));

  return res;
}


Eigen::Vector3d XR1IMUmethods::Matrix2YZX(Eigen::Matrix3d input) {

  Eigen::Vector3d res( atan2( -input(1, 2) , input(1, 1)),
                       atan2( -input(2, 0) , input(0, 0)),
                       asin(  input(1, 0)));

  return res;
}


Eigen::Matrix3d XR1IMUmethods::EulerXYZ(double x , double y , double z ) {
  Eigen::Matrix3d res;
  ;
  res <<              cos(y)*cos(z),                       -cos(y)*sin(z),         sin(y),
      cos(x)*sin(z) + cos(z)*sin(x)*sin(y), cos(x)*cos(z) - sin(x)*sin(y)*sin(z), -cos(y)*sin(x),
      sin(x)*sin(z) - cos(x)*cos(z)*sin(y), cos(z)*sin(x) + cos(x)*sin(y)*sin(z),  cos(x)*cos(y);

  return res;
}



Eigen::Matrix3d XR1IMUmethods::EulerZYX(double x , double y , double z ) {
  Eigen::Matrix3d res;
  res << cos(y)*cos(z), cos(z)*sin(x)*sin(y) - cos(x)*sin(z), sin(x)*sin(z) + cos(x)*cos(z)*sin(y),
      cos(y)*sin(z), cos(x)*cos(z) + sin(x)*sin(y)*sin(z), cos(x)*sin(y)*sin(z) - cos(z)*sin(x),
      -sin(y),                        cos(y)*sin(x),                        cos(x)*cos(y);
  return res;
}



Eigen::Vector3d XR1IMUmethods::Vector2XZ(Eigen::Vector3d v) {
  Eigen::Vector3d res = Eigen::VectorXd::Zero(3);

  res(0) = -atan2(v(2) , v(1));


  res(2) = - asin(v(0) / v.norm());

  if (abs(abs(res(2)) - PI / 2.0) < 0.03)
    if (abs(v(2)) < 0.03 && abs(v(1) < 0.03))
      res(0) = 0;

  return res;

}


Eigen::Vector3d XR1IMUmethods::Vector2YX(Eigen::Vector3d v) {
  Eigen::Vector3d res = Eigen::VectorXd::Zero(3);

  res(1) = atan2(v(0) , v(2));

  res(0) = asin(v(1) / v.norm()) - PI / 2;

  if (abs(res(0) ) < 0.03) res(1) = 0;
  return res;
}



Eigen::Vector3d XR1IMUmethods::FingerVector2YX(Eigen::Vector3d v) {
  Eigen::Vector3d res = Eigen::VectorXd::Zero(3);

  double v2 = (v(2) > 0) ? v(2) : 0.0;

  res(1) = atan2(v(2) , v(0));

  res(0) = asin(v(1) / v.norm()) - PI / 2;

  if (res(1) < -1.5) res(1) = 3.0;

  return res;
}

double XR1IMUmethods::EasyFilter(double u, double v) {

  static double filter_ratio = 0.5;
  static double filter_error = 1.0 - filter_ratio;

  return  u * filter_ratio + v * filter_error;
}
