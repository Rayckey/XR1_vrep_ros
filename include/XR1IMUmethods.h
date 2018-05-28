#ifndef XR1IMUMETHODS_H
#define XR1IMUMETHODS_H


#include "eigen3/Eigen/Dense"
#include "XR1IMUdefine.h"
#include <vector>

using namespace Eigen;
class XR1IMUmethods
{
public:
    XR1IMUmethods();

    std::vector<double> getJointAngles();

    void Initialize();

    void quaternioncallback(u_int8_t id , double w, double x, double y , double z );





private:

    void quaterion2joint();

    Vector3d quaternion2ZYX(double w, double qx, double qy , double qz );

    Vector3d quaternion2XYZ(double w, double x, double qy , double qz );

    Matrix3d EulerXYZ(double x , double y , double z ) ;

    Matrix3d EulerZYX(double x , double y , double z ) ;

    Vector3d Matrix2ZYX(Matrix3d input);

    Vector3d Matrix2YZX(Matrix3d input);

    Vector3d Vector2XZ(Vector3d v);

    Vector3d Vector2YX(Vector3d v);

    Vector3d FingerVector2YX(Vector3d v);

    double EasyFilter(double u, double v);

    std::vector<Quaterniond> Init_qs;

    std::vector<Quaterniond> Raw_qs;

    std::vector<Quaterniond> Cooked_qs;

    std::vector<Vector3d> Cooked_vs;

    Matrix3d LH_m;

    Matrix3d RH_m;

    std::vector<double> JointAngles;

    double PI;
};

#endif // XR1IMUMETHODS_H
