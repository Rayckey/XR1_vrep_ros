#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "qxboxcontroller.h"
#include "xr1define.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"

using namespace Eigen;
namespace Ui {
class MainWindow;
}
class ActuatorController;
class QTimer;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void actuatorOperation(quint8 nId, quint8 nType);

    void quaterion2joint();
private slots:
    void on_launch_clicked();

    void on_ready_clicked();

    void on_start_clicked();

    void on_pause_clicked();

    void on_reset_clicked();

    void on_close_clicked();
    void playAction();
    void on_lineEdit_returnPressed();

    void on_lineEdit_2_returnPressed();

    void handleQXBoxControllerAxisEvent(QXBoxControllerAxisEvent *event);
    void handleQXBoxControllerButtonEvent(QXBoxControllerButtonEvent *event);

    void externalControllCallback();

    void on_Controller_Check_clicked();

    void on_pushButton_clicked();

    void quatimercallback();

    void quaternioncallback(uint8_t id , double w, double x, double y , double z );

    Vector3d quaternion2ZYX(double w, double qx, double qy , double qz );

    Vector3d quaternion2XYZ(double w, double x, double qy , double qz );

    void on_pushButton_2_clicked();

private:
    bool allActuatorHasLaunched()const;
    QVector<double> SpeedCal(double x,double y, double yaw);
private:
    Ui::MainWindow *ui;
    ActuatorController * m_pController;
    QTimer * m_pPlayTimer;
    QVector<QVector<double>> m_cmdValue;
    int m_nSchedule;
    std::vector<uint8_t> m_posArr;
    std::vector<uint8_t> m_velArr;
    int m_nVel;
    int m_nInterval;

    double X_Speed;
    double Y_Speed;
    double Z_CWSpeed;
    double Z_CCWSpeed;

    double speed_scale;

    QTimer * ExternalControlTimer;

    Quaterniond LS_init;
    Quaterniond LA_init;
    Quaterniond LH_init;
    Quaterniond Back_init;

    Quaterniond RS_init;
    Quaterniond RA_init;
    Quaterniond RH_init;
    Quaterniond Waist_init;

    Quaterniond L_I_init;
    Quaterniond L_T_init;
    Quaterniond L_R_init;
    Quaterniond L_P_init;
    Quaterniond L_M_init;

    Quaterniond LS_raw;
    Quaterniond LA_raw;
    Quaterniond LH_raw;
    Quaterniond Back_raw;

    Quaterniond RS_raw;
    Quaterniond RA_raw;
    Quaterniond RH_raw;
    Quaterniond Waist_raw;

    Quaterniond L_I_raw;
    Quaterniond L_T_raw;
    Quaterniond L_R_raw;
    Quaterniond L_P_raw;
    Quaterniond L_M_raw;

    Quaterniond LS_q;
    Quaterniond LA_q;
    Quaterniond LH_q;
    Quaterniond Back_q;

    Quaterniond RS_q;
    Quaterniond RA_q;
    Quaterniond RH_q;
    Quaterniond Waist_q;


    Quaterniond L_I_qq;
    Quaterniond L_T_qq;
    Quaterniond L_R_qq;
    Quaterniond L_P_qq;
    Quaterniond L_M_qq;

    double ShoulderX_q;
    double ShoulderY_q;
    double Elbow_Z_q;
    double Elbow_X_q;
    double    Wrist_Y_q;
    double    Wrist_X_q;
    double    Wrist_Z_q;

    double L_I_q;
    double L_T_q;
    double L_R_q;
    double L_P_q;
    double L_M_q;


};

#endif // MAINWINDOW_H
