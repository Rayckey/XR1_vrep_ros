#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFile>
#include <QDebug>
#include "actuatorcontroller.h"

using namespace std;
using namespace Eigen;

double nScale[31] = {120,-80,120,-80,36,36,36,70.56,70.56,-36,36,-36,1,1,-70.56,70.56,-36,-36,-36,1,1,1,1,1,1,1,1,1,1,1,1};
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_pController(nullptr),
    m_pPlayTimer(nullptr),
    m_nSchedule(0),
    m_nVel(1600),
    m_nInterval(10)
  ,X_Speed(0)
  ,Y_Speed(0)
  ,Z_CWSpeed(0)
  ,Z_CCWSpeed(0)
  ,ExternalControlTimer(NULL)
  ,Back_init(Quaterniond::Identity())
  ,LS_init(Quaterniond::Identity())
  ,LA_init(Quaterniond::Identity())
{
    ui->setupUi(this);
    //ui->close->setEnabled(false);
    ui->launch->setEnabled(false);
    ui->pause->setEnabled(false);
    ui->ready->setEnabled(false);
    ui->reset->setEnabled(false);
    ui->start->setEnabled(false);
    QFile file("/home/innfos/WORK/XR1Actions/xr1rules.data");
    if(file.open(QFile::ReadOnly | QFile::Text))
    {
        while (!file.atEnd()) {
            QByteArray arr = file.readLine();
            if(/*arr[arr.size()-1] == '\n'*/true)
                arr = arr.left(arr.size()-1);
            //arr.replace('\n','0');
            QList<QByteArray> arrList = arr.split(',');
            if(arrList.size() < 11)
            {
                qDebug() << "Data error";
                continue;
            }

            QVector<double> cmdValue;
            foreach (QByteArray tmp, arrList) {
                cmdValue.push_back(tmp.toDouble());
            }
            m_cmdValue.push_back(cmdValue);
        }
    }




//    if (GeneratedConfiguration.size()) {
//      QFileDialog dialog(0, tr("Save Action"), QDir::currentPath());
//      dialog.setFileMode(QFileDialog::AnyFile);
//      dialog.setAcceptMode(QFileDialog::AcceptSave);
//      dialog.setNameFilter(tr("ActuatorData(*.data)"));

//      if (dialog.exec() == QDialog::Accepted)
//      {
//        QString path = dialog.selectedFiles().first();
//        if (path.size() > 0)
//        {

//          if (!path.endsWith(".data"))
//          {
//            path += ".data";
//          }
//          QFile file(path);

//          QFile filewrite("/home/innfos/WORK/XR1Actions/xr1rulesslowlyreread.data");
//          if (filewrite.open(QFile::WriteOnly | QFile::Text | QFile::Truncate))
//          {
//            QTextStream out(&filewrite);

//            for (int i = 0 ; i < m_cmdValue.size() ; i++) {
//              for (int j = 0 ; j < m_cmdValue[0].size() - 1; j++) {
//                out << m_cmdValue[i][j] << ",";
//              }
//              out << m_cmdValue[i][m_cmdValue[0].size() - 1];
//              out <<  endl;
//            }
//          }
//          filewrite.close();
//        }
//    }

//    }




    qDebug() << "Data size" << m_cmdValue.size() << " " << m_cmdValue[0].size();

    m_pController = ActuatorController::getInstance();

    m_pController->m_sOperationFinished.connect_member(this,&MainWindow::actuatorOperation);
    m_pController->m_sActuatorAttrChanged.s_Connect([=](uint8_t id,uint8_t attr,double value){
        switch (attr) {
        case Actuator::ACTUAL_POSITION:
        {
            QLabel * pLabel = findChild<QLabel*>(tr("p%1").arg(id));
            if(pLabel)
            {
                QRegExp rx;
                rx.setPattern("(\\.){0,1}0+$");
                pLabel->setText(QString::number(value,'f',6).replace(rx,""));
            }

        }

            break;
        default:
            break;
        }
    });

    m_pPlayTimer = new QTimer(this);
    connect(m_pPlayTimer,&QTimer::timeout,this,&MainWindow::playAction);

//    m_posArr.push_back(4);
//    m_posArr.push_back(5);
//    m_posArr.push_back(6);
//    m_posArr.push_back(7);
//    m_posArr.push_back(8);
//    m_posArr.push_back(9);
//    m_posArr.push_back(10);
//    m_posArr.push_back(11);

    for (int i = XR1::Knee_X ; i < XR1::Actuator_Total ; i++)
        m_posArr.push_back(i);

    m_velArr.push_back(XR1::Left_Front_Wheel);
    m_velArr.push_back(XR1::Right_Front_Wheel);
    m_velArr.push_back(XR1::Back_Wheel);


    QXBoxController * ptr_XBox;

    // In case you have a lot of controllers
    // There you go
    for (int i = 0; i < 10; i++)
    {
        ptr_XBox = new QXBoxController(i, this);
        if (ptr_XBox->isValid())
        {
            connect(ptr_XBox, SIGNAL(XBoxControllerAxisEvent(QXBoxControllerAxisEvent*)), this, SLOT(handleQXBoxControllerAxisEvent(QXBoxControllerAxisEvent*)));
            connect(ptr_XBox, SIGNAL(XBoxControllerButtonEvent(QXBoxControllerButtonEvent*)), this, SLOT(handleQXBoxControllerButtonEvent(QXBoxControllerButtonEvent*)));

            QTimer *timer = new QTimer(this);
            timer->setInterval(10);

            connect(timer, SIGNAL(timeout()), ptr_XBox, SLOT(readXBoxController()));
            timer->start();
        }
        else
        {
            delete ptr_XBox;
            break;
        }
    }

    speed_scale = 1.0;


    QTimer * QuaTimer = new QTimer;

    connect(QuaTimer , &QTimer::timeout , this , &MainWindow::quatimercallback);
    QuaTimer->start(50);

    m_pController->m_sQuaternion.connect_member(this , &MainWindow::quaternioncallback);



    QTimer * ConvertTimer = new QTimer;

    connect(ConvertTimer , &QTimer::timeout , this , &MainWindow::quaterion2joint);
    ConvertTimer->start(1000);


}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::actuatorOperation(quint8 nId, quint8 nType)
{
    switch (nType) {
    case Actuator::Recognize_Finished:
        if(m_pController->hasAvailableActuator())
            ui->launch->setEnabled(true);
        break;
    case Actuator::Launch_Finished:
        if(allActuatorHasLaunched())
        {
            ui->ready->setEnabled(true);
            m_pController->switchAutoRefresh(nId,true);
        }

        break;
    default:
        break;
    }
}

void MainWindow::on_launch_clicked()
{
    m_pController->launchAllActuators();
    if(allActuatorHasLaunched())
        ui->ready->setEnabled(true);
}

void MainWindow::on_ready_clicked()
{

    m_pController->activeActuatorMode(m_posArr,Actuator::Mode_Profile_Pos);
    m_pController->activeActuatorMode(m_velArr,Actuator::Mode_Profile_Vel);

    ui->close->setEnabled(true);
    ui->pause->setEnabled(true);
    ui->reset->setEnabled(true);
    ui->start->setEnabled(true);
    vector<uint8_t> idArray = m_pController->getActuatorIdArray();
    for(int i=0;i<idArray.size();++i)
    {
        m_pController->switchAutoRefresh(idArray.at(i),false);
    }
}

void MainWindow::on_start_clicked()
{
    if(m_pPlayTimer)
        m_pPlayTimer->start(m_nInterval);
}

void MainWindow::on_pause_clicked()
{
    m_pPlayTimer->stop();
    for(int i=0;i<m_velArr.size();++i)
    {
        m_pController->setVelocity(m_velArr.at(i),0);
    }
}

void MainWindow::on_reset_clicked()
{
    m_nSchedule = 0;

    for(int i=0;i<m_posArr.size();++i)
    {
        m_pController->setPosition(m_posArr.at(i),0);
    }

    for(int i=0;i<m_velArr.size();++i)
    {
        m_pController->setVelocity(m_velArr.at(i),0);
    }
}

void MainWindow::on_close_clicked()
{
    on_pause_clicked();
    m_pController->closeAllActuators();
    ui->close->setEnabled(false);
    ui->pause->setEnabled(false);
    ui->ready->setEnabled(false);
    ui->reset->setEnabled(false);
    ui->start->setEnabled(false);
}

void MainWindow::playAction()
{

    if(m_nSchedule < m_cmdValue.size())
    {
        QVector<double> cmdValue = m_cmdValue.at(m_nSchedule);
        ++m_nSchedule;
        //        qDebug() << "At Step " << m_nSchedule;

        for(int i=0;i<m_posArr.size();++i)
        {
            m_pController->setPosition(m_posArr.at(i),cmdValue.at(i)*nScale[i]/2/3.14);
            qDebug() << " Setting " <<m_posArr.at(i) <<" position to " << cmdValue.at(i);
        }


//        m_pController->setPosition(m_posArr.at(4),cmdValue.at(7)*nScale[4]/2/3.14);
//        m_pController->setPosition(m_posArr.at(5),cmdValue.at(8)*nScale[5]/2/3.14);
//        m_pController->setPosition(m_posArr.at(6),cmdValue.at(14)*nScale[6]/2/3.14);
//        m_pController->setPosition(m_posArr.at(7),cmdValue.at(15)*nScale[7]/2/3.14);

        //        qDebug() << " Setting " <<m_posArr.at(4) <<" position to " << cmdValue.at(7);
        //        qDebug() << " Setting " <<m_posArr.at(5) <<" position to " << cmdValue.at(8);
        //        qDebug() << " Setting " <<m_posArr.at(6) <<" position to " << cmdValue.at(14);
        //        qDebug() << " Setting " <<m_posArr.at(7) <<" position to " << cmdValue.at(15);

        //        qDebug() << " Read Speed X" <<cmdValue.at(cmdValue.size()-3) ;
        //        qDebug() << " Read Speed Y" <<cmdValue.at(cmdValue.size()-2) ;
        //        qDebug() << " Read Speed Z " <<cmdValue.at(cmdValue.size()-1) ;


//        if(!(ui->Controller_Check->isChecked())){
//            QVector<double> velValue = SpeedCal(-cmdValue.at(cmdValue.size()-2),-cmdValue.at(cmdValue.size()-3),cmdValue.at(cmdValue.size()-1));

//            //        if (abs(cmdValue.at(cmdValue.size()-1) - pre_velValue) > 0.0034)
//            //             qDebug() << "The World ends At Step " << m_nSchedule;
//            for(int i=0;i<3;++i)
//            {
//                double value = velValue.at(i)*m_nVel;
//                m_pController->setVelocity(m_velArr.at(i),value);
//            }

//            qDebug() << " Setting Vel to " << velValue.at(0)*m_nVel <<" " << velValue.at(1)*m_nVel << " " << velValue.at(2)*m_nVel;
//        }

    }
    else
    {
        //m_pPlayTimer->stop();
        m_nSchedule = 0;
    }
}

bool MainWindow::allActuatorHasLaunched()const
{
    vector<uint8_t> idArray = m_pController->getActuatorIdArray();
    for(int i=0;i<idArray.size();++i)
    {
        if(m_pController->getActuatorAttribute(idArray.at(i),Actuator::ACTUATOR_SWITCH) == Actuator::ACTUATOR_SWITCH_OFF)
            return false;
    }
    return true;
}

QVector<double> MainWindow::SpeedCal(double x, double y, double yaw)
{

    double F1, F2 ,F3;
    F1 = 0.0;
    F2 = 0.0;
    F3 = 0.0;


    double Speedo = sqrt(x*x + y*y);

    if( abs(x) > 0.0000001 || abs(y) > 0.0000001){

        double angle = atan2(y,x);

        F1 = Speedo * cos(2.618 - angle) ;
        F2 = Speedo * cos(0.5236 - angle);
        F3 = Speedo * cos(4.7124 - angle);
    }

    F1 += yaw;
    F2 += yaw;
    F3 += yaw;

    QVector<double> arr;
    arr.push_back(F1);
    arr.push_back(F2);
    arr.push_back(F3);

    return arr;

}

void MainWindow::on_lineEdit_returnPressed()
{
    QLineEdit * pEdit = qobject_cast<QLineEdit*>(sender());
    int nVel = pEdit->text().toUInt();
    if(nVel > 0)
    {
        m_nVel = nVel;
    }
}

void MainWindow::on_lineEdit_2_returnPressed()
{
    QLineEdit * pEdit = qobject_cast<QLineEdit*>(sender());
    int nVel = pEdit->text().toUInt();
    if(nVel > 0)
    {
        m_nInterval = nVel;
    }
}


void MainWindow::handleQXBoxControllerAxisEvent(QXBoxControllerAxisEvent *event){

    uint axis = event->axis();

    if(axis == 2){
        Y_Speed = -event->value()*speed_scale;
    }

    else if(axis == 3){
        X_Speed = -event->value()*speed_scale;
    }

    if(axis == 4){
        Z_CWSpeed = event->value()*speed_scale + 1.0;
    }

    else if(axis == 5){
        Z_CCWSpeed = event->value()*speed_scale + 1.0;
    }

    delete event;   //QGameControllerEvents unlike QEvents are not deleted automatically.

}

void MainWindow::handleQXBoxControllerButtonEvent(QXBoxControllerButtonEvent *event){
    delete event;
}


void MainWindow::externalControllCallback(){

    double F1, F2 ,F3;
    F1 = 0.0;
    F2 = 0.0;
    F3 = 0.0;


    double Speedo = sqrt(X_Speed*X_Speed + Y_Speed*Y_Speed);

    if( abs(X_Speed) > 0.0000001 || abs(Y_Speed) > 0.0000001){

        double angle = atan2(Y_Speed,X_Speed);

        F1 = Speedo * cos(2.618 - angle) ;
        F2 = Speedo * cos(0.5236 - angle) ;
        F3 = Speedo * cos(4.7124 - angle) ;
    }

    F1 += Z_CCWSpeed - Z_CWSpeed;
    F2 += Z_CCWSpeed - Z_CWSpeed;
    F3 += Z_CCWSpeed - Z_CWSpeed;

    F1 = F1 * m_nVel;
    F2 = F2 * m_nVel;
    F3 = F3 * m_nVel;


    qDebug() << "Setting Velocity 1 as " << F1;
    qDebug() << "Setting Velocity 2 as " << F2;
    qDebug() << "Setting Velocity 3 as " << F3;


    m_pController->setVelocity(m_velArr.at(0),F1);
    m_pController->setVelocity(m_velArr.at(1),F2);
    m_pController->setVelocity(m_velArr.at(2),F3);

}

void MainWindow::on_Controller_Check_clicked()
{
    if(!ExternalControlTimer){
        ExternalControlTimer = new QTimer;
        connect(ExternalControlTimer , &QTimer::timeout , this , &MainWindow::externalControllCallback);
    }

    else ExternalControlTimer->stop();

    if (ui->Controller_Check->isChecked()) ExternalControlTimer->start(10);
}

void MainWindow::on_pushButton_clicked()
{
    Back_init = Back_raw;
    LS_init = LS_raw;
    LA_init = LA_raw;
    LH_init = LH_raw;

   L_I_init = L_I_raw ;
   L_T_init = L_T_raw ;
   L_R_init = L_R_raw ;
   L_P_init = L_P_raw ;
   L_M_init = L_M_raw ;

}

void MainWindow::quatimercallback()
{
    m_pController->requestAllQuaternions();
}


void MainWindow::quaternioncallback(uint8_t id ,double w,double x ,double y ,double z){

//    qDebug() << id << " " << x << " " << y << " " << " " << z <<  " " << w << endl;

    Quaterniond temp_q;
    Quaterniond temp_temp_q;
    Quaterniond new_q;
    Quaterniond new_new_q;

    switch (id) {
    case 1:
        Back_raw.w() = w;
        Back_raw.x() = x;
        Back_raw.y() = y;
        Back_raw.z() = z;
        Back_q = Back_init.inverse() * Back_raw ;
        break;

    case 2:
        LS_raw.w() = w;
        LS_raw.x() = x;
        LS_raw.y() = y;
        LS_raw.z() = z;



//        temp_q = Back_init.inverse() * LS_init;
//        new_q = temp_q.inverse() * Back_q * temp_q;
//        LS_q = new_q.inverse() * LS_init.inverse() * LS_raw;


        temp_q =   Back_init.inverse() * LS_init;
//        LS_q = temp_q.inverse() * Back_q.inverse() * temp_q * LS_init.inverse() * LS_raw;
        LS_q = Back_raw.inverse() * LS_raw * temp_q.inverse();
//        LS_q.vec() = -temp_temp_q.vec();
//        LS_q.w() = temp_temp_q.w();


    case 3:
        LA_raw.w() = w;
        LA_raw.x() = x;
        LA_raw.y() = y;
        LA_raw.z() = z;

//        temp_q = LS_init.inverse() * LA_init;
//        new_q = temp_q.inverse() * LS_init.inverse() * LS_raw * temp_q;
//        LA_q = new_q.inverse() * LA_init.inverse() * LA_raw;


        temp_q = Back_init.inverse() * LA_init;
        LA_q = LS_q.inverse() * Back_raw.inverse() * LA_raw * temp_q.inverse();


    case 7:
        LH_raw.w() = w;
        LH_raw.x() = x;
        LH_raw.y() = y;
        LH_raw.z() = z;

        temp_q = Back_init.inverse() * LH_init;
        LH_q = LA_q.inverse() * Back_raw.inverse() * LH_raw * temp_q.inverse();


    case 9:

        L_T_raw.w() = w;
        L_T_raw.x() = x;
        L_T_raw.y() = y;
        L_T_raw.z() = z;

        temp_q = LH_init.inverse() * L_T_init;
        L_T_qq = LH_raw.inverse() * L_T_raw * temp_q.inverse();

    case 11:

        L_I_raw.w() = w;
        L_I_raw.x() = x;
        L_I_raw.y() = y;
        L_I_raw.z() = z;

        temp_q = LH_init.inverse() * L_I_init;
        L_I_qq = LH_raw.inverse() * L_I_raw * temp_q.inverse();

    case 13:

        L_M_raw.w() = w;
        L_M_raw.x() = x;
        L_M_raw.y() = y;
        L_M_raw.z() = z;

        temp_q = LH_init.inverse() * L_M_init;
        L_M_qq = LH_raw.inverse() * L_M_raw * temp_q.inverse();

    case 15:

        L_R_raw.w() = w;
        L_R_raw.x() = x;
        L_R_raw.y() = y;
        L_R_raw.z() = z;

        temp_q = LH_init.inverse() * L_R_init;
        L_R_qq = LH_raw.inverse() * L_R_raw * temp_q.inverse();


    case 17:

        L_P_raw.w() = w;
        L_P_raw.x() = x;
        L_P_raw.y() = y;
        L_P_raw.z() = z;

        temp_q = LH_init.inverse() * L_P_init;
        L_P_qq = LH_raw.inverse() * L_P_raw * temp_q.inverse();


    case 4:
        RS_raw.w() = w;
        RS_raw.x() = x;
        RS_raw.y() = y;
        RS_raw.z() = z;

        temp_q =   Back_init.inverse() * RS_init;
        RS_q = Back_raw.inverse() * RS_raw * temp_q.inverse();


    case 5:
        RA_raw.w() = w;
        RA_raw.x() = x;
        RA_raw.y() = y;
        RA_raw.z() = z;

        temp_q = Back_init.inverse() * RA_init;
        RA_q = RS_q.inverse() * Back_raw.inverse() * RA_raw * temp_q.inverse();

    case 8:
        RH_raw.w() = w;
        RH_raw.x() = x;
        RH_raw.y() = y;
        RH_raw.z() = z;

        temp_q = Back_init.inverse() * RH_init;
        RH_q = RA_q.inverse() * Back_raw.inverse() * RH_raw * temp_q.inverse();

    default:
        break;
    }
}



void MainWindow::quaterion2joint(){

//    qDebug() << "Base : " << Back_q.w() << " " << Back_q.x() << " " << Back_q.y() << " " << Back_q.z() <<  endl;

    Vector3d temp = quaternion2ZYX( Back_q.w(), Back_q.x(), Back_q.y(), Back_q.z());
    qDebug() << " The Back X is " << temp(2);
    qDebug() << " The Back Z is " << temp(0);


     temp = quaternion2XYZ( LS_q.w(), LS_q.x(), LS_q.y(), LS_q.z());

    ShoulderX_q = temp(0);
    ShoulderY_q = temp(2);

    qDebug() << " The ShoulderX_q is " << ShoulderX_q;
    qDebug() << " The ShoulderY_q is " << ShoulderY_q;

     temp = quaternion2ZYX( LA_q.w(), LA_q.x(), LA_q.y(), LA_q.z());

    Elbow_Z_q = temp(1);
    Elbow_X_q = temp(2);

    qDebug() << " The Elbow_Z_q is " << Elbow_Z_q;
    qDebug() << " The Elbow_X_q is " << Elbow_X_q;

    temp = quaternion2ZYX( LH_q.w(), LH_q.x(), LH_q.y(), LH_q.z());

   Wrist_Z_q = temp(1);
   Wrist_Y_q = temp(0);
   Wrist_X_q = temp(2);

    qDebug() << " The Wrist_Z_q is " << Wrist_Z_q;
    qDebug() << " The Wrist_Y_q is " << Wrist_Y_q;
    qDebug() << " The Wrist_X_q is " << Wrist_X_q;



    temp = quaternion2ZYX( LH_q.w(), LH_q.x(), LH_q.y(), LH_q.z());

   Wrist_Z_q = temp(1);
   Wrist_Y_q = temp(0);
   Wrist_X_q = temp(2);

    qDebug() << " The Wrist_Z_q is " << Wrist_Z_q;
    qDebug() << " The Wrist_Y_q is " << Wrist_Y_q;
    qDebug() << " The Wrist_X_q is " << Wrist_X_q;



    temp = quaternion2XYZ( L_T_qq.w(), L_T_qq.x(), L_T_qq.y(), L_T_qq.z());

    L_T_q = temp(1);

    qDebug() << " The Left_index is " << L_T_q;

    temp = quaternion2XYZ( L_I_qq.w(), L_I_qq.x(), L_I_qq.y(), L_I_qq.z());

    L_I_q = temp(0);

    qDebug() << " The Left_index is " << L_I_q;

    temp = quaternion2XYZ( L_M_qq.w(), L_M_qq.x(), L_M_qq.y(), L_M_qq.z());

    L_M_q = temp(0);

    qDebug() << " The Left_index is " << L_M_q;


    temp = quaternion2XYZ( L_R_qq.w(), L_R_qq.x(), L_R_qq.y(), L_R_qq.z());

    L_R_q = temp(0);

    qDebug() << " The Left_index is " << L_R_q;

    temp = quaternion2XYZ( L_P_qq.w(), L_P_qq.x(), L_P_qq.y(), L_P_qq.z());

    L_P_q = temp(0);

    qDebug() << " The Left_index is " << L_P_q;




    Quaterniond temp_p = Back_init.inverse() * Back_raw;
     temp = quaternion2ZYX( temp_p.w(), temp_p.x(), temp_p.y(), temp_p.z());
    qDebug() << " The 01 Angles are " << temp_p.w() << " " << temp_p.x() << " " << temp_p.y() << " " << temp_p.z();



     temp_p = LS_init.inverse() * LS_raw;
     temp = quaternion2ZYX( temp_p.w(), temp_p.x(), temp_p.y(), temp_p.z());
    qDebug() << " The 02 Angles are " <<  LS_q.w() << " " << LS_q.x() << " " << LS_q.y() << " " << LS_q.z();

     temp_p = LA_init.inverse() * LA_raw;
     temp = quaternion2ZYX( temp_p.w(), temp_p.x(), temp_p.y(), temp_p.z());
//    qDebug() << " The 03 Angles are " << temp(0) << " " << temp(1) << " " << temp(2);
     qDebug() << " The 03 Angles are " << LA_q.w() << " " << LA_q.x() << " " << LA_q.y() << " " << LA_q.z();
}


Vector3d MainWindow::quaternion2ZYX(double qw, double qx, double qy, double qz){

    double aSinInput = -2*(qx*qz-qw*qy);
    if (aSinInput > 1) aSinInput = 1;
    if (aSinInput < -1) aSinInput= -1;

    Vector3d res(atan2( 2*(qx*qy+qw*qz), qw*qw + qx*qx - qy*qy - qz*qz ),
                 asin( aSinInput ),
                 atan2( 2*(qy*qz+qw*qx), qw*qw - qx*qx - qy*qy + qz*qz ));

    return res;
}


Vector3d MainWindow::quaternion2XYZ(double qw, double qx, double qy, double qz){

    double aSinInput = 2*(qx*qz + qy*qw);
    if (aSinInput > 1) aSinInput = 1;
    if (aSinInput < -1) aSinInput= -1;

    Vector3d res( atan2( -2*(qy*qz - qx*qw), qw*qw - qx*qx - qy*qy + qz*qz ),
        asin( aSinInput ),
        atan2( -2*(qx*qy - qz*qw), qw*qw + qx*qx - qy*qy - qz*qz ));

    return res;
}

void MainWindow::on_pushButton_2_clicked()
{

    vector<uint8_t> temp;


    for (int i = XR1::Knee_X ; i < XR1::Actuator_Total ; i++){
        temp.push_back(i);
    }


    m_pController->activeActuatorMode(temp, Actuator::Mode_Pos);
}
