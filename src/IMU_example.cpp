#include "my_plugin.h"




#define PI 3.141592654
namespace vrep_test {


void MyPlugin::gettingIMUStarted() {



  ROS_INFO("Calling IMU Library");
  //Getting the IMU library ready
  XR1IMUptr = new XR1IMUmethods();



  ROS_INFO("Setting IMU Controller");
  // Initilization of Actuator Controller A.K.A. the Liang's library
  int stuff = 0 ;

  ActuatorController::initController(stuff, 0);

  ptr_AC = ActuatorController::getInstance();

  ptr_AC->autoRecoginze();





  ROS_INFO("Setting IMU Timers");
  //Using a Qt timer to request the quaternion at a fixed interval
  //Currently, the fastest rate is close to 30 Hz avoid to avoid request conflict
  QTimer * QuaTimer = new QTimer;
  connect(QuaTimer , &QTimer::timeout , this , &MyPlugin::quatimercallback);
  QuaTimer->start(30);


  //Connect the Actuator Controller Signal to a member function
  ptr_AC->m_sQuaternion.connect_member(this , &MyPlugin::quaternioncallback);


  //Using a Qt timer to calculate the joint angles at a fixed interval
  //You can combine this functnion with the previous timer too
  ConvertTimer = new QTimer;
  connect(ConvertTimer , &QTimer::timeout , this , &MyPlugin::quaterion2joint);


  // This is a button trigger the initlization
  connect(ui_.InitIMU , &QPushButton::clicked, this , &MyPlugin::on_InitIMU_clicked);



  // This is a button that trigger the recording function
  connect(ui_.CollectIMU , &QPushButton::clicked , this , &MyPlugin::on_CollectIMU_clicked);

  // This Button saves the recording
  connect(ui_.SaveIMU , &QPushButton::clicked , this , &MyPlugin::on_SaveIMU_clicked);



  // Clears the data if you don't want it 
  connect(ui_.ClearIMU , &QPushButton::clicked , this , &MyPlugin::on_ClearIMU_clicked);
}





// This initializes the IMU
void MyPlugin::on_InitIMU_clicked()
{

  XR1IMUptr->Initialize();

  //Start the calculation timer
  ConvertTimer->stop();

  //There is no point in having the timer running faster than the reading timer.
  ConvertTimer->start(50);
}





void MyPlugin::quatimercallback()
{
  // Tell em we need the IMU readings yo
  ptr_AC->requestAllQuaternions();
}





void MyPlugin::quaternioncallback(uint8_t id , double w, double x , double y , double z) {

  // Just feed all the information to XR1IMU library
  XR1IMUptr->quaternioncallback( id ,  w,  x ,  y ,  z);
}





void MyPlugin::quaterion2joint() {


  //Now you get the joint angles from them
  std::vector<double> angles = XR1IMUptr->getJointAngles();


  //Check if any module is missing or lagging
  std::vector<u_int8_t> missed_ids = XR1IMUptr->checkModules();

  for (int i = 0 ; i < missed_ids.size() ; i++){
  	ROS_INFO("The Missing ID inclues [%d]" , missed_ids[i]);
  }




  // The rest is just vrep and ROS mumble jumble
  if (ui_.tabWidget->currentIndex() == 7) {

    std::vector<double> imu_joint_angles;

    for (int i = XR1::Knee_X ; i <= XR1::Right_Wrist_Y ; i++)
      imu_joint_angles.push_back(angles[i]);

    JointTargetPositionPublisher.publish(ConvertJointAnglesMsgs(imu_joint_angles));

    double imu_hand_anglesl[5];

    for (int i = XR1::Left_Thumb ; i <= XR1::Left_Pinky ; i++)
      imu_hand_anglesl[i - XR1::Left_Thumb] = angles[i];


    LeftHandJointTargetPositionPublisher.publish(ConvertHandJointAngleMsgs(imu_hand_anglesl));

    double imu_hand_anglesr[5];


    for (int i = XR1::Right_Thumb ; i <= XR1::Right_Pinky ; i++)
      imu_hand_anglesr[i - XR1::Right_Thumb] = angles[i];

    RightHandJointTargetPositionPublisher.publish(ConvertHandJointAngleMsgs(imu_hand_anglesr));


    if (CollectIMUSwitch) {

      std::vector<double> temp_IMUData;

      for (int i = XR1::Knee_X; i <= XR1::Right_Pinky; ++i)
        temp_IMUData.push_back(angles[i]) ;

      //Not Controlling them with omni wheels I mean come on
      temp_IMUData.push_back(0);
      temp_IMUData.push_back(0);
      temp_IMUData.push_back(0);


      IMUData.push_back(temp_IMUData);

    }

  }
}


// This just flips a switch
void MyPlugin::on_CollectIMU_clicked() {

  CollectIMUSwitch = !CollectIMUSwitch;

  QString text = (CollectIMUSwitch) ? "Recording" : "Recording Paused";

  ui_.CollectIMU->setText(text);

}


void MyPlugin::on_SaveIMU_clicked() {



  // std::vector<std::vector<double> > ready2writedata = saveIMUHelper();


  std::vector<std::vector<double> > ready2writedata = IMUData;

  if (ready2writedata.size()) {
    QFileDialog dialog(0, tr("Save IMU Data"), QDir::currentPath());
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setAcceptMode(QFileDialog::AcceptSave);
    dialog.setNameFilter(tr("XR1Data(*.data)"));

    if (dialog.exec() == QDialog::Accepted)
    {
      QString path = dialog.selectedFiles().first();
      if (path.size() > 0)
      {

        if (!path.endsWith(".data"))
        {
          path += ".data";
        }
        QFile file(path);
        if (file.open(QFile::WriteOnly | QFile::Text | QFile::Truncate))
        {
          QTextStream out(&file);

          for (int i = 0 ; i < ready2writedata.size() ; i++) {
            for (int j = 0 ; j < ready2writedata[0].size(); j++) {
              out << ready2writedata[i][j] << ",";
            }
            out <<  endl;
          }
        }
        file.close();
      }
    }
  }


}


void MyPlugin::on_ClearIMU_clicked(){
  while (!IMUData.empty())
    IMUData.pop_back();
}


std::vector<std::vector<double> > MyPlugin::saveIMUHelper() {

  std::vector<std::vector<double> > res;


  

  if (IMUData.size()) {


    for (int i = 0 ; i <= IMUData.size() ; i++) {


      ROS_INFO("Recording Step [%d]"  , i);

      std::vector<double> goal_position;

      double steps = 5.0;

      if ( i == IMUData.size() ) {
        while (goal_position.size() < IMUData[0].size())
          goal_position.push_back(0);

        steps = 50;
      }
      else
        goal_position = IMUData[i];

      std::vector<double> start_position ;

      if ( i == 0 ) {
        while (start_position.size() < goal_position.size())
          start_position.push_back(0);
        steps = 50;
      }

      else
        start_position = IMUData[i - 1];



      for (int i = 1 ; i < steps ; i++) {

        std::vector<double> intermediate_position;

        for (int j = 0 ; j < start_position.size() ; j++) {
          intermediate_position.push_back(tinyBezier ((double)i/steps,  start_position[j] , start_position[j]  , goal_position[j] , goal_position[j] ) );
        }

        res.push_back(intermediate_position);

      }

      res.push_back(goal_position);
    }

  }


  return res;
}



double MyPlugin::tinyBezier(double double_index , double pt_s , double pt_1 , double pt_2 , double pt_e){

        return pow( (1.0 - double_index) , 3.0) * pt_s + 
                    3.0 * pow( (1.0 - double_index) , 2.0) *  double_index * pt_1 + 
                    3.0 * (1.0 - double_index) *  pow(double_index ,2.0) * pt_2 +
                    pow(double_index ,3.0) * pt_e;
}



} // namespace


// PLUGINLIB_DECLARE_CLASS(vrep_test, MyPlugin, vrep_test::MyPlugin, rqt_gui_cpp::Plugin)
