#include "my_plugin.h"
#include "ratio_layouted_frame.h"
#include <pluginlib/class_list_macros.h>


#define PI 3.141592654
namespace vrep_test {

MyPlugin::MyPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
  , Path_Ex_Timer(NULL)
  , playing_switch(0)
  , CollectIMUSwitch(0)
{
  // Constructor is called first before initPlugin function, needless to say.
  ROS_INFO("Loading vrep_test plugin !");
  // give QObjects reasonable names
  setObjectName("MyPlugin");
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{

  ROS_INFO("Loading vrep_test plugin !");
  Path_idx = 0;

  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);

  //Simulation Control buttons set up
  SimulationStartPublisher =  getNodeHandle().advertise<std_msgs::Bool>("/startSimulation", 1);
  SimulationPausePublisher =  getNodeHandle().advertise<std_msgs::Bool>("/pauseSimulation", 1);
  SimulationStopPublisher =  getNodeHandle().advertise<std_msgs::Bool>("/stopSimulation", 1);
  connect(ui_.Start_Simulation, SIGNAL(clicked()), this, SLOT(onStartButtonClicked()));
  connect(ui_.Stop_Simulation, SIGNAL(clicked()), this, SLOT(onStopButtonClicked()));
  connect(ui_.Pause_Simulation, SIGNAL(clicked()), this, SLOT(onPauseButtonClicked()));
  connect(ui_.Zero, SIGNAL(clicked()), this, SLOT(onZero()));

  // Get Joint Current Angles Subcribers under control
  // Linking the subcriber and refresher WILL crash the GUI
  sortLabelLists();
  JointCurrentPositionSubscriber = getNodeHandle().subscribe("/JointAngle/Current" , 100 , &MyPlugin::subscribeJointCurrentPosition, this);
  while (currentPosition.size() < currentPositionLabels.size()) currentPosition.push_back(0.0);
  JointCurrentPositionTimer = new QTimer(this);
  connect(JointCurrentPositionTimer, SIGNAL(timeout()), this, SLOT(JointCurrentPositionRefresher()));
  JointCurrentPositionTimer->start(500);

  // Set up Joint Target Position Publsiher
  JointTargetPositionPublisher = getNodeHandle().advertise<vrep_test::JointAngles>("/JointAngle/Target", 10);
  sortSliderLists();


  // Reset the joint limits;
  setup_main_joint_limit();
  setup_arm_joint_limit();
  setup_hand_joint_limit();
  // setup_hand_joint_limit();


  // Set up all the sliders for each joint
  for (int i = 0 ; i < joint_lower_limit.size() ; i++) {
    targetPositionSliders[i]->setMinimum(0);
    targetPositionSliders[i]->setMaximum(100);
    targetPositionSliders[i]->setValue(floor((0.0 - joint_lower_limit[i]) / (joint_upper_limit[i] - joint_lower_limit[i]) * 100));
  }


  //Hand Control Set up
  sortHandSliderLists();
  LeftHandJointTargetPositionPublisher = getNodeHandle().advertise<vrep_test::HandJointAngles>("/HandJointAngles/Left/Target", 10);
  RightHandJointTargetPositionPublisher = getNodeHandle().advertise<vrep_test::HandJointAngles>("/HandJointAngles/Right/Target", 10);

  for (int i = 0 ; i < hand_joint_lower_limit.size() ; i++) {
    HandPositionSliders[0][i]->setMinimum(0);
    HandPositionSliders[0][i]->setMaximum(100);
    HandPositionSliders[0][i]->setValue(floor((0.0 - hand_joint_lower_limit[i]) / (hand_joint_upper_limit[i] - hand_joint_lower_limit[i]) * 100));
    HandPositionSliders[1][i]->setMinimum(0);
    HandPositionSliders[1][i]->setMaximum(100);
    HandPositionSliders[1][i]->setValue(floor((0.0 - hand_joint_lower_limit[i]) / (hand_joint_upper_limit[i] - hand_joint_lower_limit[i]) * 100));

  }


  for (int i = 0 ; i < HandPositionSliders[0].size() ; i++ ) connect(HandPositionSliders[0][i], SIGNAL(valueChanged(int)), this, SLOT(onHandJointTargetPositionChanged(int)));
  for (int i = 0 ; i < HandPositionSliders[1].size() ; i++ ) connect(HandPositionSliders[1][i], SIGNAL(valueChanged(int)), this, SLOT(onHandJointTargetPositionChanged(int)));


  for (int i = 0 ; i < targetPositionSliders.size() ; i++ ) connect(targetPositionSliders[i], SIGNAL(valueChanged(int)), this, SLOT(onJointTargetPositionChanged(int)));
  for (int i = 0 ; i < targetPositionSliders.size() ; i++ ) connect(targetPositionSliders[i], SIGNAL(sliderPressed()), this, SLOT(onJointRotationVisualization()));
  for (int i = 0 ; i < targetPositionSliders.size() ; i++ ) connect(targetPositionSliders[i], SIGNAL(sliderReleased()), this, SLOT(onJointRotationVisualizationFinish()));


  JointVisualizationPublisher = getNodeHandle().advertise<std_msgs::Int32>("/JointVisualization/Signal", 1);



  //ImageTransportage Viewer Setup
  ui_.ImageTopicComboBox->setCurrentIndex(ui_.ImageTopicComboBox->findText(""));
  connect(ui_.ImageTopicComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicChanged(int)));
  ui_.RefreshTopics->setIcon(QIcon::fromTheme("view-refresh"));
  connect(ui_.RefreshTopics, SIGNAL(pressed()), this, SLOT(updateTopicList()));
  ui_.image_frame->setOuterLayout(ui_.image_layout);
  QString transport = "raw";
  image_transport::ImageTransport it(getNodeHandle());
  image_transport::TransportHints hints(transport.toStdString());
  CameraSubscriber = it.subscribe("/sensor_msgs/Image/FrontCamera", 1, &MyPlugin::callbackImage, this);



  //Combo Box to Change Mode
  ModeChangePublisher = getNodeHandle().advertise<std_msgs::Bool>("/XR1/ModeChange", 1);
  // connect(ui_.Mode_Combo, SIGNAL(currentIndexChanged(int)), this, SLOT(onModeChanged(int)));
  connect(ui_.tabWidget , SIGNAL(currentChanged(int)) , this , SLOT(onModeChanged(int)));



  //IK Mode setup
  sortSpinBoxLists();
  IKTargetPositionPublisher = getNodeHandle().advertise<vrep_test::IK_msg>("/XR1/IK_msg", 10);
  for (int i = 0 ; i < targetPositionSpinBox.size() ; i++ ) connect(targetPositionSpinBox[i], SIGNAL(valueChanged(double)), this, SLOT(onIKTargetPositionChanged(double)));


  //Steering
  // TwistPublisher = getNodeHandle().advertise<geometry_msgs::Twist>("/XR1/Base/cmd", 10);

  //GetTF
  // ptr_XR1 = new XR1();

  //Inertia Parameters Query
  // connect(ui_.InertiaParaQuery, SIGNAL(clicked()), this, SLOT(onInertiaParaClicked()));
  // InertiaParaClient = getNodeHandle().serviceClient<vrep_test::InertiaPara>("InertiaPara_Query");

  CurrentClient = getNodeHandle().serviceClient<vrep_test::JointCurrent>("/vrep_ros_interface/JointCurrent_Query");


  //MakeShift Timer?
  //LOL nevermind ros timer does not work in rqt
  // ros::Timer just_timer = getNodeHandle().createTimer(ros::Duration(0.001),boost::bind(&MyPlugin::just_timer_callback,this));


  //TODO Calculate torques and stuff


  // connect(ui_.Generate_Configuration, SIGNAL(clicked()) , this , SLOT(onGenerate_ConfigurationClicked()));


  //TODO Gravity Compensation
  //


  // connect(ui_.Use_Sync , SIGNAL(clicked()) , this , connect(on_Sync_Clicked));

  // SyncPublisher = getNodeHandle().advertise<std_msgs::Bool>("/enableSyncMode", 1);

  // SyncFinishedSubscriber = getNodeHandle().subscribe("/simulationStepDone" , 1 , &MyPlugin::syncFinishedCallback, this);

  // NextStepPublisher  = nh.advertise<std_msgs::Bool>("triggerNextStep", 1);











  // Emergency Dance

  connect(ui_.Dance_Button, SIGNAL(clicked()) , this , SLOT(onDance_ButtonClicked()));

  //add by lzj
  connect(ui_.readAction, &QPushButton::clicked, this, &MyPlugin::readAction);
  connect(ui_.actionList, &QListWidget::currentRowChanged, this, &MyPlugin::selectActionChanged);
  connect(ui_.saveAction, &QPushButton::clicked, this, &MyPlugin::saveAction);
  connect(ui_.removeAction, &QPushButton::clicked, this, &MyPlugin::onBtnRemoveClicked);
  connect(ui_.modifyAction, &QPushButton::clicked, this, &MyPlugin::modifyAction);
  connect(ui_.play, &QPushButton::clicked, this, &MyPlugin::play);
  connect(ui_.generate, &QPushButton::clicked, this, &MyPlugin::generateActuatorData);
  connect(ui_.addAction, &QPushButton::clicked, this, &MyPlugin::onBtnAddClicked);


  Path_Ex_Timer = new QTimer(this);
  connect(Path_Ex_Timer, SIGNAL(timeout()), this, SLOT(Path_Ex_Fun()));


  // OmniPositions.push_back(0.0);OmniPositions.push_back(0.0);OmniPositions.push_back(0.0);











  //--------------------------------------------------------------------------------------

  int stuff = 0 ;

  ActuatorController::initController(stuff, 0);

  ptr_AC = ActuatorController::getInstance();

  ptr_AC->autoRecoginze();


  QTimer * QuaTimer = new QTimer;

  connect(QuaTimer , &QTimer::timeout , this , &MyPlugin::quatimercallback);
  QuaTimer->start(30);

  ptr_AC->m_sQuaternion.connect_member(this , &MyPlugin::quaternioncallback);

  ConvertTimer = new QTimer;

  connect(ConvertTimer , &QTimer::timeout , this , &MyPlugin::quaterion2joint);



  connect(ui_.InitIMU , &QPushButton::clicked, this , &MyPlugin::on_InitIMU_clicked);

  FigureGearRatios = 1.0 + 54.0 / 45.0 + 54.0 * 54.0 / 45.0 / 45.0;

  ThumbGearRatio = 1.0 + 54.0 / 45.0;

  connect(ui_.CollectIMU , &QPushButton::clicked , this , &MyPlugin::on_CollectIMU_clicked);
  connect(ui_.SaveIMU , &QPushButton::clicked , this , &MyPlugin::on_SaveIMU_clicked);




  //-------------------------------------------------------------------------------


  Left_ShoulderX_q = 0;
  Left_ShoulderY_q = 0;
  Left_Elbow_Z_q = 0;
  Left_Elbow_X_q = 0;
  Left_Wrist_Y_q = 0;
  Left_Wrist_X_q = 0;
  Left_Wrist_Z_q = 0;
  L_I_q = 0;
  L_T_q = 0;
  L_R_q = 0;
  L_P_q = 0;
  L_M_q = 0;
  Right_ShoulderX_q = 0;
  Right_ShoulderY_q = 0;
  Right_Elbow_Z_q = 0;
  Right_Elbow_X_q = 0;
  Right_Wrist_Y_q = 0;
  Right_Wrist_X_q = 0;
  Right_Wrist_Z_q = 0;
  R_I_q = 0;
  R_T_q = 0;
  R_R_q = 0;
  R_P_q = 0;
  R_M_q = 0;


}

void MyPlugin::shutdownPlugin()
{
  // unregister all publishers here
  JointTargetPositionPublisher.shutdown();
  SimulationStartPublisher.shutdown();
  SimulationPausePublisher.shutdown();
  SimulationStopPublisher.shutdown();
  IKTargetPositionPublisher.shutdown();
  LeftHandJointTargetPositionPublisher.shutdown();
  RightHandJointTargetPositionPublisher.shutdown();
  ModeChangePublisher.shutdown();
  JointVisualizationPublisher.shutdown();
  TwistPublisher.shutdown();
}

void MyPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void MyPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/


void MyPlugin::JointCurrentPositionRefresher() {
  // Do not refresh the rqt interface at a rate faster than 50 Hz, it may crash the application
  for (int i = 0; i < currentPosition.size(); i++) {
    currentPositionLabels[i]->setText(QString::number(currentPosition[i], 'g', 3));
    if (ui_.tabWidget->currentIndex() == 5 )
      targetPositionSliders[i]->setValue((int) ((currentPosition[i] - joint_lower_limit[i]) / (joint_upper_limit[i] - joint_lower_limit[i]) * 100 ));
  }
}

void MyPlugin::onJointTargetPositionChanged(int i) {
  std::vector<double> targetPosition;
  for (int i = 0; i < targetPositionSliders.size(); i++) {
    targetPosition.push_back((double)targetPositionSliders[i]->value() / 100. * ( joint_upper_limit[i] - joint_lower_limit[i]) + joint_lower_limit[i] );
    targetPositionLabels[i]->setText(QString::number(targetPosition.back(), 'g', 3));
  }
  if (ui_.tabWidget->currentIndex() != 5  && !playing_switch )JointTargetPositionPublisher.publish(ConvertJointAnglesMsgs(targetPosition));
}



vrep_test::HandJointAngles MyPlugin::ConvertHandJointAngleMsgs(double HandPosition[5]) {
  vrep_test::HandJointAngles msg;
  msg.Thumb = HandPosition[0];
  msg.Index = HandPosition[1];
  msg.Middle = HandPosition[2];
  msg.Ring = HandPosition[3];
  msg.Pinky = HandPosition[4];
  return msg;
}

void MyPlugin::sortLabelLists() {
  while (!currentPositionLabels.isEmpty())  currentPositionLabels.pop_back();
  for (int i = 0 ; i < 21; i++) currentPositionLabels.append(ui_.tabWidget->findChild<QLabel *>("Joint_Current_Label_" + QString::number(i)));

  while (!targetPositionLabels.isEmpty())  targetPositionLabels.pop_back();
  for (int i = 0 ; i < 21; i++) targetPositionLabels.append(ui_.tabWidget->findChild<QLabel *>("Joint_Target_Label_" + QString::number(i)));
}

void MyPlugin::sortSliderLists() {
  while (!targetPositionSliders.isEmpty())  targetPositionSliders.pop_back();
  for (int i = 0 ; i < 7; i++) targetPositionSliders.append(ui_.tabWidget->findChild<QSlider *>("Body_T_S_" + QString::number(i)));
  for (int i = 0 ; i < 7; i++) targetPositionSliders.append(ui_.tabWidget->findChild<QSlider *>("Left_Arm_T_S_" + QString::number(i)));
  for (int i = 0 ; i < 7; i++) targetPositionSliders.append(ui_.tabWidget->findChild<QSlider *>("Right_Arm_T_S_" + QString::number(i)));
}

void MyPlugin::sortSpinBoxLists() {
  targetPositionSpinBox.append(ui_.LPX);
  targetPositionSpinBox.append(ui_.LPY);
  targetPositionSpinBox.append(ui_.LPZ);
  targetPositionSpinBox.append(ui_.LRX);
  targetPositionSpinBox.append(ui_.LRY);
  targetPositionSpinBox.append(ui_.LRZ);
  targetPositionSpinBox.append(ui_.RPX);
  targetPositionSpinBox.append(ui_.RPY);
  targetPositionSpinBox.append(ui_.RPZ);
  targetPositionSpinBox.append(ui_.RRX);
  targetPositionSpinBox.append(ui_.RRY);
  targetPositionSpinBox.append(ui_.RRZ);
}

void MyPlugin::sortHandSliderLists() {
  QVector<QSlider *> temp;
  temp.append(ui_.LHT);
  temp.append(ui_.LHI);
  temp.append(ui_.LHM);
  temp.append(ui_.LHR);
  temp.append(ui_.LHP);
  HandPositionSliders.append(temp);
  while (!temp.isEmpty()) temp.pop_back();
  temp.append(ui_.RHT);
  temp.append(ui_.RHI);
  temp.append(ui_.RHM);
  temp.append(ui_.RHR);
  temp.append(ui_.RHP);
  HandPositionSliders.append(temp);
}

void MyPlugin::onStartButtonClicked() {
  std_msgs::Bool data;
  data.data = true;
  SimulationStartPublisher.publish(data);

  updateTopicList();
  onZero();
}

void MyPlugin::onStopButtonClicked() {
  std_msgs::Bool data;
  data.data = true;
  SimulationStopPublisher.publish(data);
  ui_.image_frame->setImage(QImage());
  onZero();
}

void MyPlugin::onPauseButtonClicked() {
  std_msgs::Bool data;
  data.data = true;
  SimulationPausePublisher.publish(data);
}

void MyPlugin::onZero() {
  // Zero function doing what zeroing functioins do
  for (int i = 0 ; i < currentPositionLabels.size(); i++) currentPositionLabels[i]->setText("0.0");
  for (int i = 0 ; i < targetPositionLabels.size(); i++) targetPositionLabels[i]->setText("0.0");



  // Set up all the sliders for each joint
  for (int i = 0 ; i < joint_lower_limit.size() ; i++) {
    targetPositionSliders[i]->setValue(floor((0.0 - joint_lower_limit[i]) / (joint_upper_limit[i] - joint_lower_limit[i]) * 100));
  }

  for (int i = 0 ; i < hand_joint_lower_limit.size() ; i++) {
    HandPositionSliders[0][i]->setValue(floor((0.0 - hand_joint_lower_limit[i]) / (hand_joint_upper_limit[i] - hand_joint_lower_limit[i]) * 100));
    HandPositionSliders[1][i]->setValue(floor((0.0 - hand_joint_lower_limit[i]) / (hand_joint_upper_limit[i] - hand_joint_lower_limit[i]) * 100));
  }

  // ui_.Twist_X->setValue(50);
  // ui_.Twist_Y->setValue(50);
  // ui_.Twist_Z->setValue(50);

  targetPositionSpinBox[0]->setValue(0.2);
  targetPositionSpinBox[1]->setValue(0.0);
  targetPositionSpinBox[2]->setValue(0.75);
  targetPositionSpinBox[3]->setValue(0.0);
  targetPositionSpinBox[4]->setValue(0.0);
  targetPositionSpinBox[5]->setValue(0.0);
  targetPositionSpinBox[6]->setValue(0.2);
  targetPositionSpinBox[7]->setValue(0.0);
  targetPositionSpinBox[8]->setValue(0.75);
  targetPositionSpinBox[9]->setValue(0.0);
  targetPositionSpinBox[10]->setValue(0.0);
  targetPositionSpinBox[11]->setValue(0.0);
}


void MyPlugin::callbackImage(const sensor_msgs::Image::ConstPtr& msg) {
  try
  {
    // First let cv_bridge do its magic
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    conversion_mat_ = cv_ptr->image;

    // if (num_gridlines_ > 0)
    // overlayGrid();
  }
  catch (cv_bridge::Exception& e)
  {
    targetPositionLabels[4]->setText("cv nope");
    try
    {
      // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
      if (msg->encoding == "CV_8UC3")
      {
        // assuming it is rgb
        conversion_mat_ = cv_ptr->image;
      } else if (msg->encoding == "8UC1") {
        // convert gray to rgb
        cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
      } else if (msg->encoding == "16UC1" || msg->encoding == "32FC1") {
        // scale / quantify
        double min = 0;
        double max = 100.000000000000000;
        if (msg->encoding == "16UC1") max *= 1000;
        // if (ui_.dynamic_range_check_box->isChecked())
        {
          // dynamically adjust range based on min/max in image
          cv::minMaxLoc(cv_ptr->image, &min, &max);
          if (min == max) {
            // completely homogeneous images are displayed in gray
            min = 0;
            max = 2;
          }
        }
        cv::Mat img_scaled_8u;
        cv::Mat(cv_ptr->image - min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));
        cv::cvtColor(img_scaled_8u, conversion_mat_, CV_GRAY2RGB);
      } else {
        qWarning("ImageView.callback_image() could not convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what());
        ui_.image_frame->setImage(QImage());
        return;
      }
    }
    catch (cv_bridge::Exception& e)
    {
      qWarning("ImageView.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg->encoding.c_str(), e.what());
      ui_.image_frame->setImage(QImage());
      return;
    }
  }

  // image must be copied since it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
  QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step[0], QImage::Format_RGB888);
  ui_.image_frame->setImage(image);
  // if (!ui_.zoom_1_push_button->isEnabled())
  // {
  //   ui_.zoom_1_push_button->setEnabled(true);
  // }
  // // Need to update the zoom 1 every new image in case the image aspect ratio changed,
  // // though could check and see if the aspect ratio changed or not.
  // onZoom1(ui_.zoom_1_push_button->isChecked());
}

QList<QString> MyPlugin::getTopicList(const QSet<QString>& message_types, const QList<QString>& transports) {

  QSet<QString> message_sub_types;
  return getTopics(message_types, message_sub_types, transports).values();
}

QSet<QString> MyPlugin::getTopics(const QSet<QString>& message_types, const QSet<QString>& message_sub_types, const QList<QString>& transports) {
  // Get all the Image topics
  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  QSet<QString> all_topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    all_topics.insert(it->name.c_str());
  }

  QSet<QString> topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    if (message_types.contains(it->datatype.c_str()))
    {
      QString topic = it->name.c_str();

      // add raw topic
      topics.insert(topic);
      //qDebug("ImageView::getTopics() raw topic '%s'", topic.toStdString().c_str());

      // add transport specific sub-topics
      for (QList<QString>::const_iterator jt = transports.begin(); jt != transports.end(); jt++)
      {
        if (all_topics.contains(topic + "/" + *jt))
        {
          QString sub = topic + " " + *jt;
          topics.insert(sub);
          //qDebug("ImageView::getTopics() transport specific sub-topic '%s'", sub.toStdString().c_str());
        }
      }
    }
    if (message_sub_types.contains(it->datatype.c_str()))
    {
      QString topic = it->name.c_str();
      int index = topic.lastIndexOf("/");
      if (index != -1)
      {
        topic.replace(index, 1, " ");
        topics.insert(topic);
        //qDebug("ImageView::getTopics() transport specific sub-topic '%s'", topic.toStdString().c_str());
      }
    }
  }
  return topics;
}

void MyPlugin::selectTopic(const QString& topic) {
  int index = ui_.ImageTopicComboBox->findText(topic);
  if (index == -1)
  {
    // add topic name to list if not yet in
    QString label(topic);
    label.replace(" ", "/");
    ui_.ImageTopicComboBox->addItem(label, QVariant(topic));
    index = ui_.ImageTopicComboBox->findText(topic);
  }
  ui_.ImageTopicComboBox->setCurrentIndex(index);
}

void MyPlugin::updateTopicList() {
  QSet<QString> message_types;
  message_types.insert("sensor_msgs/Image");
  QSet<QString> message_sub_types;
  message_sub_types.insert("sensor_msgs/CompressedImage");

  // get declared transports
  QList<QString> transports;
  image_transport::ImageTransport it(getNodeHandle());
  std::vector<std::string> declared = it.getDeclaredTransports();
  for (std::vector<std::string>::const_iterator it = declared.begin(); it != declared.end(); it++)
  {
    QString transport = it->c_str();
    // strip prefix from transport name
    QString prefix = "image_transport/";
    if (transport.startsWith(prefix))
    {
      transport = transport.mid(prefix.length());
    }
    transports.append(transport);
  }

  QString selected = ui_.ImageTopicComboBox->currentText();

  // fill combo box
  QList<QString> topics = getTopics(message_types, message_sub_types, transports).values();
  topics.append("");
  qSort(topics);
  ui_.ImageTopicComboBox->clear();
  for (QList<QString>::const_iterator it = topics.begin(); it != topics.end(); it++)
  {
    QString label(*it);
    label.replace(" ", "/");
    ui_.ImageTopicComboBox->addItem(label, QVariant(*it));
  }

  // restore previous selection
  selectTopic(selected);
}

void MyPlugin::onTopicChanged(int index) {
  //Change the display image topics
  CameraSubscriber.shutdown();

  // reset image on topic change
  ui_.image_frame->setImage(QImage());

  QStringList parts = ui_.ImageTopicComboBox->itemData(index).toString().split(" ");
  QString topic = parts.first();
  QString transport = parts.length() == 2 ? parts.last() : "raw";

  if (!topic.isEmpty())
  {
    image_transport::ImageTransport it(getNodeHandle());
    image_transport::TransportHints hints(transport.toStdString());
    try {
      CameraSubscriber = it.subscribe(topic.toStdString(), 1, &MyPlugin::callbackImage, this, hints);
    } catch (image_transport::TransportLoadException& e) {
      QMessageBox::warning(widget_, tr("Loading image transport plugin failed"), e.what());
    }
  }
  // onMousePublish(ui_.publish_click_location_check_box->isChecked());
}

void MyPlugin::onModeChanged(int index) {
  // Change control mode
  std_msgs::Bool mode;
  (index == 5) ? mode.data = true : mode.data = false;
  for (int i = 0 ; i < targetPositionSliders.size(); i++) targetPositionSliders[i]->setEnabled(!mode.data);
  ModeChangePublisher.publish(mode);
  // onMousePublish(ui_.publish_click_location_check_box->isChecked());
}

void MyPlugin::onIKTargetPositionChanged(double d) {
  // move the target dummy in vrep
  std::vector<double> IKtargetPosition;
  for (int i = 0; i < targetPositionSpinBox.size(); i++) {
    IKtargetPosition.push_back(targetPositionSpinBox[i]->value());
  }

  IKTargetPositionPublisher.publish(ConvertIkMsgs(IKtargetPosition));
}

void MyPlugin::onHandJointTargetPositionChanged(int i) {

  double temp[5];
  for (int i = 0; i < 5; i++) {
    temp[i] = (double)HandPositionSliders[0][i]->value() / 100. * (hand_joint_upper_limit[i] - hand_joint_lower_limit[i]) + hand_joint_lower_limit[i];
  }
  if (!playing_switch) LeftHandJointTargetPositionPublisher.publish(ConvertHandJointAngleMsgs(temp));
  for (int i = 0; i < 5; i++) {
    temp[i] = (double)HandPositionSliders[1][i]->value() / 100. * (hand_joint_upper_limit[i] - hand_joint_lower_limit[i]) + hand_joint_lower_limit[i];
  }
  if (!playing_switch) RightHandJointTargetPositionPublisher.publish(ConvertHandJointAngleMsgs(temp));

}


void MyPlugin::onJointRotationVisualization() {
  //It just moves the silly ring around the selected joint when it moves
  QSlider * slider = (QSlider *) sender();
  std_msgs::Int32 msg;
  msg.data = -1;
  for (int i = 0; i < targetPositionSliders.size(); i++) {
    if (targetPositionSliders[i] == slider) msg.data = i;
  }

  if (msg.data >= 0) JointVisualizationPublisher.publish(msg);
}

void MyPlugin::onJointRotationVisualizationFinish() {
  //Show's over, go back to hiding, ring
  QSlider * slider = (QSlider *) sender();
  std_msgs::Int32 msg;
  msg.data = -1;
  JointVisualizationPublisher.publish(msg);
}


// void MyPlugin::onSteeringValueChanged(int) {
//   // Send out a twist msg
//   geometry_msgs::Twist msg;
//   msg.linear.x = ((double)ui_.Twist_X->value() - 50.) * 0.02;
//   msg.linear.y = ((double)ui_.Twist_Y->value() - 50.) * 0.02;
//   msg.angular.z = ((double)ui_.Twist_Z->value() - 50.) * 0.01;

//   TwistPublisher.publish(msg);
// }

// void MyPlugin::onInertiaParaClicked() {
//   //Example for ros service communication
//   ros::ServiceClient client = getNodeHandle().serviceClient<vrep_test::InertiaPara>("/vrep_ros_interface/InertiaPara_Query");
//   // ptr_XR1->callInertiaPara(client);
// }


// void MyPlugin::onGenerate_ConfigurationClicked() {
//   // static const double ranges[7] = {2 * PI , PI , 2 * PI , PI / 2.0 - 0.1 , 2.0 * PI , PI / 2.0 , PI / 2.0};

//   // static const double lower_bounds[7] = { -PI , 0.0 , -PI , 0.0, -PI , -PI / 4.0 , -PI / 4.0};

//   static const double ranges[7] =     {2 * PI , PI ,  PI      , -PI / 2.0 + 0.1 , 2.0 * PI , PI / 2.0 , PI / 2.0};

//   static const double lower_bounds[7] = { -PI , 0.0 , -PI / 2.0 , 0.0,              -PI ,     -PI / 4.0 , -PI / 4.0};

//   ROS_INFO("Generating Configurations");

//   while (!GeneratedConfiguration.empty()) GeneratedConfiguration.pop_back();
//   while (!CurrentData.empty()) CurrentData.pop_back();

//   for (int i = 0 ; i < 100 ; i++) {
//     std::vector<double> temp_angles(7);
//     for (int j = 0 ; j < temp_angles.size(); j++) temp_angles[j] = ((double)rand() / (double)RAND_MAX * ranges[j]) + lower_bounds[j];
//     GeneratedConfiguration.push_back(temp_angles);
//   }


//   //   for (int i = 0 ; i < 100 ; i++) {
//   //   std::vector<double> temp_angles(7);
//   //   for (int j = 0 ; j < temp_angles.size(); j++) temp_angles[j] = ((double)i * ranges[j] / 100.0) + lower_bounds[j];
//   //   GeneratedConfiguration.push_back(temp_angles);
//   // }

//   // int tempie = GeneratedConfiguration.size();
//   // int tempie2 = GeneratedConfiguration[0].size();
//   // ROS_INFO("Generated Configurations , size is [%d] [%d]" , tempie , tempie2);


//   // //Configuration 1: Excite the Hand link

//   // //MSX
//   // for (int i = 0 ; i < 50 ; i++){
//   //   std::vector<double> temp_angles(7);
//   //   temp_angles[0] = -PI/2;
//   //   temp_angles[1] = 0.0;
//   //   temp_angles[2] = 0.0;
//   //   temp_angles[3] = 0.0;
//   //   temp_angles[4] = 0.0;
//   //   temp_angles[5] = 0.0;
//   //   temp_angles[6] = ((double)rand() / (double)RAND_MAX * ranges[6]) + lower_bounds[6];
//   //   GeneratedConfiguration.push_back(temp_angles);
//   // }



// }




// void MyPlugin::onCollect_CurrentClicked() {

//   ui_.Save_Current->setEnabled(false);
//   ui_.Collect_Current->setText("Waiting");

//   if (GeneratedConfiguration.empty()) {

//   }
//   else {
//     for (int i = 0 ; i < targetPositionSliders.size(); i++) targetPositionSliders[i]->setEnabled(false);

//     for (int i = 0 ; i < GeneratedConfiguration.size() ; i++) {
//       ROS_INFO("New Position");
//       std::vector<double> targetPosition(21);

//       for (int j = 0; j < 7; j++) {
//         targetPosition[j] = 0.0;
//         targetPosition[j + 7] = GeneratedConfiguration[i][j];
//         targetPosition[j + 14] = 0.0;

//       }
//       ROS_INFO("Publishing New Position");
//       JointTargetPositionPublisher.publish(ConvertJointAnglesMsgs(targetPosition));


//       delay(2000);

//       CurrentData.push_back(processCurrents());

//     }
//   }

//   ui_.Generate_Configuration->setText("Finished Collection");

//   ui_.Save_Current->setEnabled(true);
// }



void MyPlugin::on_InitIMU_clicked()
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


  RS_init = RS_raw;
  RA_init = RA_raw;
  RH_init = RH_raw;

  R_I_init = R_I_raw ;
  R_T_init = R_T_raw ;
  R_R_init = R_R_raw ;
  R_P_init = R_P_raw ;
  R_M_init = R_M_raw ;

  ConvertTimer->stop();
  ConvertTimer->start(50);
}

void MyPlugin::quatimercallback()
{
  // ROS_INFO("Requesting !!!!");
  ptr_AC->requestAllQuaternions();
}


void MyPlugin::quaternioncallback(uint8_t id , double w, double x , double y , double z) {
  ROS_INFO("[%d] [%f] [%f] [%f] [%f]", id , w, x , y , z);
  Quaterniond temp_q;
  Quaterniond temp_temp_q;
  Quaterniond new_q;
  Quaterniond new_new_q;

  Matrix3d rot;

  Vector3d unit_y(0, 1, 0);
  Vector3d unit_x(1, 0, 0);

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
    LS_q = Back_raw.inverse() * LS_raw * temp_q.inverse();


    temp_q = Back_raw.inverse() * LS_raw;
    LS_v = temp_q.toRotationMatrix() * unit_y;
    break;


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


    rot = EulerXYZ(Left_ShoulderX_q , 0 , Left_ShoulderY_q);

    temp_q = Back_raw.inverse() * LA_raw;
    LA_v = temp_q.toRotationMatrix() * unit_y;
    LA_v = rot.transpose() * LA_v;
    break;

  case 7:
    LH_raw.w() = w;
    LH_raw.x() = x;
    LH_raw.y() = y;
    LH_raw.z() = z;


    temp_temp_q = Back_init.inverse() * LH_init;

    rot = EulerXYZ(Left_ShoulderX_q , 0 , Left_ShoulderY_q) * EulerZYX(-Left_Elbow_X_q , Left_Elbow_Z_q, 0);

    // rot =  temp_temp_q.toRotationMatrix();

    temp_q = Back_raw.inverse() * LH_raw ;
    LH_m =  rot.transpose() * temp_q.toRotationMatrix() * temp_temp_q.inverse().toRotationMatrix();
    break;

  case 9:

    L_T_raw.w() = w;
    L_T_raw.x() = x;
    L_T_raw.y() = y;
    L_T_raw.z() = z;


    temp_q = LH_init.inverse() * L_T_init;
    new_q = temp_q.inverse() * LH_init.inverse() * LH_raw * temp_q;
    L_T_qq = new_q.inverse() * L_T_init.inverse() * L_T_raw;

    L_T_v = L_T_qq.toRotationMatrix() * unit_x;
    break;

  case 11:

    L_I_raw.w() = w;
    L_I_raw.x() = x;
    L_I_raw.y() = y;
    L_I_raw.z() = z;

    temp_q = LH_raw.inverse() * L_I_raw;
    L_I_v = temp_q.toRotationMatrix() * unit_x;
    break;

  case 13:

    L_M_raw.w() = w;
    L_M_raw.x() = x;
    L_M_raw.y() = y;
    L_M_raw.z() = z;

    temp_q = LH_raw.inverse() * L_M_raw;
    L_M_v = temp_q.toRotationMatrix() * unit_x;
    break;

  case 15:

    L_R_raw.w() = w;
    L_R_raw.x() = x;
    L_R_raw.y() = y;
    L_R_raw.z() = z;

    temp_q = LH_raw.inverse() * L_R_raw;
    L_R_v = temp_q.toRotationMatrix() * unit_x;
    break;

  case 17:

    L_P_raw.w() = w;
    L_P_raw.x() = x;
    L_P_raw.y() = y;
    L_P_raw.z() = z;

    temp_q = LH_raw.inverse() * L_P_raw;
    L_P_v = temp_q.toRotationMatrix() * unit_x;

    break;

  case 4:
    RS_raw.w() = w;
    RS_raw.x() = x;
    RS_raw.y() = y;
    RS_raw.z() = z;

    temp_q =   Back_init.inverse() * RS_init;
    RS_q = Back_raw.inverse() * RS_raw * temp_q.inverse();


    temp_q = Back_raw.inverse() * RS_raw;
    RS_v = temp_q.toRotationMatrix() * unit_y;
    break;

  case 5:
    RA_raw.w() = w;
    RA_raw.x() = x;
    RA_raw.y() = y;
    RA_raw.z() = z;

    temp_q = Back_init.inverse() * RA_init;
    RA_q = RS_q.inverse() * Back_raw.inverse() * RA_raw * temp_q.inverse();


    rot = EulerXYZ(Right_ShoulderX_q , 0 , Right_ShoulderY_q);

    temp_q = Back_raw.inverse() * RA_raw;
    RA_v = temp_q.toRotationMatrix() * unit_y;
    RA_v = rot.transpose() * RA_v;
    break;

  case 8:
    RH_raw.w() = w;
    RH_raw.x() = x;
    RH_raw.y() = y;
    RH_raw.z() = z;

    temp_temp_q = Back_init.inverse() * RH_init;

    rot = EulerXYZ(Right_ShoulderX_q , 0 , Right_ShoulderY_q) * EulerZYX(-Right_Elbow_X_q , Right_Elbow_Z_q, 0);

    // rot =  temp_temp_q.toRotationMatrix();

    temp_q = Back_raw.inverse() * RH_raw ;
    RH_m =  rot.transpose() * temp_q.toRotationMatrix() * temp_temp_q.inverse().toRotationMatrix();
    break;


  case 10:

    R_T_raw.w() = w;
    R_T_raw.x() = x;
    R_T_raw.y() = y;
    R_T_raw.z() = z;


    temp_q = RH_init.inverse() * R_T_init;
    new_q = temp_q.inverse() * RH_init.inverse() * RH_raw * temp_q;
    R_T_qq = new_q.inverse() * R_T_init.inverse() * R_T_raw;

    R_T_v = R_T_qq.toRotationMatrix() * unit_x;
    break;

  case 12:

    R_I_raw.w() = w;
    R_I_raw.x() = x;
    R_I_raw.y() = y;
    R_I_raw.z() = z;

    temp_q = RH_raw.inverse() * R_I_raw;
    R_I_v = temp_q.toRotationMatrix() * unit_x;
    break;
  case 14:

    R_M_raw.w() = w;
    R_M_raw.x() = x;
    R_M_raw.y() = y;
    R_M_raw.z() = z;

    temp_q = RH_raw.inverse() * R_M_raw;
    R_M_v = temp_q.toRotationMatrix() * unit_x;
    break;

  case 16:

    R_R_raw.w() = w;
    R_R_raw.x() = x;
    R_R_raw.y() = y;
    R_R_raw.z() = z;

    temp_q = RH_raw.inverse() * R_R_raw;
    R_R_v = temp_q.toRotationMatrix() * unit_x;
    break;

  case 18:

    R_P_raw.w() = w;
    R_P_raw.x() = x;
    R_P_raw.y() = y;
    R_P_raw.z() = z;

    temp_q = RH_raw.inverse() * R_P_raw;
    R_P_v = temp_q.toRotationMatrix() * unit_x;
    break;

  default:
    break;
  }
}



void MyPlugin::quaterion2joint() {


  Vector3d temp = quaternion2ZYX( Back_q.w(), Back_q.x(), Back_q.y(), Back_q.z());
  ROS_INFO(" The Back X is [%f] " , temp(2));
  ROS_INFO(" The Back Z is [%f] " , temp(0));



  Quaterniond ran_q = Back_raw.inverse() * RS_raw;
  ROS_INFO(" The RS Quaternion is [%f] [%f] [%f] [%f]" , ran_q.w() , ran_q.x() , ran_q.y() , ran_q.z());


  ROS_INFO(" The RS Vector is [%f] [%f] [%f]" , RS_v(0) , RS_v(1) , RS_v(2));
  temp = Vector2XZ(RS_v);

  Right_ShoulderX_q = EasyKalman(Right_ShoulderX_q , temp(0));
  Right_ShoulderY_q = EasyKalman(Right_ShoulderY_q , temp(2));


  ROS_INFO(" The Right_ShoulderX_q is [%f] " , Right_ShoulderX_q);
  ROS_INFO(" The Right_ShoulderY_q is [%f] " , Right_ShoulderY_q);


  ROS_INFO(" The RA Vector is [%f] [%f] [%f]" , RA_v(0) , RA_v(1) , RA_v(2));

  temp = Vector2YX(RA_v);

  Right_Elbow_Z_q = EasyKalman(Right_Elbow_Z_q , temp(1));
  Right_Elbow_X_q = EasyKalman(Right_Elbow_X_q , temp(0));

  ROS_INFO(" The Right_Elbow_Z_q is [%f] " ,  Right_Elbow_Z_q);
  ROS_INFO(" The Right_Elbow_X_q is [%f] " ,  Right_Elbow_X_q);


  temp = Matrix2YZX( RH_m);

  Right_Wrist_Z_q = EasyKalman(Right_Wrist_Z_q , temp(1));
  Right_Wrist_Y_q = EasyKalman(Right_Wrist_Y_q , temp(2));
  Right_Wrist_X_q = EasyKalman(Right_Wrist_X_q , temp(0));

  ROS_INFO(" The Right_Wrist_Z_q is [%f] " , Right_Wrist_Z_q);
  ROS_INFO(" The Right_Wrist_Y_q is [%f] " , Right_Wrist_Y_q);
  ROS_INFO(" The Right_Wrist_X_q is [%f] " , Right_Wrist_X_q);



  temp = FingerVector2YX( R_T_v);
  ROS_INFO( " THE R_T_v is [%f] [%f] [%f]" , R_T_v(0) , R_T_v(1) , R_T_v(2));
  R_T_q = EasyKalman(R_T_q , temp(1));

  ROS_INFO("  The Right_index is [%f] ", R_T_q);

  temp = FingerVector2YX(R_I_v);
  ROS_INFO( " THE R_I_v is [%f] [%f] [%f]" , R_I_v(0) , R_I_v(1) , R_I_v(2));
  R_I_q = EasyKalman(R_I_q , temp(1));

  ROS_INFO("  The Right_index is [%f] ", R_I_q);

  temp = FingerVector2YX( R_M_v);
  ROS_INFO( " THE L_M_v is [%f] [%f] [%f]" , R_M_v(0) , R_M_v(1) , R_M_v(2));
  R_M_q = EasyKalman(R_M_q , temp(1));

  ROS_INFO("  The Right_index is [%f] ", R_M_q);

  temp = FingerVector2YX( R_R_v);
  ROS_INFO( " THE R_R_v is [%f] [%f] [%f]" , R_R_v(0) , R_R_v(1) , R_R_v(2));
  R_R_q = EasyKalman(R_R_q , temp(1));

  ROS_INFO("  The Right_index is [%f] ", R_R_q);

  temp = FingerVector2YX( R_P_v);
  ROS_INFO( " THE L_P_v is [%f] [%f] [%f]" , R_P_v(0) , R_P_v(1) , R_P_v(2));
  R_P_q = EasyKalman(R_P_q , temp(1));

  ROS_INFO("  The Right_index is [%f] ", R_P_q);



  // temp = quaternion2XYZ( L_T_qq.w(), L_T_qq.x(), L_T_qq.y(), L_T_qq.z());

  // L_T_q = temp(1);

  // ROS_INFO("  The Left_index is [%f] ", L_T_q);

  // temp = quaternion2XYZ( L_I_qq.w(), L_I_qq.x(), L_I_qq.y(), L_I_qq.z());

  // L_I_q = temp(0);

  // ROS_INFO("  The Left_index is [%f] ", L_I_q);

  // temp = quaternion2XYZ( L_M_qq.w(), L_M_qq.x(), L_M_qq.y(), L_M_qq.z());

  // L_M_q = temp(0);

  // ROS_INFO("  The Left_index is [%f] ", L_M_q);


  // temp = quaternion2XYZ( L_R_qq.w(), L_R_qq.x(), L_R_qq.y(), L_R_qq.z());

  // L_R_q = temp(0);

  // ROS_INFO("  The Left_index is [%f] ", L_R_q);

  // temp = quaternion2XYZ( L_P_qq.w(), L_P_qq.x(), L_P_qq.y(), L_P_qq.z());

  // L_P_q = temp(0);

  // ROS_INFO("  The Left_index is [%f] ", L_P_q);




  ROS_INFO(" The LS Vector is [%f] [%f] [%f]" , LS_v(0) , LS_v(1) , LS_v(2));
  temp = Vector2XZ(LS_v);

  Left_ShoulderX_q = EasyKalman(Left_ShoulderX_q , temp(0));
  Left_ShoulderY_q = EasyKalman(Left_ShoulderY_q , temp(2));


  ROS_INFO(" The Left_ShoulderX_q is [%f] " , Left_ShoulderX_q);
  ROS_INFO(" The Left_ShoulderY_q is [%f] " , Left_ShoulderY_q);



  // temp = quaternion2ZYX( LA_q.w(), LA_q.x(), LA_q.y(), LA_q.z());

  // Left_Elbow_Z_q = temp(1);
  // Left_Elbow_X_q = temp(2);

  ROS_INFO(" The LA Vector is [%f] [%f] [%f]" , LA_v(0) , LA_v(1) , LA_v(2));

  temp = Vector2YX(LA_v);

  Left_Elbow_Z_q = EasyKalman(Left_Elbow_Z_q , temp(1));
  Left_Elbow_X_q = EasyKalman(Left_Elbow_X_q , temp(0));

  ROS_INFO(" The Left_Elbow_Z_q is [%f] " ,  Left_Elbow_Z_q);
  ROS_INFO(" The Left_Elbow_X_q is [%f] " ,  Left_Elbow_X_q);





  // temp = quaternion2ZYX( LH_q.w(), LH_q.x(), LH_q.y(), LH_q.z());

  // Left_Wrist_Z_q = temp(1);
  // Left_Wrist_Y_q = temp(0);
  // Left_Wrist_X_q = temp(2);

  // ROS_INFO(" The Left_Wrist_Z_q is [%f] " , Left_Wrist_Z_q);
  // ROS_INFO(" The Left_Wrist_Y_q is [%f] " , Left_Wrist_Y_q);
  // ROS_INFO(" The Left_Wrist_X_q is [%f] " , Left_Wrist_X_q);



  // Eigen::Quaterniond temp_q = Back_raw.inverse() * LH_raw;
  // ROS_INFO( " THE Quaterion for back is [%f] [%f] [%f] [%f]" , Back_q.w() , Back_q.z() , Back_q.y() , Back_q.z());
  // ROS_INFO( " THE Quaterion for hand is [%f] [%f] [%f] [%f]" , temp_q.w() , temp_q.z() , temp_q.y() , temp_q.z());
  // temp_q = Back_raw.inverse() * LA_raw;
  // ROS_INFO( " THE Quaterion for arms is [%f] [%f] [%f] [%f]" , temp_q.w() , temp_q.z() , temp_q.y() , temp_q.z());



  // Eigen::Matrix3d rot = EulerXYZ(Left_ShoulderX_q , 0 , Left_ShoulderY_q) * EulerZYX(-Left_Elbow_X_q , Left_Elbow_Z_q, 0);


  // ROS_INFO( " THE MATRIX for hand is [%f] [%f] [%f]" , LH_m(0, 0) , LH_m(0, 1) , LH_m(0, 2));
  // ROS_INFO( " THE MATRIX for hand is [%f] [%f] [%f]" , LH_m(1, 0) , LH_m(1, 1) , LH_m(1, 2));
  // ROS_INFO( " THE MATRIX for hand is [%f] [%f] [%f]" , LH_m(2, 0) , LH_m(2, 1) , LH_m(2, 2));


  // ROS_INFO( " THE MATRIX for rot is [%f] [%f] [%f]" , rot(0, 0) , rot(0, 1) , rot(0, 2));
  // ROS_INFO( " THE MATRIX for rot is [%f] [%f] [%f]" , rot(1, 0) , rot(1, 1) , rot(1, 2));
  // ROS_INFO( " THE MATRIX for rot is [%f] [%f] [%f]" , rot(2, 0) , rot(2, 1) , rot(2, 2));



  temp = Matrix2YZX( LH_m);

  Left_Wrist_Z_q = EasyKalman(Left_Wrist_Z_q, temp(1));
  Left_Wrist_Y_q = EasyKalman(Left_Wrist_Y_q, temp(2));
  Left_Wrist_X_q = EasyKalman(Left_Wrist_X_q, temp(0));

  ROS_INFO(" The Left_Wrist_Z_q is [%f] " , Left_Wrist_Z_q);
  ROS_INFO(" The Left_Wrist_Y_q is [%f] " , Left_Wrist_Y_q);
  ROS_INFO(" The Left_Wrist_X_q is [%f] " , Left_Wrist_X_q);



  temp = FingerVector2YX( L_T_v);
  ROS_INFO( " THE L_T_v is [%f] [%f] [%f]" , L_T_v(0) , L_T_v(1) , L_T_v(2));
  L_T_q = EasyKalman(L_T_q , temp(1));

  ROS_INFO("  The Left_index is [%f] ", L_T_q);

  temp = FingerVector2YX( L_I_v);
  ROS_INFO( " THE L_I_v is [%f] [%f] [%f]" , L_I_v(0) , L_I_v(1) , L_I_v(2));
  L_I_q = EasyKalman(L_I_q , temp(1));

  ROS_INFO("  The Left_index is [%f] ", L_I_q);

  temp = FingerVector2YX( L_M_v);
  ROS_INFO( " THE L_M_v is [%f] [%f] [%f]" , L_M_v(0) , L_M_v(1) , L_M_v(2));
  L_M_q = EasyKalman(L_M_q , temp(1));

  ROS_INFO("  The Left_index is [%f] ", L_M_q);


  temp = FingerVector2YX( L_R_v);
  ROS_INFO( " THE L_R_v is [%f] [%f] [%f]" , L_R_v(0) , L_R_v(1) , L_R_v(2));
  L_R_q = EasyKalman(L_R_q , temp(1));

  ROS_INFO("  The Left_index is [%f] ", L_R_q);

  temp = FingerVector2YX( L_P_v);
  ROS_INFO( " THE L_P_v is [%f] [%f] [%f]" , L_P_v(0) , L_P_v(1) , L_P_v(2));
  L_P_q = EasyKalman(L_P_q , temp(1));

  ROS_INFO("  The Left_index is [%f] ", L_P_q);

//   Quaterniond temp_p = Back_init.inverse() * Back_raw;
//   temp = quaternion2ZYX( temp_p.w(), temp_p.x(), temp_p.y(), temp_p.z());
//   ROS_INFO( " The 01 Angles are [%f] [%f] [%f] [%f] " , temp_p.w() , temp_p.x() , temp_p.y() ,temp_p.z());



//   temp_p = LS_init.inverse() * LS_raw;
//   temp = quaternion2ZYX( temp_p.w(), temp_p.x(), temp_p.y(), temp_p.z());
//   ROS_INFO( "  The 02 Angles are [%f] [%f] [%f] [%f] " , LS_q.w() ,LS_q.x() , LS_q.y() , LS_q.z());

//   temp_p = LA_init.inverse() * LA_raw;
//   temp = quaternion2ZYX( temp_p.w(), temp_p.x(), temp_p.y(), temp_p.z());
// //    qDebug() << " The 03 Angles are " << temp(0) << " " << temp(1) << " " << temp(2);
//   ROS_INFO( "  The 03 Angles are [%f] [%f] [%f] [%f] " , LA_q.w() , LA_q.x() , LA_q.y() , LA_q.z());



  if (ui_.tabWidget->currentIndex() == 7) {

    std::vector<double> imu_joint_angles;

    for (int i = 0 ; i < 7 ; i++)
      imu_joint_angles.push_back(0.0);

    imu_joint_angles.push_back(-Left_ShoulderX_q);
    imu_joint_angles.push_back(Left_ShoulderY_q);
    imu_joint_angles.push_back(Left_Elbow_Z_q);
    imu_joint_angles.push_back(Left_Elbow_X_q);
    imu_joint_angles.push_back(Left_Wrist_Z_q);
    imu_joint_angles.push_back(Left_Wrist_Y_q);
    imu_joint_angles.push_back(-Left_Wrist_X_q);

    imu_joint_angles.push_back(-Right_ShoulderX_q);
    imu_joint_angles.push_back(Right_ShoulderY_q);
    imu_joint_angles.push_back(Right_Elbow_Z_q);
    imu_joint_angles.push_back(Right_Elbow_X_q);
    imu_joint_angles.push_back(Right_Wrist_Z_q);
    imu_joint_angles.push_back(Right_Wrist_Y_q);
    imu_joint_angles.push_back(-Right_Wrist_X_q);

    JointTargetPositionPublisher.publish(ConvertJointAnglesMsgs(imu_joint_angles));



    double imu_hand_angles[5];

    imu_hand_angles[0] = L_T_q / ThumbGearRatio;
    imu_hand_angles[1] = L_I_q / FigureGearRatios;
    imu_hand_angles[2] = L_M_q / FigureGearRatios;
    imu_hand_angles[3] = L_R_q / FigureGearRatios;
    imu_hand_angles[4] = L_P_q / FigureGearRatios;

    LeftHandJointTargetPositionPublisher.publish(ConvertHandJointAngleMsgs(imu_hand_angles));


    imu_hand_angles[0] = R_T_q / ThumbGearRatio;
    imu_hand_angles[1] = R_I_q / FigureGearRatios;
    imu_hand_angles[2] = R_M_q / FigureGearRatios;
    imu_hand_angles[3] = R_R_q / FigureGearRatios;
    imu_hand_angles[4] = R_P_q / FigureGearRatios;

    RightHandJointTargetPositionPublisher.publish(ConvertHandJointAngleMsgs(imu_hand_angles));


    if (CollectIMUSwitch) {

      std::vector<double> temp_IMUData;

      temp_IMUData.push_back(Back_raw.w());
      temp_IMUData.push_back(Back_raw.x());
      temp_IMUData.push_back(Back_raw.y());
      temp_IMUData.push_back(Back_raw.z());
      temp_IMUData.push_back(LS_raw.w());
      temp_IMUData.push_back(LS_raw.x());
      temp_IMUData.push_back(LS_raw.y());
      temp_IMUData.push_back(LS_raw.z());
      temp_IMUData.push_back(LA_raw.w());
      temp_IMUData.push_back(LA_raw.x());
      temp_IMUData.push_back(LA_raw.y());
      temp_IMUData.push_back(LA_raw.z());
      temp_IMUData.push_back(LH_raw.w());
      temp_IMUData.push_back(LH_raw.x());
      temp_IMUData.push_back(LH_raw.y());
      temp_IMUData.push_back(LH_raw.z());
      temp_IMUData.push_back(L_T_raw.w());
      temp_IMUData.push_back(L_T_raw.x());
      temp_IMUData.push_back(L_T_raw.y());
      temp_IMUData.push_back(L_T_raw.z());
      temp_IMUData.push_back(L_I_raw.w());
      temp_IMUData.push_back(L_I_raw.x());
      temp_IMUData.push_back(L_I_raw.y());
      temp_IMUData.push_back(L_I_raw.z());
      temp_IMUData.push_back(L_M_raw.w());
      temp_IMUData.push_back(L_M_raw.x());
      temp_IMUData.push_back(L_M_raw.y());
      temp_IMUData.push_back(L_M_raw.z());
      temp_IMUData.push_back(L_R_raw.w());
      temp_IMUData.push_back(L_R_raw.x());
      temp_IMUData.push_back(L_R_raw.y());
      temp_IMUData.push_back(L_R_raw.z());
      temp_IMUData.push_back(L_P_raw.w());
      temp_IMUData.push_back(L_P_raw.x());
      temp_IMUData.push_back(L_P_raw.y());
      temp_IMUData.push_back(L_P_raw.z());

      temp_IMUData.push_back(RS_raw.w());
      temp_IMUData.push_back(RS_raw.x());
      temp_IMUData.push_back(RS_raw.y());
      temp_IMUData.push_back(RS_raw.z());
      temp_IMUData.push_back(RA_raw.w());
      temp_IMUData.push_back(RA_raw.x());
      temp_IMUData.push_back(RA_raw.y());
      temp_IMUData.push_back(RA_raw.z());
      temp_IMUData.push_back(RH_raw.w());
      temp_IMUData.push_back(RH_raw.x());
      temp_IMUData.push_back(RH_raw.y());
      temp_IMUData.push_back(RH_raw.z());
      temp_IMUData.push_back(R_T_raw.w());
      temp_IMUData.push_back(R_T_raw.x());
      temp_IMUData.push_back(R_T_raw.y());
      temp_IMUData.push_back(R_T_raw.z());
      temp_IMUData.push_back(R_I_raw.w());
      temp_IMUData.push_back(R_I_raw.x());
      temp_IMUData.push_back(R_I_raw.y());
      temp_IMUData.push_back(R_I_raw.z());
      temp_IMUData.push_back(R_M_raw.w());
      temp_IMUData.push_back(R_M_raw.x());
      temp_IMUData.push_back(R_M_raw.y());
      temp_IMUData.push_back(R_M_raw.z());
      temp_IMUData.push_back(R_R_raw.w());
      temp_IMUData.push_back(R_R_raw.x());
      temp_IMUData.push_back(R_R_raw.y());
      temp_IMUData.push_back(R_R_raw.z());
      temp_IMUData.push_back(R_P_raw.w());
      temp_IMUData.push_back(R_P_raw.x());
      temp_IMUData.push_back(R_P_raw.y());
      temp_IMUData.push_back(R_P_raw.z());


      temp_IMUData.push_back(-Left_ShoulderX_q);
      temp_IMUData.push_back(Left_ShoulderY_q);
      temp_IMUData.push_back(Left_Elbow_Z_q);
      temp_IMUData.push_back(Left_Elbow_X_q);
      temp_IMUData.push_back(Left_Wrist_Z_q);
      temp_IMUData.push_back(Left_Wrist_Y_q);
      temp_IMUData.push_back(-Left_Wrist_X_q);

      temp_IMUData.push_back(L_T_q);
      temp_IMUData.push_back(L_I_q);
      temp_IMUData.push_back(L_M_q);
      temp_IMUData.push_back(L_R_q);
      temp_IMUData.push_back(L_P_q);

      temp_IMUData.push_back(-Right_ShoulderX_q);
      temp_IMUData.push_back(Right_ShoulderY_q);
      temp_IMUData.push_back(Right_Elbow_Z_q);
      temp_IMUData.push_back(Right_Elbow_X_q);
      temp_IMUData.push_back(Right_Wrist_Z_q);
      temp_IMUData.push_back(Right_Wrist_Y_q);
      temp_IMUData.push_back(-Right_Wrist_X_q);

      temp_IMUData.push_back(R_T_q);
      temp_IMUData.push_back(R_I_q);
      temp_IMUData.push_back(R_M_q);
      temp_IMUData.push_back(R_R_q);
      temp_IMUData.push_back(R_P_q);


      IMUData.push_back(temp_IMUData);

    }

  }

}


Eigen::Vector3d MyPlugin::quaternion2ZYX(double qw, double qx, double qy, double qz) {

  double aSinInput = -2 * (qx * qz - qw * qy);
  if (aSinInput > 1) aSinInput = 1;
  if (aSinInput < -1) aSinInput = -1;

  Eigen::Vector3d res(atan2( 2 * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz ),
                      asin( aSinInput ),
                      atan2( 2 * (qy * qz + qw * qx), qw * qw - qx * qx - qy * qy + qz * qz ));

  return res;
}


Eigen::Vector3d MyPlugin::quaternion2XYZ(double qw, double qx, double qy, double qz) {

  double aSinInput = 2 * (qx * qz + qy * qw);
  if (aSinInput > 1) aSinInput = 1;
  if (aSinInput < -1) aSinInput = -1;

  Eigen::Vector3d res( atan2( -2 * (qy * qz - qx * qw), qw * qw - qx * qx - qy * qy + qz * qz ),
                       asin( aSinInput ),
                       atan2( -2 * (qx * qy - qz * qw), qw * qw + qx * qx - qy * qy - qz * qz ));

  return res;
}

Eigen::Vector3d MyPlugin::Matrix2ZYX(Eigen::Matrix3d input) {

  Eigen::Vector3d res( atan2( input(2, 1) , input(2, 2)),
                       asin(  input(1, 0)),
                       atan2( input(1, 0) , input(0, 0)));

  return res;
}


Eigen::Vector3d MyPlugin::Matrix2YZX(Eigen::Matrix3d input) {

  Eigen::Vector3d res( atan2( -input(1, 2) , input(1, 1)),
                       atan2( -input(2, 0) , input(0, 0)),
                       asin(  input(1, 0)));

  return res;
}


Eigen::Matrix3d MyPlugin::EulerXYZ(double x , double y , double z ) {
  Eigen::Matrix3d res;
  ;
  res <<              cos(y)*cos(z),                       -cos(y)*sin(z),         sin(y),
      cos(x)*sin(z) + cos(z)*sin(x)*sin(y), cos(x)*cos(z) - sin(x)*sin(y)*sin(z), -cos(y)*sin(x),
      sin(x)*sin(z) - cos(x)*cos(z)*sin(y), cos(z)*sin(x) + cos(x)*sin(y)*sin(z),  cos(x)*cos(y);

  return res;
}



Eigen::Matrix3d MyPlugin::EulerZYX(double x , double y , double z ) {
  Eigen::Matrix3d res;
  res << cos(y)*cos(z), cos(z)*sin(x)*sin(y) - cos(x)*sin(z), sin(x)*sin(z) + cos(x)*cos(z)*sin(y),
      cos(y)*sin(z), cos(x)*cos(z) + sin(x)*sin(y)*sin(z), cos(x)*sin(y)*sin(z) - cos(z)*sin(x),
      -sin(y),                        cos(y)*sin(x),                        cos(x)*cos(y);
  return res;
}



Eigen::Vector3d MyPlugin::Vector2XZ(Eigen::Vector3d v) {
  Eigen::Vector3d res = Eigen::VectorXd::Zero(3);

  res(0) = atan2(v(2) , v(1));


  res(2) = - asin(v(0) / v.norm());

  if (abs(abs(res(2)) - PI / 2.0) < 0.03)
    if (abs(v(2)) < 0.03 && abs(v(1) < 0.03))
      res(0) = 0;

  return res;

}


Eigen::Vector3d MyPlugin::Vector2YX(Eigen::Vector3d v) {
  Eigen::Vector3d res = Eigen::VectorXd::Zero(3);

  res(1) = atan2(v(0) , v(2));

  res(0) = asin(v(1) / v.norm()) - PI / 2;

  if (abs(res(0) ) < 0.03) res(1) = 0;
  return res;
}



Eigen::Vector3d MyPlugin::FingerVector2YX(Eigen::Vector3d v) {
  Eigen::Vector3d res = Eigen::VectorXd::Zero(3);

  double v2 = (v(2) > 0) ? v(2) : 0.0;

  res(1) = atan2(v(2) , v(0));

  res(0) = asin(v(1) / v.norm()) - PI / 2;

  if (res(1) < -1.5) res(1) = 3.0;

  return res;
}

double MyPlugin::EasyKalman(double u, double v) {

  static double filter_ratio = 0.5;
  static double filter_error = 1.0 - filter_ratio;

  return  u * filter_ratio + v * filter_error;
}

} // namespace


PLUGINLIB_DECLARE_CLASS(vrep_test, MyPlugin, vrep_test::MyPlugin, rqt_gui_cpp::Plugin)
