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
{
  // Constructor is called first before initPlugin function, needless to say.

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


  OmniPositions.push_back(0.0);OmniPositions.push_back(0.0);OmniPositions.push_back(0.0);


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


} // namespace


PLUGINLIB_DECLARE_CLASS(vrep_test, MyPlugin, vrep_test::MyPlugin, rqt_gui_cpp::Plugin)
