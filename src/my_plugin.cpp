#include "my_plugin.h"
#include "ratio_layouted_frame.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QVector>
#include "ros/ros.h"
#include "vrep_test/JointAngles.h"
#include "vrep_test/IK_msg.h"
#include "vrep_test/HandJointAngles.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include <QTimer>
#include <QMessageBox>
#include <QLabel>
#include <QGroupBox>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "rosbag/bag.h"



#define PI 3.141592654
namespace vrep_test {
  
MyPlugin::MyPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("MyPlugin");
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{

  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);

  //Simulation Control buttons set up
  SimulationStartPublisher =  getNodeHandle().advertise<std_msgs::Bool>("/startSimulation",1);
  SimulationPausePublisher =  getNodeHandle().advertise<std_msgs::Bool>("/pauseSimulation",1);
  SimulationStopPublisher =  getNodeHandle().advertise<std_msgs::Bool>("/stopSimulation",1);
  connect(ui_.Start_Simulation, SIGNAL(clicked()),this,SLOT(onStartButtonClicked()));
  connect(ui_.Stop_Simulation, SIGNAL(clicked()),this,SLOT(onStopButtonClicked()));
  connect(ui_.Pause_Simulation, SIGNAL(clicked()),this,SLOT(onPauseButtonClicked()));

  // Get Joint Current Angles Subcribers under control
  // Linking the subcriber and refresher WILL crash the GUI
  sortLabelLists();
  JointCurrentPositionSubscriber = getNodeHandle().subscribe("/JointAngle/Current" , 100 , &MyPlugin::subscribeJointCurrentPosition, this);
  while (currentPosition.size() < currentPositionLabels.size()) currentPosition.push_back(0.0);
  JointCurrentPositionTimer = new QTimer(this);
  connect(JointCurrentPositionTimer, SIGNAL(timeout()), this, SLOT(JointCurrentPositionRefresher()));
  JointCurrentPositionTimer->start(500);

  // Set up Joint Target Position Publsiher
  JointTargetPositionPublisher = getNodeHandle().advertise<vrep_test::JointAngles>("/JointAngle/Target",10);
  sortSliderLists();
  for (int i = 0 ; i < targetPositionSliders.size() ; i++ ) connect(targetPositionSliders[i], SIGNAL(valueChanged(int)), this, SLOT(onJointTargetPositionChanged(int)));
  for (int i = 0 ; i < targetPositionSliders.size() ; i++ ) connect(targetPositionSliders[i], SIGNAL(sliderPressed()), this, SLOT(onJointRotationVisualization()));
  for (int i = 0 ; i < targetPositionSliders.size() ; i++ ) connect(targetPositionSliders[i], SIGNAL(sliderReleased()), this, SLOT(onJointRotationVisualizationFinish()));
  JointVisualizationPublisher = getNodeHandle().advertise<std_msgs::Int32>("/JointVisualization/Signal",1);

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
  ModeChangePublisher = getNodeHandle().advertise<std_msgs::Bool>("/XR1/ModeChange",1);
  connect(ui_.Mode_Combo, SIGNAL(currentIndexChanged(int)), this, SLOT(onModeChanged(int)));

  //IK Mode setup
  sortSpinBoxLists();
  IKTargetPositionPublisher =getNodeHandle().advertise<vrep_test::IK_msg>("/XR1/IK_msg",10);
  for (int i = 0 ; i < targetPositionSpinBox.size() ; i++ ) connect(targetPositionSpinBox[i], SIGNAL(valueChanged(double)), this, SLOT(onIKTargetPositionChanged(double)));


  //Hand Control Set up
  sortHandSliderLists();
  LeftHandJointTargetPositionPublisher = getNodeHandle().advertise<vrep_test::HandJointAngles>("/HandJointAngles/Left/Target",10);
  RightHandJointTargetPositionPublisher = getNodeHandle().advertise<vrep_test::HandJointAngles>("/HandJointAngles/Right/Target",10);
  for (int i = 0 ; i < HandPositionSliders[0].size() ; i++ ) connect(HandPositionSliders[0][i], SIGNAL(valueChanged(int)), this, SLOT(onHandJointTargetPositionChanged(int)));
  for (int i = 0 ; i < HandPositionSliders[1].size() ; i++ ) connect(HandPositionSliders[1][i], SIGNAL(valueChanged(int)), this, SLOT(onHandJointTargetPositionChanged(int)));

  //Steering
  TwistPublisher = getNodeHandle().advertise<geometry_msgs::Twist>("/XR1/Base/cmd",10);
  connect((ui_.Twist_Z),SIGNAL(valueChanged(int)),this,SLOT(onSteeringValueChanged(int)));
  connect((ui_.Twist_Y),SIGNAL(valueChanged(int)),this,SLOT(onSteeringValueChanged(int)));
  connect((ui_.Twist_X),SIGNAL(valueChanged(int)),this,SLOT(onSteeringValueChanged(int)));
}

void MyPlugin::shutdownPlugin()
{
  // TODO unregister all publishers here
  JointTargetPositionPublisher.shutdown();
  SimulationStartPublisher.shutdown();
  SimulationPausePublisher.shutdown();
  SimulationStopPublisher.shutdown();
  IKTargetPositionPublisher.shutdown();
  LeftHandJointTargetPositionPublisher.shutdown();
  RightHandJointTargetPositionPublisher.shutdown();
  ModeChangePublisher.shutdown();
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

void MyPlugin::subscribeJointCurrentPosition(const vrep_test::JointAngles& msg){
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
}

void MyPlugin::JointCurrentPositionRefresher(){
  for (int i = 0; i < currentPosition.size(); i++){
    currentPositionLabels[i]->setText(QString::number(currentPosition[i],'g', 3));
    if(ui_.Mode_Combo->currentIndex() >0) targetPositionSliders[i]->setValue((currentPosition[i] + PI) / PI * 50.0);
  }
}

void MyPlugin::onJointTargetPositionChanged(int i){
  std::vector<double> targetPosition;
  for(int i = 0; i < targetPositionSliders.size(); i++){
    targetPosition.push_back((double)targetPositionSliders[i]->value()/100. * PI * 2. - PI);
    targetPositionLabels[i]->setText(QString::number(targetPosition.back(),'g', 3));
  }
  if (ui_.Mode_Combo->currentIndex() == 0 )JointTargetPositionPublisher.publish(ConvertJointAnglesMsgs(targetPosition));
}

vrep_test::JointAngles MyPlugin::ConvertJointAnglesMsgs(std::vector<double> targetPosition){

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

vrep_test::IK_msg MyPlugin::ConvertIkMsgs(std::vector<double> IKtargetPosition){

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

vrep_test::HandJointAngles MyPlugin::ConvertHandJointAngleMsgs(double HandPosition[5]){
  vrep_test::HandJointAngles msg;
  msg.Thumb = HandPosition[0];
  msg.Index = HandPosition[1];
  msg.Middle = HandPosition[2];
  msg.Ring = HandPosition[3];
  msg.Pinky = HandPosition[4];
  return msg;
}

void MyPlugin::sortLabelLists(){
  while(!currentPositionLabels.isEmpty())  currentPositionLabels.pop_back();
  for(int i = 0 ; i < 21; i++) currentPositionLabels.append(ui_.Current_Group->findChild<QLabel *>("Joint_Current_Label_"+QString::number(i)));

  while(!targetPositionLabels.isEmpty())  targetPositionLabels.pop_back();
  for(int i = 0 ; i < 21; i++) targetPositionLabels.append(ui_.Target_Group->findChild<QLabel *>("Joint_Target_Label_"+QString::number(i)));
} 

void MyPlugin::sortSliderLists(){
  while(!targetPositionSliders.isEmpty())  targetPositionSliders.pop_back();
  for(int i = 0 ; i < 7; i++) targetPositionSliders.append(ui_.Position_Control_Group->findChild<QSlider *>("Body_T_S_"+QString::number(i)));
  for(int i = 0 ; i < 7; i++) targetPositionSliders.append(ui_.Position_Control_Group->findChild<QSlider *>("Left_Arm_T_S_"+QString::number(i)));
  for(int i = 0 ; i < 7; i++) targetPositionSliders.append(ui_.Position_Control_Group->findChild<QSlider *>("Right_Arm_T_S_"+QString::number(i)));
}

void MyPlugin::sortSpinBoxLists(){
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

void MyPlugin::sortHandSliderLists(){
  QVector<QSlider *> temp;
  temp.append(ui_.LHT);
  temp.append(ui_.LHI);
  temp.append(ui_.LHM);
  temp.append(ui_.LHR);
  temp.append(ui_.LHP);
  HandPositionSliders.append(temp);
  while(!temp.isEmpty()) temp.removeLast();
  temp.append(ui_.RHT);
  temp.append(ui_.RHI);
  temp.append(ui_.RHM);
  temp.append(ui_.RHR);
  temp.append(ui_.RHP);
  HandPositionSliders.append(temp);
}

void MyPlugin::onStartButtonClicked(){
  std_msgs::Bool data;
  data.data = true;
  SimulationStartPublisher.publish(data);

   updateTopicList();
}

void MyPlugin::onStopButtonClicked(){
  std_msgs::Bool data;
  data.data = true;
  SimulationStopPublisher.publish(data);
  ui_.image_frame->setImage(QImage());
}

void MyPlugin::onPauseButtonClicked(){
  std_msgs::Bool data;
  data.data = true;
  SimulationPausePublisher.publish(data);
}

void MyPlugin::callbackImage(const sensor_msgs::Image::ConstPtr& msg){
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
        cv::Mat(cv_ptr->image-min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));
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

QList<QString> MyPlugin::getTopicList(const QSet<QString>& message_types, const QList<QString>& transports){
  QSet<QString> message_sub_types;
  return getTopics(message_types, message_sub_types, transports).values();
}

QSet<QString> MyPlugin::getTopics(const QSet<QString>& message_types, const QSet<QString>& message_sub_types, const QList<QString>& transports){
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

void MyPlugin::selectTopic(const QString& topic){
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

void MyPlugin::updateTopicList(){
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

void MyPlugin::onTopicChanged(int index){
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

void MyPlugin::onModeChanged(int index){

  std_msgs::Bool mode;
  (index >0) ? mode.data = true : mode.data = false;
  for(int i = 0 ; i< targetPositionSliders.size(); i++) targetPositionSliders[i]->setEnabled(!mode.data);
  ModeChangePublisher.publish(mode);
  // onMousePublish(ui_.publish_click_location_check_box->isChecked());
}

void MyPlugin::onIKTargetPositionChanged(double d){

  std::vector<double> IKtargetPosition;
  for(int i = 0; i < targetPositionSpinBox.size(); i++){
      IKtargetPosition.push_back(targetPositionSpinBox[i]->value());
  }

  IKTargetPositionPublisher.publish(ConvertIkMsgs(IKtargetPosition));
}

void MyPlugin::onHandJointTargetPositionChanged(int i){

  double temp[5];
  for(int i = 0; i < 5; i++){
      temp[i] = (double)HandPositionSliders[0][i]->value() /200 * PI;
  }
  LeftHandJointTargetPositionPublisher.publish(ConvertHandJointAngleMsgs(temp));
  for(int i = 0; i < 5; i++){
      temp[i] = (double)HandPositionSliders[1][i]->value() /200 * PI;
  }
  RightHandJointTargetPositionPublisher.publish(ConvertHandJointAngleMsgs(temp));

}


void MyPlugin::onJointRotationVisualization(){
  QSlider * slider = (QSlider *) sender();
  std_msgs::Int32 msg;
  msg.data = -1;
    for(int i = 0; i< targetPositionSliders.size(); i++){
      if(targetPositionSliders[i] == slider) msg.data = i;
    }

  if(msg.data>=0) JointVisualizationPublisher.publish(msg);
}

void MyPlugin::onJointRotationVisualizationFinish(){
  QSlider * slider = (QSlider *) sender();
  std_msgs::Int32 msg;
  msg.data = -1;
  JointVisualizationPublisher.publish(msg); 
}


void MyPlugin::onSteeringValueChanged(int){
  geometry_msgs::Twist msg;
  msg.linear.x = ((double)ui_.Twist_X->value() - 50.)*0.005;
  msg.linear.y = ((double)ui_.Twist_Y->value() - 50.)*0.005;
  msg.angular.z = ((double)ui_.Twist_Z->value() - 50.)*0.05;

  TwistPublisher.publish(msg);
}
  

} // namespace


PLUGINLIB_DECLARE_CLASS(vrep_test, MyPlugin, vrep_test::MyPlugin, rqt_gui_cpp::Plugin)