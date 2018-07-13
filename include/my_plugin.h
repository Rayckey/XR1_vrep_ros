#ifndef vrep_test_my_plugin_H
#define vrep_test_my_plugin_H

#include <rqt_gui_cpp/plugin.h>
#include <vrep_test/ui_my_plugin.h>

#include <QWidget>
#include <QMap>
#include "ros/ros.h"
#include "vrep_test/JointAngles.h"
#include "vrep_test/IK_msg.h"
#include "vrep_test/HandJointAngles.h"
#include "vrep_test/JointVisualization.h"
#include "vrep_test/JointCurrent.h"
#include <QVector>
#include <QTimer>
#include "std_msgs/Bool.h"
#include "XR1.h"
#include <tf2_ros/transform_broadcaster.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>

#include <opencv2/core/core.hpp>



#include <QStringList>
#include "ros/ros.h"
#include "vrep_test/JointAngles.h"
#include "vrep_test/IK_msg.h"
#include "vrep_test/HandJointAngles.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "vrep_test/InertiaPara.h"
#include "XR1.h"
#include "geometry_msgs/Twist.h"
#include <QMessageBox>
#include <QLabel>
#include <QGroupBox>
#include <QFile>
#include <QTime>
#include <QTextStream>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "rosbag/bag.h"
#include <tf2_ros/transform_broadcaster.h>
#include <QListWidget>
#include <QFileDialog>
#include <QXmlStreamWriter>
#include <QXmlStreamReader>


namespace vrep_test {

class MyPlugin
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  MyPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
  virtual void subscribeJointCurrentPosition(const vrep_test::JointAngles& msg);

  virtual vrep_test::JointAngles ConvertJointAnglesMsgs(std::vector<double> targetPosition);
  virtual vrep_test::IK_msg ConvertIkMsgs(std::vector<double> IKtargetPosition);
  virtual vrep_test::HandJointAngles ConvertHandJointAngleMsgs(double HandPosition[5]);

  // virtual std::vector<double> processCurrents();

  void just_timer_callback();
  // Comment in to signal that the plugin has a way to configure it
  //bool hasConfiguration() const;
  //void triggerConfiguration();

protected:
  void callbackImage(const sensor_msgs::Image::ConstPtr& msg);
  void invertPixels(int x, int y);
  QList<QString> getTopicList(const QSet<QString>& message_types, const QList<QString>& transports);
  QSet<QString> getTopics(const QSet<QString>& message_types, const QSet<QString>& message_sub_types, const QList<QString>& transports);
  void selectTopic(const QString& topic);
  cv::Mat conversion_mat_;
  int counter;
  double actual_time;
  void delay(int delay_time);

protected slots:
  virtual void sortLabelLists();
  virtual void sortSliderLists();
  virtual void sortSpinBoxLists();
  virtual void updateTopicList();
  virtual void sortHandSliderLists();
  virtual void onTopicChanged(int index);
  virtual void onModeChanged(int index);
  virtual void onIKTargetPositionChanged(double d);
  virtual void onHandJointTargetPositionChanged(int i);
  virtual void onJointTargetPositionChanged(int i);
  virtual void onJointRotationVisualization();
  virtual void onJointRotationVisualizationFinish();
  // virtual void onSteeringValueChanged(int);
  // virtual void onInertiaParaClicked();
  virtual void onZero();
  // virtual void onGenerate_ConfigurationClicked();
  // virtual void onSave_CurrentClicked();
  // virtual void onCollect_CurrentClicked();


//add by lzj
  void onBtnAddClicked();
  void onBtnRemoveClicked();
  void modifyAction();
  void play();
  void selectActionChanged(int nActionIdx);
  void readAction();
  void saveAction();
  void generateActuatorData();
private:
  void clearAction();
  void addAction(std::vector<double> & position, double time, QString actionName);
  void addHandAction(std::vector<double> position);
  void removeAction(int nActionIdx);

  bool playing_switch;


protected:

  // void playcall_back( std::vector<double > start_position ,  std::vector<double> goal_position, 
  //   std::vector<double > start_hand_position ,  std::vector<double> goal_hand_position ,  double time);

private:
  Ui::MyPluginWidget ui_;
  QWidget* widget_;
  ros::Publisher JointTargetPositionPublisher;
  ros::Publisher SimulationStartPublisher;
  ros::Publisher SimulationPausePublisher;
  ros::Publisher SimulationStopPublisher;
  ros::Publisher ModeChangePublisher;
  ros::Publisher IKTargetPositionPublisher;
  ros::Publisher LeftHandJointTargetPositionPublisher;
  ros::Publisher RightHandJointTargetPositionPublisher;
  ros::Publisher JointVisualizationPublisher;
  ros::Publisher TwistPublisher;
  ros::Subscriber JointCurrentPositionSubscriber;
  ros::ServiceClient InertiaParaClient;
  ros::ServiceClient CurrentClient;
  XR1 * ptr_XR1;
  std::vector<double> currentPosition;
  image_transport::Subscriber CameraSubscriber;
  QVector<QLabel *> currentPositionLabels;
  QVector<QLabel *> targetPositionLabels;
  QVector<QSlider *> targetPositionSliders;
  QVector<QVector<QSlider *> > HandPositionSliders;
  QVector<QDoubleSpinBox  *> targetPositionSpinBox;
  QTimer *JointCurrentPositionTimer;
  QString arg_topic_name;
  bool pub_topic_custom_;
  QAction* hide_toolbar_action_;
  int num_gridlines_;
  static double previous;

  std::vector<std::vector<double> > GeneratedConfiguration;
  std::vector<std::vector<double> > CurrentData;

  // std::vector<std::vector<double> > m_cmdValue;


  void setup_main_joint_limit();
  void setup_arm_joint_limit();
  void setup_hand_joint_limit();


  std::vector<double> joint_lower_limit;
  std::vector<double> joint_upper_limit;


  std::vector<double> hand_joint_lower_limit;
  std::vector<double> hand_joint_upper_limit;



  QTimer * Path_Ex_Timer;

  int Path_idx;

  //add by lzj
  QVector<std::vector<double> > m_Actions;
  QVector<double> m_ActionsTimes;
  QVector<std::vector<double> > m_ActionsHands;
  QVector<std::vector<double> > m_ActionsOmni;

  std::vector<double> OmniPositions;



  QTimer * playback_timer;

  void  generateActuatorDataHelper();
  std::vector<double> getHandTargetPositions();
  std::vector<double> getOmniAction();
  void addOmniAction(std::vector<double> OmniAction);

private slots:
  void onStartButtonClicked();
  void onStopButtonClicked();
  void onPauseButtonClicked();
  void JointCurrentPositionRefresher();
  void Path_Ex_Fun();
  void read_saved_path();
  void onDance_ButtonClicked();

  void updateTargetSlider(std::vector<double> v , std::vector<double> u);

  double tinyBezier(double double_index , double pt_s , double pt_1 , double pt_2 , double pt_e);


}; //class
} // namespace
#endif // my_namespace__my_plugin_H
