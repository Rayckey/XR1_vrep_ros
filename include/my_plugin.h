#ifndef vrep_test_my_plugin_H
#define vrep_test_my_plugin_H

#include <rqt_gui_cpp/plugin.h>
#include <vrep_test/ui_my_plugin.h>

#include <QWidget>
#include "ros/ros.h"
#include "vrep_test/JointAngles.h"
#include "vrep_test/IK_msg.h"
#include "vrep_test/HandJointAngles.h"
#include "vrep_test/JointVisualization.h"
#include <QVector>
#include <QTimer>
#include "std_msgs/Bool.h"
#include "XR1.h"
#include <tf2_ros/transform_broadcaster.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>

#include <opencv2/core/core.hpp>


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
  virtual void onSteeringValueChanged(int);
  virtual void onInertiaParaClicked();
  virtual void onZero();



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


private slots:
  void onStartButtonClicked();
  void onStopButtonClicked();
  void onPauseButtonClicked();
  void JointCurrentPositionRefresher();

}; //class
} // namespace
#endif // my_namespace__my_plugin_H
