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

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "actuatorcontroller.h"


using namespace Eigen;

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
  ros::Publisher SyncPublisher;
  ros::Publisher NextStepPublisher;
  ros::Subscriber SyncFinishedSubscriber;
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



  ActuatorController * ptr_AC;
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


  Quaterniond R_I_init;
  Quaterniond R_T_init;
  Quaterniond R_R_init;
  Quaterniond R_P_init;
  Quaterniond R_M_init;


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


  Quaterniond R_I_raw;
  Quaterniond R_T_raw;
  Quaterniond R_R_raw;
  Quaterniond R_P_raw;
  Quaterniond R_M_raw;



  Quaterniond LS_q;
  Quaterniond LA_q;
  Quaterniond LH_q;
  Quaterniond Back_q;

  Quaterniond RS_q;
  Quaterniond RA_q;
  Quaterniond RH_q;
  Quaterniond Waist_q;

  Eigen::Matrix3d LH_m;
  Eigen::Matrix3d RH_m;

  Quaterniond L_I_qq;
  Quaterniond L_T_qq;
  Quaterniond L_R_qq;
  Quaterniond L_P_qq;
  Quaterniond L_M_qq;


  Quaterniond R_I_qq;
  Quaterniond R_T_qq;
  Quaterniond R_R_qq;
  Quaterniond R_P_qq;
  Quaterniond R_M_qq;

  double Left_ShoulderX_q;
  double Left_ShoulderY_q;
  double Left_Elbow_Z_q;
  double Left_Elbow_X_q;
  double Left_Wrist_Y_q;
  double Left_Wrist_X_q;
  double Left_Wrist_Z_q;

  double L_I_q;
  double L_T_q;
  double L_R_q;
  double L_P_q;
  double L_M_q;

  double Right_ShoulderX_q;
  double Right_ShoulderY_q;
  double Right_Elbow_Z_q;
  double Right_Elbow_X_q;
  double Right_Wrist_Y_q;
  double Right_Wrist_X_q;
  double Right_Wrist_Z_q;

  double R_I_q;
  double R_T_q;
  double R_R_q;
  double R_P_q;
  double R_M_q;



  Vector3d LS_v;
  Vector3d LA_v;
  Vector3d LH_v;

  Vector3d L_T_v;
  Vector3d L_I_v;
  Vector3d L_M_v;
  Vector3d L_R_v;
  Vector3d L_P_v;

  Vector3d RS_v;
  Vector3d RA_v;
  Vector3d RH_v;

  Vector3d R_T_v;
  Vector3d R_I_v;
  Vector3d R_M_v;
  Vector3d R_R_v;
  Vector3d R_P_v;



  double FigureGearRatios;
  double ThumbGearRatio;
  QTimer * ConvertTimer;

  bool CollectIMUSwitch;

  std::vector<std::vector<double> > IMUData;

private slots:
  void onStartButtonClicked();
  void onStopButtonClicked();
  void onPauseButtonClicked();
  void JointCurrentPositionRefresher();
  void Path_Ex_Fun();
  void read_saved_path();
  void onDance_ButtonClicked();

  void updateTargetSlider(std::vector<double> v , std::vector<double> u);


  void quaterion2joint();

  void quatimercallback();

  void quaternioncallback(uint8_t id , double w, double x, double y , double z );

  Eigen::Vector3d quaternion2ZYX(double w, double qx, double qy , double qz );

  Eigen::Vector3d quaternion2XYZ(double w, double x, double qy , double qz );


  Eigen::Matrix3d EulerXYZ(double x , double y , double z ) ;

  Eigen::Matrix3d EulerZYX(double x , double y , double z ) ;

  Eigen::Vector3d Matrix2ZYX(Eigen::Matrix3d input);
  Eigen::Vector3d Matrix2YZX(Eigen::Matrix3d input) ;
  Eigen::Vector3d Vector2XZ(Eigen::Vector3d v);
  Eigen::Vector3d Vector2YX(Eigen::Vector3d v);
  Eigen::Vector3d FingerVector2YX(Eigen::Vector3d v);


  void on_CollectIMU_clicked();

  void on_SaveIMU_clicked();


  // void on_Sync_Clicked();

  // void syncFinishedCallback();

  // void triggerNextStep();

  void on_InitIMU_clicked();

  double EasyKalman(double u, double v);

}; //class
} // namespace
#endif // my_namespace__my_plugin_H
