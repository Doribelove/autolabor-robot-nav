#ifndef AUTOLABOR_OPERATOR_GUI_MAIN_WINDOW_H
#define AUTOLABOR_OPERATOR_GUI_MAIN_WINDOW_H

#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <autolabor_canbus_driver/CanBusMessage.h>
#include <autolabor_operator_msgs/RabbitMqStatus.h>
#include <autolabor_operator_msgs/RemoteTarget.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <QFutureWatcher>
#include <QMainWindow>
#include <QProcess>
#include <QString>
#include <QTimer>

#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>

class QCloseEvent;
class QFrame;
class QLabel;
class QPlainTextEdit;
class QPushButton;
class QTabWidget;
class QVBoxLayout;

namespace rviz
{
class VisualizationFrame;
}

namespace autolabor_operator_gui
{

struct MasterProbeResult
{
  bool online = false;
  bool has_origin = false;
  double origin_latitude = 0.0;
  double origin_longitude = 0.0;
};

struct TimedScalar
{
  bool received = false;
  double value = 0.0;
  ros::WallTime received_at;
};

struct TelemetrySnapshot
{
  bool fix_received = false;
  sensor_msgs::NavSatFix fix;
  ros::WallTime fix_received_at;

  TimedScalar heading;

  bool odom_received = false;
  nav_msgs::Odometry odom;
  ros::WallTime odom_received_at;

  TimedScalar error_current;
  TimedScalar error_rms;
  TimedScalar error_max;
  TimedScalar error_std_x;
  TimedScalar error_std_y;
  std::string error_summary;
  ros::WallTime error_summary_received_at;

  bool can_received = false;
  autolabor_canbus_driver::CanBusMessage can;
  ros::WallTime can_received_at;

  bool scan_received = false;
  std::size_t scan_sample_count = 0;
  ros::WallTime scan_received_at;

  bool navigation_received = false;
  actionlib_msgs::GoalStatusArray navigation;
  ros::WallTime navigation_received_at;

  bool rabbit_status_received = false;
  autolabor_operator_msgs::RabbitMqStatus rabbit_status;
  ros::WallTime rabbit_status_received_at;

  bool remote_target_received = false;
  autolabor_operator_msgs::RemoteTarget remote_target;
  ros::WallTime remote_target_received_at;
};

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  enum class Health
  {
    Idle,
    Good,
    Warning,
    Bad
  };

  explicit MainWindow(QWidget* parent = nullptr);
  ~MainWindow() override;

protected:
  void closeEvent(QCloseEvent* event) override;

private Q_SLOTS:
  void refreshUi();
  void requestMasterProbe();
  void handleMasterProbeFinished();
  void toggleRvizPanels();
  void sendForwardGoal();
  void cancelNavigation();
  void resetStaticError();
  void publishRabbitTarget();
  void clearRabbitTarget();
  void toggleRecording();
  void handleRecorderFinished(int exit_code, QProcess::ExitStatus exit_status);
  void handleRecorderError(QProcess::ProcessError error);

private:
  struct StatusCard
  {
    QFrame* frame = nullptr;
    QLabel* state = nullptr;
    QLabel* detail = nullptr;
  };

  void buildUi();
  QWidget* buildOverviewPage();
  QWidget* buildGpsPage();
  QWidget* buildRabbitPage();
  QWidget* buildTestPage();
  QWidget* buildPlaceholderPage(const QString& title, const QString& subtitle,
                                const QStringList& planned_items);
  QWidget* buildLogPage();
  QFrame* createStatusCard(const QString& key, const QString& title);
  QLabel* createValueLabel(const QString& initial = QStringLiteral("--"));
  QWidget* createMetricRow(const QString& label, QLabel** value_out,
                           const QString& unit = QString());
  void setStatus(const QString& key, Health health, const QString& state,
                 const QString& detail);
  void appendEvent(const QString& text, bool warning = false);

  void setupRosInterfaces();
  void setupEmbeddedRviz();
  void shutdownRosInterfaces();
  void callTriggerService(const std::string& service_name, QPushButton* button,
                          const QString& action_name);

  void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
  void headingCallback(const std_msgs::Float64::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void errorCurrentCallback(const std_msgs::Float64::ConstPtr& msg);
  void errorRmsCallback(const std_msgs::Float64::ConstPtr& msg);
  void errorMaxCallback(const std_msgs::Float64::ConstPtr& msg);
  void errorStdXCallback(const std_msgs::Float64::ConstPtr& msg);
  void errorStdYCallback(const std_msgs::Float64::ConstPtr& msg);
  void errorSummaryCallback(const std_msgs::String::ConstPtr& msg);
  void canCallback(const autolabor_canbus_driver::CanBusMessage::ConstPtr& msg);
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void navigationCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
  void rabbitStatusCallback(const autolabor_operator_msgs::RabbitMqStatus::ConstPtr& msg);
  void remoteTargetCallback(const autolabor_operator_msgs::RemoteTarget::ConstPtr& msg);

  TelemetrySnapshot snapshot() const;
  static double wallAge(const ros::WallTime& stamp);
  static double yawFromQuaternion(const geometry_msgs::Quaternion& quaternion);
  static QString navigationState(const actionlib_msgs::GoalStatusArray& status);
  static QString ageText(double age_seconds);

  mutable std::mutex snapshot_mutex_;
  TelemetrySnapshot telemetry_;

  std::unique_ptr<ros::NodeHandle> node_;
  std::unique_ptr<ros::AsyncSpinner> spinner_;
  ros::Subscriber fix_subscriber_;
  ros::Subscriber heading_subscriber_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber error_current_subscriber_;
  ros::Subscriber error_rms_subscriber_;
  ros::Subscriber error_max_subscriber_;
  ros::Subscriber error_std_x_subscriber_;
  ros::Subscriber error_std_y_subscriber_;
  ros::Subscriber error_summary_subscriber_;
  ros::Subscriber can_subscriber_;
  ros::Subscriber scan_subscriber_;
  ros::Subscriber navigation_subscriber_;
  ros::Subscriber rabbit_status_subscriber_;
  ros::Subscriber remote_target_subscriber_;
  ros::Publisher goal_publisher_;
  ros::Publisher cancel_publisher_;
  ros::Publisher error_reset_publisher_;

  bool master_online_ = false;
  bool previous_probe_online_ = false;
  bool ros_interfaces_ready_ = false;
  bool rviz_initialized_ = false;
  bool enable_rviz_ = true;
  std::string rviz_config_path_;
  bool has_origin_ = false;
  double origin_latitude_ = 0.0;
  double origin_longitude_ = 0.0;
  QTimer master_probe_timer_;
  QFutureWatcher<MasterProbeResult> master_probe_watcher_;
  QTimer ui_refresh_timer_;

  std::map<QString, StatusCard> status_cards_;
  std::map<QString, QLabel*> values_;
  QTabWidget* tabs_ = nullptr;
  QWidget* rviz_host_ = nullptr;
  QVBoxLayout* rviz_layout_ = nullptr;
  QLabel* rviz_placeholder_ = nullptr;
  rviz::VisualizationFrame* rviz_frame_ = nullptr;
  QPlainTextEdit* overview_events_ = nullptr;
  QPlainTextEdit* log_events_ = nullptr;
  QPushButton* rviz_panels_button_ = nullptr;
  QPushButton* forward_goal_button_ = nullptr;
  QPushButton* rabbit_publish_button_ = nullptr;
  QPushButton* rabbit_clear_button_ = nullptr;
  QPushButton* record_button_ = nullptr;

  QProcess recorder_;
  bool recorder_error_ = false;
  bool recorder_stop_requested_ = false;
};

}  // namespace autolabor_operator_gui

#endif  // AUTOLABOR_OPERATOR_GUI_MAIN_WINDOW_H
