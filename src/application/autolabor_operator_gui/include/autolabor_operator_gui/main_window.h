#ifndef AUTOLABOR_OPERATOR_GUI_MAIN_WINDOW_H
#define AUTOLABOR_OPERATOR_GUI_MAIN_WINDOW_H

#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <autolabor_canbus_driver/CanBusMessage.h>
#include <autolabor_fod_msgs/FodDetectionArray.h>
#include <autolabor_operator_msgs/RabbitMqStatus.h>
#include <autolabor_operator_msgs/RemoteTarget.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <QFutureWatcher>
#include <QImage>
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
class QCheckBox;
class QDoubleSpinBox;
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

struct DiagnosticSnapshot
{
  bool received = false;
  unsigned char level = 0;
  std::string message;
  std::map<std::string, std::string> values;
  ros::WallTime received_at;
};

struct CameraControlsResult
{
  bool success = false;
  QString message;
  bool auto_exposure_gain = true;
  double exposure_percent = 100.0;
  double gain_percent = 100.0;
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

  bool camera_received = false;
  std::size_t camera_width = 0;
  std::size_t camera_height = 0;
  std::string camera_encoding;
  ros::WallTime camera_received_at;
  QImage raw_preview;
  ros::WallTime raw_preview_received_at;

  bool debug_image_received = false;
  QImage debug_image;
  ros::WallTime debug_image_received_at;

  bool detections_received = false;
  autolabor_fod_msgs::FodDetectionArray detections;
  ros::WallTime detections_received_at;
  TimedScalar detection_fps;

  bool mode_state_received = false;
  std::string mode_state;
  ros::WallTime mode_state_received_at;
  bool mode_status_received = false;
  std::string mode_status;
  ros::WallTime mode_status_received_at;

  bool visual_state_received = false;
  std::string visual_state;
  ros::WallTime visual_state_received_at;
  bool visual_status_received = false;
  std::string visual_status;
  ros::WallTime visual_status_received_at;

  DiagnosticSnapshot detector_diagnostic;
  DiagnosticSnapshot image_quality_diagnostic;
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
  void sendManualGpsGoal();
  void useCurrentGpsForGoal();
  void cancelNavigation();
  void resetStaticError();
  void publishRabbitTarget();
  void clearRabbitTarget();
  void toggleRecording();
  void startFodMode();
  void stopFodMode();
  void queryCameraControls();
  void applyCameraControls();
  void enableImageQualityControl();
  void disableImageQualityControl();
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
  QWidget* buildVisionPage();
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
  void callSetBoolService(const std::string& service_name, bool enabled,
                          QPushButton* button, const QString& action_name);
  void requestFodMode(bool enabled);
  void requestCameraControls(bool apply_changes);
  void applyCameraControlsResult(const CameraControlsResult& result);
  bool gpsGoalReady(const TelemetrySnapshot& data, QString* reason = nullptr) const;
  void publishGpsGoal(double latitude, double longitude, double altitude,
                      const QString& source_description);
  void updateImageLabel(QLabel* label, const QImage& image,
                        const QString& placeholder);

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
  void cameraImageCallback(const sensor_msgs::Image::ConstPtr& msg);
  void debugImageCallback(const sensor_msgs::Image::ConstPtr& msg);
  void detectionsCallback(const autolabor_fod_msgs::FodDetectionArray::ConstPtr& msg);
  void modeStateCallback(const std_msgs::String::ConstPtr& msg);
  void modeStatusCallback(const std_msgs::String::ConstPtr& msg);
  void visualStateCallback(const std_msgs::String::ConstPtr& msg);
  void visualStatusCallback(const std_msgs::String::ConstPtr& msg);
  void diagnosticsCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg);

  TelemetrySnapshot snapshot() const;
  static double wallAge(const ros::WallTime& stamp);
  static double yawFromQuaternion(const geometry_msgs::Quaternion& quaternion);
  static QString navigationState(const actionlib_msgs::GoalStatusArray& status);
  static QString ageText(double age_seconds);
  static bool imageMessageToQImage(const sensor_msgs::Image& message, QImage* image);

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
  ros::Subscriber camera_image_subscriber_;
  ros::Subscriber debug_image_subscriber_;
  ros::Subscriber detections_subscriber_;
  ros::Subscriber mode_state_subscriber_;
  ros::Subscriber mode_status_subscriber_;
  ros::Subscriber visual_state_subscriber_;
  ros::Subscriber visual_status_subscriber_;
  ros::Subscriber diagnostics_subscriber_;
  ros::Publisher goal_publisher_;
  ros::Publisher cancel_publisher_;
  ros::Publisher error_reset_publisher_;

  bool master_online_ = false;
  bool previous_probe_online_ = false;
  bool ros_interfaces_ready_ = false;
  bool rviz_initialized_ = false;
  bool enable_rviz_ = true;
  std::string navigation_mode_label_ = "GPS";
  std::string odom_topic_ = "/gps/odom";
  std::string rviz_config_path_;
  std::string rviz_startup_fixed_frame_ = "base_link";
  std::string rviz_navigation_fixed_frame_ = "camera_init";
  bool has_origin_ = false;
  double origin_latitude_ = 0.0;
  double origin_longitude_ = 0.0;
  QTimer master_probe_timer_;
  QFutureWatcher<MasterProbeResult> master_probe_watcher_;
  QTimer ui_refresh_timer_;

  std::map<QString, StatusCard> status_cards_;
  std::map<QString, QLabel*> values_;
  QLabel* app_subtitle_ = nullptr;
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
  QLabel* overview_camera_preview_ = nullptr;
  QLabel* vision_camera_preview_ = nullptr;
  QPlainTextEdit* vision_detections_ = nullptr;
  QDoubleSpinBox* gps_latitude_input_ = nullptr;
  QDoubleSpinBox* gps_longitude_input_ = nullptr;
  QPushButton* manual_goal_button_ = nullptr;
  QPushButton* overview_fod_start_button_ = nullptr;
  QPushButton* overview_fod_stop_button_ = nullptr;
  QPushButton* fod_start_button_ = nullptr;
  QPushButton* fod_stop_button_ = nullptr;
  QCheckBox* exposure_auto_checkbox_ = nullptr;
  QDoubleSpinBox* exposure_input_ = nullptr;
  QDoubleSpinBox* gain_input_ = nullptr;
  QPushButton* camera_query_button_ = nullptr;
  QPushButton* camera_apply_button_ = nullptr;
  QPushButton* image_quality_enable_button_ = nullptr;
  QPushButton* image_quality_disable_button_ = nullptr;

  ros::WallTime last_raw_preview_conversion_;
  ros::WallTime last_debug_preview_conversion_;
  bool manual_goal_initialized_ = false;
  bool mode_request_pending_ = false;
  bool camera_request_pending_ = false;

  QProcess recorder_;
  bool recorder_error_ = false;
  bool recorder_stop_requested_ = false;
};

}  // namespace autolabor_operator_gui

#endif  // AUTOLABOR_OPERATOR_GUI_MAIN_WINDOW_H
