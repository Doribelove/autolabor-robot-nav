#ifndef AUTOLABOR_OPERATOR_GUI_MAIN_WINDOW_H
#define AUTOLABOR_OPERATOR_GUI_MAIN_WINDOW_H

#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <autolabor_canbus_driver/CanBusMessage.h>
#include <autolabor_fod_msgs/FodDetectionArray.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <QFutureWatcher>
#include <QImage>
#include <QMainWindow>
#include <QProcess>
#include <QString>
#include <QStringList>
#include <QTimer>

#include <cstdint>
#include <deque>
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
  bool odom_received = false;
  nav_msgs::Odometry odom;
  ros::WallTime odom_received_at;
  double odom_rate_hz = 0.0;
  std::uint64_t odom_message_count = 0;
  double recent_pose_step_m = 0.0;
  double recent_yaw_step_deg = 0.0;
  double stationary_drift_m = 0.0;
  double stationary_window_seconds = 0.0;

  bool cloud_received = false;
  std::size_t cloud_point_count = 0;
  ros::WallTime cloud_received_at;
  double cloud_rate_hz = 0.0;
  std::uint64_t cloud_message_count = 0;

  bool imu_received = false;
  ros::WallTime imu_received_at;
  double imu_rate_hz = 0.0;
  std::uint64_t imu_message_count = 0;
  bool imu_values_finite = false;

  bool can_received = false;
  autolabor_canbus_driver::CanBusMessage can;
  ros::WallTime can_received_at;

  bool scan_received = false;
  std::size_t scan_sample_count = 0;
  ros::WallTime scan_received_at;

  bool navigation_received = false;
  actionlib_msgs::GoalStatusArray navigation;
  ros::WallTime navigation_received_at;

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
  void sendForwardRelativeGoal();
  void sendRelativeGoal();
  void cancelNavigation();
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

  struct FastLioHealthResult
  {
    Health health = Health::Idle;
    int score = 0;
    bool critical_streams_ready = false;
    bool tf_ready = false;
    double position_sigma_m = 0.0;
    double yaw_sigma_deg = 0.0;
    QString state;
    QString summary;
    QStringList findings;
  };

  struct OdomHealthSample
  {
    ros::WallTime received_at;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;
    double linear_speed = 0.0;
    double angular_speed = 0.0;
    double pose_step_m = 0.0;
    double yaw_step_deg = 0.0;
  };

  void buildUi();
  QWidget* buildOverviewPage();
  QWidget* buildFastLioPage();
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
  void callSetBoolService(const std::string& service_name, bool enabled,
                          QPushButton* button, const QString& action_name);
  void requestFodMode(bool enabled);
  void requestCameraControls(bool apply_changes);
  void applyCameraControlsResult(const CameraControlsResult& result);
  FastLioHealthResult evaluateFastLioHealth(const TelemetrySnapshot& data) const;
  bool relativeGoalReady(const TelemetrySnapshot& data,
                         const FastLioHealthResult& health,
                         QString* reason = nullptr) const;
  void publishRelativeGoal(double forward_m, double left_m, double delta_yaw_deg,
                           const QString& source_description);
  void updateImageLabel(QLabel* label, const QImage& image,
                        const QString& placeholder);

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void canCallback(const autolabor_canbus_driver::CanBusMessage::ConstPtr& msg);
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void navigationCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
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
  std::deque<OdomHealthSample> odom_health_history_;

  std::unique_ptr<ros::NodeHandle> node_;
  std::unique_ptr<ros::AsyncSpinner> spinner_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber cloud_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber can_subscriber_;
  ros::Subscriber scan_subscriber_;
  ros::Subscriber navigation_subscriber_;
  ros::Subscriber camera_image_subscriber_;
  ros::Subscriber debug_image_subscriber_;
  ros::Subscriber detections_subscriber_;
  ros::Subscriber mode_state_subscriber_;
  ros::Subscriber mode_status_subscriber_;
  ros::Subscriber visual_state_subscriber_;
  ros::Subscriber visual_status_subscriber_;
  ros::Subscriber diagnostics_subscriber_;
  ros::Publisher relative_goal_publisher_;
  ros::Publisher cancel_publisher_;
  mutable tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  bool master_online_ = false;
  bool previous_probe_online_ = false;
  bool ros_interfaces_ready_ = false;
  bool rviz_initialized_ = false;
  bool enable_rviz_ = true;
  std::string navigation_mode_label_ = "FAST_LIO";
  std::string odom_topic_ = "/Odometry";
  std::string cloud_topic_ = "/cloud_registered_body";
  std::string imu_topic_ = "/livox/imu";
  std::string rviz_config_path_;
  std::string rviz_startup_fixed_frame_ = "base_link";
  std::string rviz_navigation_fixed_frame_ = "camera_init";
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
  QPushButton* record_button_ = nullptr;
  QLabel* overview_camera_preview_ = nullptr;
  QLabel* vision_camera_preview_ = nullptr;
  QPlainTextEdit* vision_detections_ = nullptr;
  QDoubleSpinBox* relative_forward_input_ = nullptr;
  QDoubleSpinBox* relative_left_input_ = nullptr;
  QDoubleSpinBox* relative_yaw_input_ = nullptr;
  QPushButton* relative_goal_button_ = nullptr;
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
  bool mode_request_pending_ = false;
  bool camera_request_pending_ = false;
  Health previous_fastlio_health_ = Health::Idle;

  QProcess recorder_;
  bool recorder_error_ = false;
  bool recorder_stop_requested_ = false;
};

}  // namespace autolabor_operator_gui

#endif  // AUTOLABOR_OPERATOR_GUI_MAIN_WINDOW_H
