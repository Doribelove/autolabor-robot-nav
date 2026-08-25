#include <autolabor_operator_gui/main_window.h>

#include <dynamic_reconfigure/Reconfigure.h>
#include <autolabor_coverage/PlanCoverage.h>
#include <autolabor_coverage/StartCoverage.h>
#include <rviz/default_plugin/map_display.h>
#include <rviz/display.h>
#include <rviz/display_group.h>
#include <rviz/render_panel.h>
#include <rviz/tool.h>
#include <rviz/tool_manager.h>
#include <rviz/view_controller.h>
#include <rviz/view_manager.h>
#include <rviz/visualization_frame.h>
#include <rviz/visualization_manager.h>

#include <ros/master.h>
#include <ros/package.h>
#include <ros/topic.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <QApplication>
#include <QCheckBox>
#include <QCloseEvent>
#include <QDateTime>
#include <QDir>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QFuture>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMenuBar>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPushButton>
#include <QScrollArea>
#include <QSizePolicy>
#include <QSplitter>
#include <QStatusBar>
#include <QTabWidget>
#include <QTextBrowser>
#include <QPixmap>
#include <QtConcurrent/QtConcurrentRun>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>

namespace autolabor_operator_gui
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kFreshCameraSeconds = 1.5;
constexpr double kFreshDetectionSeconds = 1.5;
constexpr double kFreshModeSeconds = 2.0;
constexpr double kFastLioFreshOdomSeconds = 0.30;
constexpr double kFastLioFreshCloudSeconds = 0.30;
constexpr double kFastLioFreshImuSeconds = 0.10;
constexpr double kFastLioCriticalStreamSeconds = 1.0;
constexpr double kFastLioStationaryWindowSeconds = 5.0;
constexpr double kMapDisplayRefreshIntervalSeconds = 1.0;
constexpr unsigned int kMapDisplayMaxRefreshAttempts = 5;
const char* const kStaticMapDisplayName = "Static global map";
const char* const kPriorMapDisplayName = "Known 3D global map (optional)";
const char* const kTebGlobalPlanDisplayName = "TEB global plan";
const char* const kTebLocalPlanDisplayName = "TEB local plan";

rviz::Display* findDisplayByName(rviz::DisplayGroup* group,
                                 const QString& name)
{
  if (!group)
    return nullptr;
  for (int index = 0; index < group->numDisplays(); ++index)
  {
    rviz::Display* display = group->getDisplayAt(index);
    if (!display)
      continue;
    if (display->getName() == name)
      return display;
    if (auto* nested_group = dynamic_cast<rviz::DisplayGroup*>(display))
    {
      if (rviz::Display* match = findDisplayByName(nested_group, name))
        return match;
    }
  }
  return nullptr;
}

class ScrollSafeDoubleSpinBox final : public QDoubleSpinBox
{
public:
  using QDoubleSpinBox::QDoubleSpinBox;

protected:
  void wheelEvent(QWheelEvent* event) override
  {
    // The controls live inside a scroll area.  Never reinterpret sidebar
    // scrolling as an operating-parameter change; operators can still type a
    // value or use the explicit step buttons.
    event->ignore();
  }
};

const char* const kZedReconfigureService = "/zed2/zed_node/set_parameters";
const char* const kZedParameterUpdatesTopic = "/zed2/zed_node/parameter_updates";
const char* const kImageQualityControlService =
    "/fod_image_quality_controller/set_enabled";
const char* const kFodModeService =
    "/fod_navigation_mode/set_fod_enabled";

struct ModeStatusView
{
  bool valid = false;
  bool navigation_paused = false;
  bool move_base_goals_allowed = false;
  QString state;
  QString visual_state;
  QString reason;
  QString command_source;
};

bool configBool(const dynamic_reconfigure::Config& config, const std::string& name,
                bool* value)
{
  for (const auto& parameter : config.bools)
  {
    if (parameter.name == name)
    {
      *value = parameter.value;
      return true;
    }
  }
  return false;
}

bool configInt(const dynamic_reconfigure::Config& config, const std::string& name,
               int* value)
{
  for (const auto& parameter : config.ints)
  {
    if (parameter.name == name)
    {
      *value = parameter.value;
      return true;
    }
  }
  return false;
}

bool setConfigBool(dynamic_reconfigure::Config* config, const std::string& name,
                   bool value)
{
  for (auto& parameter : config->bools)
  {
    if (parameter.name == name)
    {
      parameter.value = value;
      return true;
    }
  }
  return false;
}

bool setConfigInt(dynamic_reconfigure::Config* config, const std::string& name,
                  int value)
{
  for (auto& parameter : config->ints)
  {
    if (parameter.name == name)
    {
      parameter.value = value;
      return true;
    }
  }
  return false;
}

QString numberOrDash(double value, int precision = 3)
{
  return std::isfinite(value) ? QString::number(value, 'f', precision) : QStringLiteral("--");
}

QString statusColor(MainWindow::Health health)
{
  switch (health)
  {
    case MainWindow::Health::Good:
      return QStringLiteral("#20b47a");
    case MainWindow::Health::Warning:
      return QStringLiteral("#e5a93d");
    case MainWindow::Health::Bad:
      return QStringLiteral("#e45b61");
    case MainWindow::Health::Idle:
    default:
      return QStringLiteral("#778293");
  }
}

QString statusBackground(MainWindow::Health health)
{
  switch (health)
  {
    case MainWindow::Health::Good:
      return QStringLiteral("#173e36");
    case MainWindow::Health::Warning:
      return QStringLiteral("#473c27");
    case MainWindow::Health::Bad:
      return QStringLiteral("#4a2d34");
    case MainWindow::Health::Idle:
    default:
      return QStringLiteral("#2b3442");
  }
}

QString modeDisplayName(const QString& state)
{
  if (state == QStringLiteral("GPS_ACTIVE"))
    return QStringLiteral("相对导航");
  if (state == QStringLiteral("ENTERING_FOD"))
    return QStringLiteral("正在切入视觉");
  if (state == QStringLiteral("FOD_ACTIVE"))
    return QStringLiteral("视觉控制");
  if (state == QStringLiteral("FOD_COMPLETE_STOP"))
    return QStringLiteral("视觉完成停车");
  if (state == QStringLiteral("RETURNING_GPS"))
    return QStringLiteral("正在恢复相对导航");
  if (state == QStringLiteral("FOD_ABORTED"))
    return QStringLiteral("视觉中止停车");
  if (state == QStringLiteral("FAULT_STOP"))
    return QStringLiteral("故障停车");
  return state.isEmpty() ? QStringLiteral("未知") : state;
}

ModeStatusView parseModeStatus(const std::string& json)
{
  ModeStatusView result;
  QJsonParseError error;
  const QJsonDocument document =
      QJsonDocument::fromJson(QByteArray::fromStdString(json), &error);
  if (error.error != QJsonParseError::NoError || !document.isObject())
    return result;
  const QJsonObject object = document.object();
  result.valid = true;
  // The mode arbiter keeps this historical JSON field for wire compatibility.
  // In the indoor console it represents the local move_base route, not GNSS.
  result.navigation_paused =
      object.value(QStringLiteral("gps_paused")).toBool(false);
  result.move_base_goals_allowed =
      object.value(QStringLiteral("move_base_goals_allowed")).toBool(false);
  result.state = object.value(QStringLiteral("state")).toString();
  result.visual_state = object.value(QStringLiteral("visual_state")).toString();
  result.reason = object.value(QStringLiteral("reason")).toString();
  result.command_source = object.value(QStringLiteral("command_source")).toString();
  return result;
}

QString diagnosticValue(const DiagnosticSnapshot& diagnostic, const std::string& key,
                        const QString& fallback = QStringLiteral("--"))
{
  const auto found = diagnostic.values.find(key);
  return found == diagnostic.values.end() ? fallback
                                           : QString::fromStdString(found->second);
}

bool textIsTrue(const QString& value)
{
  return value.compare(QStringLiteral("true"), Qt::CaseInsensitive) == 0 ||
         value == QStringLiteral("1");
}

double updateRateEstimate(double previous_rate, const ros::WallTime& previous_stamp,
                          const ros::WallTime& now)
{
  if (previous_stamp.isZero())
    return previous_rate;
  const double interval = (now - previous_stamp).toSec();
  if (!std::isfinite(interval) || interval < 0.0005 || interval > 2.0)
    return previous_rate;
  const double instantaneous = 1.0 / interval;
  return previous_rate > 0.0 ? previous_rate * 0.85 + instantaneous * 0.15
                             : instantaneous;
}

double angleDistanceRadians(double first, double second)
{
  return std::abs(std::atan2(std::sin(first - second), std::cos(first - second)));
}

}  // namespace

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent)
{
  buildUi();

  connect(&ui_refresh_timer_, &QTimer::timeout, this, &MainWindow::refreshUi);
  ui_refresh_timer_.start(250);

  connect(&master_probe_timer_, &QTimer::timeout, this, &MainWindow::requestMasterProbe);
  connect(&master_probe_watcher_, &QFutureWatcher<MasterProbeResult>::finished, this,
          &MainWindow::handleMasterProbeFinished);
  master_probe_timer_.start(1500);

  connect(&recorder_,
          QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
          &MainWindow::handleRecorderFinished);
  connect(&recorder_, &QProcess::errorOccurred, this, &MainWindow::handleRecorderError);
  connect(&recorder_, &QProcess::readyReadStandardOutput, this, [this]() {
    const QString output = QString::fromLocal8Bit(recorder_.readAllStandardOutput()).trimmed();
    if (!output.isEmpty())
      appendEvent(output);
  });
  connect(&recorder_, &QProcess::readyReadStandardError, this, [this]() {
    const QString output = QString::fromLocal8Bit(recorder_.readAllStandardError()).trimmed();
    if (!output.isEmpty())
      appendEvent(output, true);
  });
  connect(&mapper_,
          QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
          &MainWindow::handleMapperFinished);
  connect(&mapper_, &QProcess::errorOccurred, this, &MainWindow::handleMapperError);
  connect(&mapper_, &QProcess::readyReadStandardOutput, this, [this]() {
    const QString output = QString::fromLocal8Bit(mapper_.readAllStandardOutput()).trimmed();
    if (!output.isEmpty())
      appendEvent(output);
  });
  connect(&mapper_, &QProcess::readyReadStandardError, this, [this]() {
    const QString output = QString::fromLocal8Bit(mapper_.readAllStandardError()).trimmed();
    if (!output.isEmpty())
      appendEvent(output, true);
  });

  appendEvent(QStringLiteral("操作台界面已启动；正在后台探测 ROS master。"));
  requestMasterProbe();
}

MainWindow::~MainWindow()
{
  master_probe_timer_.stop();
  ui_refresh_timer_.stop();
  if (recorder_.state() != QProcess::NotRunning)
  {
    recorder_.terminate();
    if (!recorder_.waitForFinished(60000))
      recorder_.kill();
  }
  if (mapper_.state() != QProcess::NotRunning)
  {
    mapper_.terminate();
    if (!mapper_.waitForFinished(120000))
      mapper_.kill();
  }
  shutdownRosInterfaces();
  if (spinner_)
    spinner_->stop();
}

void MainWindow::closeEvent(QCloseEvent* event)
{
  if (recorder_.state() != QProcess::NotRunning)
  {
    recorder_.terminate();
    if (!recorder_.waitForFinished(60000))
      recorder_.kill();
  }
  if (mapper_.state() != QProcess::NotRunning)
  {
    mapper_.terminate();
    if (!mapper_.waitForFinished(120000))
      mapper_.kill();
  }
  ros::shutdown();
  event->accept();
}

void MainWindow::buildUi()
{
  setWindowTitle(QStringLiteral("Autolabor 无人车操作与诊断台"));
  resize(1680, 1000);
  setMinimumSize(1100, 700);

  setStyleSheet(QStringLiteral(R"(
    QMainWindow { background: #101721; }
    QWidget { color: #e7edf5; font-family: "Noto Sans CJK SC", "Microsoft YaHei", sans-serif; font-size: 13pt; }
    QFrame#topBar { background: #17212e; border-bottom: 1px solid #334154; }
    QLabel#appTitle { font-size: 21pt; font-weight: 700; color: #f3f7fb; }
    QLabel#appSubtitle { color: #8fa0b5; font-size: 11pt; }
    QFrame.statusCard { border-radius: 8px; border: 1px solid #405067; }
    QLabel.statusTitle { color: #aeb9c8; font-size: 11pt; font-weight: 600; }
    QLabel.statusState { font-size: 15pt; font-weight: 700; }
    QLabel.statusDetail { color: #aab5c3; font-size: 10pt; }
    QTabWidget::pane { border: 0; background: #101721; }
    QTabBar::tab { background: #17212e; color: #aeb9c8; min-width: 92px; min-height: 40px; padding: 7px 16px; border-right: 1px solid #263346; font-size: 13pt; }
    QTabBar::tab:selected { background: #243349; color: #ffffff; border-bottom: 4px solid #34a8ff; }
    QTabBar::tab:hover { background: #202d3e; }
    QGroupBox { background: #17212e; border: 1px solid #2b3a4e; border-radius: 8px; margin-top: 22px; padding: 19px 12px 12px 12px; font-size: 14pt; font-weight: 700; color: #dce5f0; }
    QGroupBox::title { subcontrol-origin: margin; left: 14px; padding: 0 6px; }
    QLabel.metricName { color: #8fa0b5; font-size: 12pt; }
    QLabel.metricValue { color: #f1f5f9; font-size: 15pt; font-weight: 650; }
    QPushButton { background: #245d87; color: white; border: 1px solid #347bb0; border-radius: 6px; min-height: 44px; padding: 4px 16px; font-size: 13pt; font-weight: 650; }
    QPushButton:hover { background: #2d72a5; }
    QPushButton:pressed { background: #1d4d71; }
    QPushButton:disabled { background: #293442; color: #6f7b8a; border-color: #3a4655; }
    QPushButton#dangerButton { background: #813a42; border-color: #b34d57; }
    QPushButton#visionButton { background: #94621f; border-color: #c98a32; }
    QPushButton#recordButton { background: #74425d; border-color: #a25b80; }
    QDoubleSpinBox, QLineEdit { background: #0d141d; color: #edf3fa; border: 1px solid #405067; border-radius: 5px; min-height: 38px; padding: 2px 8px; font-size: 12pt; }
    QDoubleSpinBox:disabled, QLineEdit:disabled { color: #6f7b8a; background: #202a37; }
    QCheckBox { color: #c7d2df; spacing: 8px; min-height: 34px; font-size: 12pt; }
    QPlainTextEdit, QTextBrowser { background: #0d141d; border: 1px solid #2c394a; border-radius: 6px; color: #c8d2df; selection-background-color: #245d87; font-size: 12pt; }
    QScrollArea { border: 0; }
    QScrollArea#coverageSide, QWidget#coverageControls { background: #f4f6f8; color: #111827; }
    QWidget#coverageControls QGroupBox { background: #ffffff; color: #111827; border-color: #cbd5e1; }
    QWidget#coverageControls QGroupBox::title { color: #111827; }
    QWidget#coverageControls QLabel, QWidget#coverageControls QCheckBox { color: #111827; }
    QWidget#coverageControls QLabel.metricName { color: #374151; }
    QWidget#coverageControls QLabel.metricValue { color: #111827; }
    QWidget#coverageControls QDoubleSpinBox { background: #ffffff; color: #111827; border-color: #94a3b8; }
    QWidget#coverageControls QDoubleSpinBox:disabled { background: #e5e7eb; color: #6b7280; }
    QToolBar { background: #f5f5f5; color: #111827; border: 0; }
    QToolBar QToolButton { background: transparent; color: #111827; border: 1px solid transparent; }
    QToolBar QToolButton:hover { background: #dce5ee; border-color: #aab8c5; }
    QToolBar QToolButton:disabled { background: transparent; color: #586474; }
    QMenu, QAbstractItemView { background: #ffffff; color: #111827; selection-background-color: #cfe8ff; selection-color: #111827; }
    QDockWidget { color: #111827; }
    QMessageBox { background: #f4f6f8; color: #111827; }
    QMessageBox QLabel { background: transparent; color: #111827; }
    QMessageBox QPushButton { background: #e5e7eb; color: #111827; border: 1px solid #94a3b8; }
    QMessageBox QPushButton:hover { background: #dbeafe; border-color: #60a5fa; }
    QMessageBox QPushButton:pressed { background: #bfdbfe; }
    QMessageBox QPushButton:disabled { background: #e5e7eb; color: #6b7280; border-color: #cbd5e1; }
    QFrame#globalMapControls { background: #17212e; border: 1px solid #334154; border-radius: 6px; }
    QLabel#globalMapInstruction { color: #dce5f0; font-size: 11pt; }
    QPushButton#mapViewButton, QPushButton#initialPoseButton, QPushButton#threeDMapButton { min-height: 36px; font-size: 11pt; padding: 2px 12px; }
    QPushButton#initialPoseButton { background: #287a5a; border-color: #36a876; }
    QPushButton#initialPoseButton:hover { background: #31936c; }
    QPushButton#threeDMapButton { background: #68458d; border-color: #9567bf; }
    QPushButton#threeDMapButton:hover { background: #7b52a5; }
    QPushButton#threeDMapButton:checked { background: #a06a24; border-color: #d89537; }
    QSplitter::handle { background: #263346; width: 5px; height: 5px; }
  )"));

  auto* central = new QWidget(this);
  auto* root = new QVBoxLayout(central);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);

  auto* top_bar = new QFrame(central);
  top_bar->setObjectName(QStringLiteral("topBar"));
  auto* top_layout = new QVBoxLayout(top_bar);
  top_layout->setContentsMargins(20, 10, 20, 12);
  top_layout->setSpacing(8);

  auto* title_box = new QWidget(top_bar);
  auto* title_layout = new QHBoxLayout(title_box);
  title_layout->setContentsMargins(0, 0, 0, 0);
  title_layout->setSpacing(20);
  auto* title = new QLabel(QStringLiteral("AUTOLABOR  操作与诊断台"), title_box);
  title->setObjectName(QStringLiteral("appTitle"));
  app_subtitle_ = new QLabel(QStringLiteral("NAVIGATION · FIELD TEST CONSOLE"), title_box);
  app_subtitle_->setObjectName(QStringLiteral("appSubtitle"));
  title_layout->addWidget(title);
  title_layout->addWidget(app_subtitle_);
  title_layout->addStretch();
  top_layout->addWidget(title_box);

  auto* cards_layout = new QHBoxLayout();
  cards_layout->setContentsMargins(0, 0, 0, 0);
  cards_layout->setSpacing(10);
  const std::pair<const char*, const char*> cards[] = {
    { "ros", "ROS" },         { "can", "CAN" },       { "fastlio", "FAST-LIO" },
    { "cloud", "点云" },      { "imu", "IMU" },       { "scan", "避障雷达" },
    { "nav", "导航" },        { "mode", "控制模式" }, { "camera", "相机" },
    { "yolo", "YOLO11" },     { "record", "录包" }
  };
  for (const auto& card : cards)
    cards_layout->addWidget(createStatusCard(QString::fromLatin1(card.first),
                                             QString::fromUtf8(card.second)), 1);
  top_layout->addLayout(cards_layout);

  root->addWidget(top_bar);

  tabs_ = new QTabWidget(central);
  tabs_->setTabPosition(QTabWidget::North);
  tabs_->setDocumentMode(true);
  overview_tab_index_ = tabs_->addTab(buildOverviewPage(), QStringLiteral("综合"));
  tabs_->addTab(buildFastLioPage(), QStringLiteral("FAST-LIO"));
  tabs_->addTab(buildTestPage(), QStringLiteral("测试"));
  tabs_->addTab(buildVisionPage(), QStringLiteral("视觉"));
  coverage_tab_index_ = tabs_->addTab(buildCoveragePage(), QStringLiteral("清扫"));
  tabs_->addTab(buildLogPage(), QStringLiteral("日志"));
  connect(tabs_, &QTabWidget::currentChanged, this, [this](int index) {
    if (!master_online_ || !ros_interfaces_ready_ || !enable_rviz_)
      return;
    // Qt/Ogre cannot reliably create a native render window below a hidden
    // QTabWidget page on Jetson/X11.  Defer construction until the selected
    // page has completed its visibility/layout event.
    if (index != overview_tab_index_ && index != coverage_tab_index_)
      return;
    QTimer::singleShot(0, this, [this, index]() {
      if (!tabs_ || tabs_->currentIndex() != index || !master_online_ ||
          !ros_interfaces_ready_ || !enable_rviz_)
        return;
      if (index == overview_tab_index_)
        setupEmbeddedRviz();
      else if (index == coverage_tab_index_)
        setupCoverageRviz();
    });
  });
  root->addWidget(tabs_, 1);

  setCentralWidget(central);
}

QFrame* MainWindow::createStatusCard(const QString& key, const QString& title)
{
  auto* frame = new QFrame(this);
  frame->setProperty("class", QStringLiteral("statusCard"));
  frame->setMinimumSize(78, 82);
  auto* layout = new QVBoxLayout(frame);
  layout->setContentsMargins(12, 7, 12, 7);
  layout->setSpacing(1);
  auto* heading = new QLabel(title, frame);
  heading->setProperty("class", QStringLiteral("statusTitle"));
  auto* state = new QLabel(QStringLiteral("未连接"), frame);
  state->setProperty("class", QStringLiteral("statusState"));
  auto* detail = new QLabel(QStringLiteral("等待数据"), frame);
  detail->setProperty("class", QStringLiteral("statusDetail"));
  detail->setTextInteractionFlags(Qt::TextSelectableByMouse);
  layout->addWidget(heading);
  layout->addWidget(state);
  layout->addWidget(detail);
  status_cards_[key] = { frame, state, detail };
  setStatus(key, Health::Idle, QStringLiteral("未连接"), QStringLiteral("等待数据"));
  return frame;
}

QLabel* MainWindow::createValueLabel(const QString& initial)
{
  auto* label = new QLabel(initial, this);
  label->setProperty("class", QStringLiteral("metricValue"));
  label->setTextInteractionFlags(Qt::TextSelectableByMouse);
  label->setWordWrap(true);
  label->setMinimumWidth(0);
  label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  return label;
}

QWidget* MainWindow::createMetricRow(const QString& label, QLabel** value_out,
                                     const QString& unit)
{
  auto* row = new QWidget(this);
  row->setMinimumHeight(44);
  auto* layout = new QHBoxLayout(row);
  layout->setContentsMargins(0, 4, 0, 4);
  layout->setSpacing(8);
  auto* name = new QLabel(label, row);
  name->setProperty("class", QStringLiteral("metricName"));
  auto* value = createValueLabel();
  layout->addWidget(name);
  layout->addWidget(value, 1);
  if (!unit.isEmpty())
  {
    auto* unit_label = new QLabel(unit, row);
    unit_label->setProperty("class", QStringLiteral("metricName"));
    layout->addWidget(unit_label);
  }
  *value_out = value;
  return row;
}

QWidget* MainWindow::buildOverviewPage()
{
  auto* page = new QWidget(this);
  auto* layout = new QVBoxLayout(page);
  layout->setContentsMargins(16, 12, 16, 12);
  layout->setSpacing(0);

  auto* splitter = new QSplitter(Qt::Horizontal, page);
  rviz_host_ = new QWidget(splitter);
  rviz_layout_ = new QVBoxLayout(rviz_host_);
  rviz_layout_->setContentsMargins(0, 0, 0, 0);
  rviz_layout_->setSpacing(6);

  rviz_map_controls_ = new QFrame(rviz_host_);
  rviz_map_controls_->setObjectName(QStringLiteral("globalMapControls"));
  auto* map_controls_layout = new QHBoxLayout(rviz_map_controls_);
  map_controls_layout->setContentsMargins(10, 6, 10, 6);
  map_controls_layout->setSpacing(8);
  rviz_map_instruction_ = new QLabel(
      QStringLiteral("全局地图加载后：先显示全图，再按车辆真实位置设置初始位姿"),
      rviz_map_controls_);
  rviz_map_instruction_->setObjectName(QStringLiteral("globalMapInstruction"));
  rviz_map_instruction_->setWordWrap(true);
  rviz_fit_map_button_ =
      new QPushButton(QStringLiteral("① 显示整张地图"), rviz_map_controls_);
  rviz_fit_map_button_->setObjectName(QStringLiteral("mapViewButton"));
  rviz_fit_map_button_->setEnabled(false);
  rviz_initial_pose_button_ =
      new QPushButton(QStringLiteral("② 设置初始位姿"), rviz_map_controls_);
  rviz_initial_pose_button_->setObjectName(QStringLiteral("initialPoseButton"));
  rviz_initial_pose_button_->setEnabled(false);
  rviz_follow_vehicle_button_ =
      new QPushButton(QStringLiteral("③ 跟随车辆"), rviz_map_controls_);
  rviz_follow_vehicle_button_->setObjectName(QStringLiteral("followVehicleButton"));
  rviz_follow_vehicle_button_->setEnabled(false);
  rviz_follow_vehicle_button_->setToolTip(
      QStringLiteral("保持 map 为固定坐标系，将二维视角锁定在 base_link；"
                     "仅在三维 ICP 已 LOCALIZED 后启用"));
  rviz_3d_map_button_ =
      new QPushButton(QStringLiteral("④ 显示静态三维先验"), rviz_map_controls_);
  rviz_3d_map_button_->setObjectName(QStringLiteral("threeDMapButton"));
  rviz_3d_map_button_->setCheckable(true);
  rviz_3d_map_button_->setChecked(false);
  rviz_3d_map_button_->setEnabled(false);
  rviz_3d_map_button_->setToolTip(
      QStringLiteral("显示启动时锁存的已知三维 PCD；它是静态先验，不是实时局部点云"));
  connect(rviz_fit_map_button_, &QPushButton::clicked, this,
          &MainWindow::fitOverviewMapView);
  connect(rviz_initial_pose_button_, &QPushButton::clicked, this,
          &MainWindow::selectInitialPoseTool);
  connect(rviz_follow_vehicle_button_, &QPushButton::clicked, this,
          &MainWindow::followOverviewVehicle);
  connect(rviz_3d_map_button_, &QPushButton::clicked, this,
          &MainWindow::toggleOverview3dMap);
  map_controls_layout->addWidget(rviz_map_instruction_, 1);
  map_controls_layout->addWidget(rviz_fit_map_button_);
  map_controls_layout->addWidget(rviz_initial_pose_button_);
  map_controls_layout->addWidget(rviz_follow_vehicle_button_);
  map_controls_layout->addWidget(rviz_3d_map_button_);
  rviz_map_controls_->setVisible(false);
  rviz_layout_->addWidget(rviz_map_controls_);

  auto* route_legend = new QLabel(
      QStringLiteral("路线图例：青色＝覆盖条带预览 · 蓝色＝全局参考路线 · "
                     "红色＝当前局部轨迹 · 绿色＝覆盖执行记录"),
      rviz_host_);
  route_legend->setWordWrap(true);
  route_legend->setStyleSheet(
      QStringLiteral("color:#aebfd2;background:#121c29;padding:5px 9px;"
                     "border-radius:5px;font-size:10pt;"));
  rviz_layout_->addWidget(route_legend);

  rviz_placeholder_ = new QLabel(
      QStringLiteral("RViz 将在 ROS master 可用后加载\n未启动导航节点时，其他页面仍可正常使用"),
      rviz_host_);
  rviz_placeholder_->setAlignment(Qt::AlignCenter);
  rviz_placeholder_->setStyleSheet(
      QStringLiteral("background:#0b1119;border:1px solid #2b3a4e;border-radius:8px;"
                     "color:#718096;font-size:14pt;"));
  rviz_layout_->addWidget(rviz_placeholder_, 1);
  rviz_host_->setMinimumWidth(600);
  splitter->addWidget(rviz_host_);

  auto* side = new QScrollArea(splitter);
  side->setWidgetResizable(true);
  side->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  side->setMinimumWidth(360);
  side->setMaximumWidth(520);
  auto* side_content = new QWidget(side);
  auto* side_layout = new QVBoxLayout(side_content);
  side_layout->setContentsMargins(10, 0, 10, 8);
  side_layout->setSpacing(14);

  auto* camera = new QGroupBox(QStringLiteral("相机 / YOLO11 实时画面"), side_content);
  auto* camera_layout = new QVBoxLayout(camera);
  overview_camera_preview_ = new QLabel(
      QStringLiteral("等待 /fod_camera/image_raw\n或 /fod/debug/image"), camera);
  overview_camera_preview_->setAlignment(Qt::AlignCenter);
  overview_camera_preview_->setMinimumHeight(210);
  overview_camera_preview_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  overview_camera_preview_->setStyleSheet(
      QStringLiteral("background:#080d13;border:1px solid #334154;border-radius:6px;"
                     "color:#718096;font-size:11pt;"));
  camera_layout->addWidget(overview_camera_preview_, 1);
  auto* open_vision = new QPushButton(QStringLiteral("打开视觉识别与相机控制"), camera);
  connect(open_vision, &QPushButton::clicked, this, [this]() { tabs_->setCurrentIndex(3); });
  camera_layout->addWidget(open_vision);
  side_layout->addWidget(camera);

  auto* vehicle = new QGroupBox(QStringLiteral("FAST-LIO 局部定位"), side_content);
  auto* vehicle_layout = new QVBoxLayout(vehicle);
  vehicle_layout->addWidget(
      createMetricRow(QStringLiteral("健康度"), &values_["overview_fastlio_health"]));
  vehicle_layout->addWidget(createMetricRow(QStringLiteral("局部坐标"), &values_["overview_xy"]));
  vehicle_layout->addWidget(createMetricRow(QStringLiteral("局部 yaw"), &values_["overview_yaw"],
                                            QStringLiteral("°")));
  vehicle_layout->addWidget(createMetricRow(QStringLiteral("速度"), &values_["overview_speed"],
                                            QStringLiteral("m/s")));
  side_layout->addWidget(vehicle);

  auto* relative_goal = new QGroupBox(QStringLiteral("车体相对目标"), side_content);
  auto* relative_goal_layout = new QGridLayout(relative_goal);
  relative_goal_layout->setHorizontalSpacing(8);
  relative_goal_layout->setVerticalSpacing(8);
  auto make_relative_input = [relative_goal](double minimum, double maximum,
                                             double initial) {
    auto* input = new QDoubleSpinBox(relative_goal);
    input->setDecimals(2);
    input->setRange(minimum, maximum);
    input->setSingleStep(0.10);
    input->setValue(initial);
    input->setKeyboardTracking(false);
    input->setSuffix(QStringLiteral(" m"));
    return input;
  };
  relative_forward_input_ = make_relative_input(-30.0, 30.0, 2.0);
  relative_left_input_ = make_relative_input(-30.0, 30.0, 0.0);
  relative_yaw_input_ = make_relative_input(-180.0, 180.0, 0.0);
  relative_yaw_input_->setSuffix(QStringLiteral(" °"));
  auto* forward_label = new QLabel(QStringLiteral("Δ前向"), relative_goal);
  auto* left_label = new QLabel(QStringLiteral("Δ左向"), relative_goal);
  auto* yaw_label = new QLabel(QStringLiteral("ΔYaw"), relative_goal);
  for (QLabel* label : { forward_label, left_label, yaw_label })
    label->setProperty("class", QStringLiteral("metricName"));
  relative_goal_layout->addWidget(forward_label, 0, 0);
  relative_goal_layout->addWidget(relative_forward_input_, 0, 1);
  relative_goal_layout->addWidget(left_label, 1, 0);
  relative_goal_layout->addWidget(relative_left_input_, 1, 1);
  relative_goal_layout->addWidget(yaw_label, 2, 0);
  relative_goal_layout->addWidget(relative_yaw_input_, 2, 1);
  relative_goal_button_ = new QPushButton(QStringLiteral("发送相对目标"), relative_goal);
  connect(relative_goal_button_, &QPushButton::clicked, this, &MainWindow::sendRelativeGoal);
  relative_goal_layout->addWidget(relative_goal_button_, 3, 0, 1, 2);
  relative_goal_layout->addWidget(
      createMetricRow(QStringLiteral("换算目标"), &values_["relative_goal_preview"]),
      4, 0, 1, 2);
  relative_goal_layout->addWidget(
      createMetricRow(QStringLiteral("入口状态"), &values_["relative_goal_hint"]),
      5, 0, 1, 2);
  side_layout->addWidget(relative_goal);

  auto* control_mode = new QGroupBox(QStringLiteral("相对导航 / 视觉控制模式"), side_content);
  auto* control_mode_layout = new QVBoxLayout(control_mode);
  control_mode_layout->addWidget(
      createMetricRow(QStringLiteral("当前模式"), &values_["overview_mode"]));
  control_mode_layout->addWidget(
      createMetricRow(QStringLiteral("相对导航"), &values_["overview_navigation_paused"]));
  overview_fod_start_button_ = new QPushButton(QStringLiteral("立即单独启动"), control_mode);
  overview_fod_start_button_->setObjectName(QStringLiteral("visionButton"));
  overview_fod_stop_button_ =
      new QPushButton(QStringLiteral("退出视觉模式并恢复相对导航"), control_mode);
  overview_fod_stop_button_->setObjectName(QStringLiteral("dangerButton"));
  connect(overview_fod_start_button_, &QPushButton::clicked, this, &MainWindow::startFodMode);
  connect(overview_fod_stop_button_, &QPushButton::clicked, this, &MainWindow::stopFodMode);
  control_mode_layout->addWidget(overview_fod_start_button_);
  control_mode_layout->addWidget(overview_fod_stop_button_);
  auto* mode_note = new QLabel(
      QStringLiteral("启动视觉控制时，安全仲裁器会先让当前局部路线休眠并确认车辆停车；"
                     "视觉完成后自动恢复保留的相对导航路线。"),
      control_mode);
  mode_note->setWordWrap(true);
  mode_note->setStyleSheet(QStringLiteral("color:#d8b46a;font-size:10pt;padding:4px;"));
  control_mode_layout->addWidget(mode_note);
  side_layout->addWidget(control_mode);

  auto* mission = new QGroupBox(QStringLiteral("当前任务"), side_content);
  auto* mission_layout = new QVBoxLayout(mission);
  mission_layout->addWidget(createMetricRow(QStringLiteral("导航状态"), &values_["overview_nav"]));
  mission_layout->addWidget(createMetricRow(QStringLiteral("定位结论"),
                                            &values_["overview_fastlio_summary"]));
  auto* open_test = new QPushButton(QStringLiteral("打开测试控制页"), mission);
  connect(open_test, &QPushButton::clicked, this, [this]() { tabs_->setCurrentIndex(2); });
  mission_layout->addWidget(open_test);
  rviz_panels_button_ = new QPushButton(QStringLiteral("显示 RViz 调试面板"), mission);
  rviz_panels_button_->setEnabled(false);
  connect(rviz_panels_button_, &QPushButton::clicked, this, &MainWindow::toggleRvizPanels);
  mission_layout->addWidget(rviz_panels_button_);
  side_layout->addWidget(mission);

  auto* note = new QLabel(
      QStringLiteral("界面不会直接发布 /cmd_vel；点击发送时只向 /move_base_simple/goal "
                     "发布 camera_init 局部目标，并受 FAST-LIO 健康门控。"),
      side_content);
  note->setWordWrap(true);
  note->setStyleSheet(QStringLiteral("color:#8fa0b5;padding:8px;"));
  side_layout->addWidget(note);
  side_layout->addStretch();
  side->setWidget(side_content);
  splitter->addWidget(side);
  splitter->setCollapsible(0, false);
  splitter->setCollapsible(1, false);
  splitter->setStretchFactor(0, 3);
  splitter->setStretchFactor(1, 1);
  splitter->setSizes({ 1100, 420 });

  overview_events_ = new QPlainTextEdit(page);
  overview_events_->setReadOnly(true);
  overview_events_->setMaximumBlockCount(300);
  overview_events_->setPlaceholderText(QStringLiteral("运行事件与告警"));
  overview_events_->setMinimumHeight(100);

  auto* vertical_splitter = new QSplitter(Qt::Vertical, page);
  vertical_splitter->addWidget(splitter);
  vertical_splitter->addWidget(overview_events_);
  vertical_splitter->setCollapsible(0, false);
  vertical_splitter->setCollapsible(1, false);
  vertical_splitter->setStretchFactor(0, 8);
  vertical_splitter->setStretchFactor(1, 1);
  vertical_splitter->setSizes({ 780, 140 });
  layout->addWidget(vertical_splitter, 1);
  return page;
}

QWidget* MainWindow::buildFastLioPage()
{
  auto* page = new QWidget(this);
  auto* root = new QVBoxLayout(page);
  root->setContentsMargins(20, 16, 20, 18);
  root->setSpacing(16);
  auto* grid = new QGridLayout();
  grid->setHorizontalSpacing(16);
  grid->setVerticalSpacing(16);

  auto* verdict = new QGroupBox(QStringLiteral("综合健康结论"), page);
  auto* verdict_layout = new QVBoxLayout(verdict);
  verdict_layout->addWidget(
      createMetricRow(QStringLiteral("健康分"), &values_["fastlio_score"],
                      QStringLiteral("/ 100")));
  values_["fastlio_score"]->setStyleSheet(
      QStringLiteral("font-size:30pt;font-weight:800;color:#69d3ff;"));
  verdict_layout->addWidget(
      createMetricRow(QStringLiteral("状态"), &values_["fastlio_state"]));
  verdict_layout->addWidget(
      createMetricRow(QStringLiteral("判定依据"), &values_["fastlio_findings"]));
  grid->addWidget(verdict, 0, 0);

  auto* streams = new QGroupBox(QStringLiteral("FAST-LIO 数据链"), page);
  auto* streams_layout = new QVBoxLayout(streams);
  streams_layout->addWidget(
      createMetricRow(QStringLiteral("里程计 年龄 / 频率"), &values_["fastlio_odom_stream"]));
  streams_layout->addWidget(
      createMetricRow(QStringLiteral("注册点云 年龄 / 频率"), &values_["fastlio_cloud_stream"]));
  streams_layout->addWidget(
      createMetricRow(QStringLiteral("IMU 年龄 / 频率"), &values_["fastlio_imu_stream"]));
  streams_layout->addWidget(
      createMetricRow(QStringLiteral("当前点数"), &values_["fastlio_cloud_points"]));
  streams_layout->addWidget(
      createMetricRow(QStringLiteral("TF camera_init → base_link"), &values_["fastlio_tf"]));
  grid->addWidget(streams, 0, 1);

  auto* pose = new QGroupBox(QStringLiteral("局部姿态与估计不确定度"), page);
  auto* pose_layout = new QVBoxLayout(pose);
  pose_layout->addWidget(createMetricRow(QStringLiteral("X / Y / Z"), &values_["fastlio_xyz"]));
  pose_layout->addWidget(createMetricRow(QStringLiteral("Yaw"), &values_["fastlio_yaw"],
                                         QStringLiteral("°")));
  pose_layout->addWidget(createMetricRow(QStringLiteral("线速度 / 角速度"),
                                         &values_["fastlio_velocity"]));
  pose_layout->addWidget(createMetricRow(QStringLiteral("内部位置 σxy"),
                                         &values_["fastlio_position_sigma"],
                                         QStringLiteral("m")));
  pose_layout->addWidget(createMetricRow(QStringLiteral("内部 yaw σ"),
                                         &values_["fastlio_yaw_sigma"],
                                         QStringLiteral("°")));
  pose_layout->addWidget(createMetricRow(QStringLiteral("坐标系"),
                                         &values_["fastlio_frames"]));
  grid->addWidget(pose, 1, 0);

  auto* stability = new QGroupBox(QStringLiteral("连续性与静止漂移"), page);
  auto* stability_layout = new QVBoxLayout(stability);
  stability_layout->addWidget(createMetricRow(QStringLiteral("近 2 秒最大单帧位移"),
                                              &values_["fastlio_pose_step"],
                                              QStringLiteral("m")));
  stability_layout->addWidget(createMetricRow(QStringLiteral("近 2 秒最大单帧转角"),
                                              &values_["fastlio_yaw_step"],
                                              QStringLiteral("°")));
  stability_layout->addWidget(createMetricRow(QStringLiteral("静止窗口"),
                                              &values_["fastlio_stationary_window"],
                                              QStringLiteral("s")));
  stability_layout->addWidget(createMetricRow(QStringLiteral("静止窗口最大漂移"),
                                              &values_["fastlio_stationary_drift"],
                                              QStringLiteral("m")));
  grid->addWidget(stability, 1, 1);

  grid->setColumnStretch(0, 1);
  grid->setColumnStretch(1, 1);
  grid->setRowStretch(0, 1);
  grid->setRowStretch(1, 1);
  root->addLayout(grid, 1);

  auto* explanation = new QLabel(
      QStringLiteral("判定口径：里程计与注册点云约 10 Hz、IMU 约 200 Hz；数据超过阈值、"
                     "TF 断开、四元数非法、位姿突跳或静止 5 秒漂移过大都会扣分。"
                     "协方差只表示 FAST-LIO 自身估计的不确定度，不等同于相对测量真值的绝对误差。"),
      page);
  explanation->setWordWrap(true);
  explanation->setStyleSheet(QStringLiteral("color:#b7c4d4;padding:10px;font-size:12pt;"));
  root->addWidget(explanation);

  auto* scroll = new QScrollArea(this);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setWidgetResizable(true);
  scroll->setWidget(page);
  return scroll;
}

QWidget* MainWindow::buildTestPage()
{
  auto* page = new QWidget(this);
  auto* root = new QVBoxLayout(page);
  root->setContentsMargins(20, 16, 20, 18);
  root->setSpacing(16);

  auto* readiness = new QGroupBox(QStringLiteral("测试前置状态"), page);
  auto* readiness_layout = new QVBoxLayout(readiness);
  readiness_layout->addWidget(createMetricRow(QStringLiteral("ROS master"), &values_["test_ros"]));
  readiness_layout->addWidget(createMetricRow(QStringLiteral("FAST-LIO 健康"),
                                              &values_["test_fastlio"]));
  readiness_layout->addWidget(
      createMetricRow(QStringLiteral("定位里程计"), &values_["test_odom"]));
  readiness_layout->addWidget(createMetricRow(QStringLiteral("局部目标订阅者"),
                                              &values_["test_goal_subscribers"]));

  auto* controls = new QGroupBox(QStringLiteral("基础现场测试"), page);
  auto* controls_layout = new QGridLayout(controls);
  controls_layout->setHorizontalSpacing(14);
  controls_layout->setVerticalSpacing(14);
  forward_goal_button_ = new QPushButton(QStringLiteral("发送车头正前方 2 m 相对目标"), controls);
  forward_goal_button_->setMinimumHeight(58);
  auto* cancel = new QPushButton(QStringLiteral("取消当前导航目标"), controls);
  cancel->setObjectName(QStringLiteral("dangerButton"));
  cancel->setMinimumHeight(58);
  record_button_ = new QPushButton(QStringLiteral("开始录包"), controls);
  record_button_->setObjectName(QStringLiteral("recordButton"));
  record_button_->setMinimumHeight(58);
  static_map_start_button_ = new QPushButton(QStringLiteral("录入静态地图"), controls);
  static_map_start_button_->setMinimumHeight(58);
  static_map_stop_button_ = new QPushButton(QStringLiteral("结束静态地图录入"), controls);
  static_map_stop_button_->setObjectName(QStringLiteral("dangerButton"));
  static_map_stop_button_->setMinimumHeight(58);
  connect(forward_goal_button_, &QPushButton::clicked, this,
          &MainWindow::sendForwardRelativeGoal);
  connect(cancel, &QPushButton::clicked, this, &MainWindow::cancelNavigation);
  connect(record_button_, &QPushButton::clicked, this, &MainWindow::toggleRecording);
  connect(static_map_start_button_, &QPushButton::clicked, this,
          &MainWindow::startStaticMapping);
  connect(static_map_stop_button_, &QPushButton::clicked, this,
          &MainWindow::stopStaticMapping);
  controls_layout->addWidget(forward_goal_button_, 0, 0);
  controls_layout->addWidget(cancel, 0, 1);
  controls_layout->addWidget(record_button_, 0, 2);
  controls_layout->addWidget(static_map_start_button_, 1, 0, 1, 2);
  controls_layout->addWidget(static_map_stop_button_, 1, 2);

  auto* roadmap = new QGroupBox(QStringLiteral("后续测试中心"), page);
  auto* roadmap_layout = new QVBoxLayout(roadmap);
  auto* roadmap_text = new QLabel(
      QStringLiteral("相对目标按当前车体姿态换算到 camera_init，再发布 PoseStamped。"
                     "发送入口要求三路 FAST-LIO 数据、TF、定位健康度、模式仲裁与 move_base "
                     "订阅全部就绪；普通录包与三地图静态建图是两个独立流程。"),
      roadmap);
  roadmap_text->setWordWrap(true);
  roadmap_text->setStyleSheet(QStringLiteral("color:#b7c4d4;font-size:12pt;"));
  roadmap_layout->addWidget(roadmap_text);

  auto* dashboard = new QGridLayout();
  dashboard->setHorizontalSpacing(16);
  dashboard->setVerticalSpacing(16);
  dashboard->addWidget(readiness, 0, 0, 2, 1);
  dashboard->addWidget(controls, 0, 1);
  dashboard->addWidget(roadmap, 1, 1);
  dashboard->setColumnStretch(0, 1);
  dashboard->setColumnStretch(1, 2);
  root->addLayout(dashboard);
  root->addStretch();

  auto* scroll = new QScrollArea(this);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setWidgetResizable(true);
  scroll->setWidget(page);
  return scroll;
}

QWidget* MainWindow::buildVisionPage()
{
  auto* page = new QWidget(this);
  auto* root = new QVBoxLayout(page);
  root->setContentsMargins(16, 12, 16, 14);
  root->setSpacing(10);

  auto* splitter = new QSplitter(Qt::Horizontal, page);
  auto* image_panel = new QWidget(splitter);
  auto* image_layout = new QVBoxLayout(image_panel);
  image_layout->setContentsMargins(0, 0, 8, 0);
  image_layout->setSpacing(8);
  auto* image_title = new QLabel(QStringLiteral("YOLO11 标注画面（识别未就绪时回退到相机原图）"),
                                 image_panel);
  image_title->setStyleSheet(
      QStringLiteral("font-size:15pt;font-weight:700;color:#dce7f4;padding:4px;"));
  image_layout->addWidget(image_title);
  vision_camera_preview_ = new QLabel(
      QStringLiteral("等待 /fod/debug/image 或 /fod_camera/image_raw"), image_panel);
  vision_camera_preview_->setAlignment(Qt::AlignCenter);
  vision_camera_preview_->setMinimumSize(640, 480);
  vision_camera_preview_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  vision_camera_preview_->setStyleSheet(
      QStringLiteral("background:#080d13;border:1px solid #334154;border-radius:8px;"
                     "color:#718096;font-size:14pt;"));
  image_layout->addWidget(vision_camera_preview_, 1);
  image_layout->addWidget(
      createMetricRow(QStringLiteral("当前画面来源"), &values_["vision_image_source"]));

  auto* detections_group = new QGroupBox(QStringLiteral("当前检测目标"), image_panel);
  auto* detections_layout = new QVBoxLayout(detections_group);
  vision_detections_ = new QPlainTextEdit(detections_group);
  vision_detections_->setReadOnly(true);
  vision_detections_->setMaximumBlockCount(100);
  vision_detections_->setMinimumHeight(150);
  vision_detections_->setPlaceholderText(QStringLiteral("尚未收到 /fod/detections"));
  detections_layout->addWidget(vision_detections_);
  image_layout->addWidget(detections_group);
  splitter->addWidget(image_panel);

  auto* controls_scroll = new QScrollArea(splitter);
  controls_scroll->setWidgetResizable(true);
  controls_scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  controls_scroll->setMinimumWidth(430);
  controls_scroll->setMaximumWidth(580);
  auto* controls = new QWidget(controls_scroll);
  auto* controls_layout = new QVBoxLayout(controls);
  controls_layout->setContentsMargins(8, 0, 8, 8);
  controls_layout->setSpacing(14);

  auto* health = new QGroupBox(QStringLiteral("相机与 YOLO11 状态"), controls);
  auto* health_layout = new QVBoxLayout(health);
  health_layout->addWidget(
      createMetricRow(QStringLiteral("相机流"), &values_["vision_camera_status"]));
  health_layout->addWidget(
      createMetricRow(QStringLiteral("分辨率 / 编码"), &values_["vision_camera_format"]));
  health_layout->addWidget(
      createMetricRow(QStringLiteral("YOLO 状态"), &values_["vision_yolo_status"]));
  health_layout->addWidget(
      createMetricRow(QStringLiteral("模型"), &values_["vision_model"]));
  health_layout->addWidget(
      createMetricRow(QStringLiteral("推理耗时"), &values_["vision_inference"],
                      QStringLiteral("ms")));
  health_layout->addWidget(
      createMetricRow(QStringLiteral("消息频率"), &values_["vision_fps"],
                      QStringLiteral("Hz")));
  health_layout->addWidget(
      createMetricRow(QStringLiteral("目标数"), &values_["vision_detection_count"]));
  controls_layout->addWidget(health);

  auto* imaging = new QGroupBox(QStringLiteral("ZED 2 曝光 / 增益"), controls);
  auto* imaging_layout = new QGridLayout(imaging);
  imaging_layout->setHorizontalSpacing(10);
  imaging_layout->setVerticalSpacing(9);
  exposure_auto_checkbox_ =
      new QCheckBox(QStringLiteral("ZED 自动曝光与增益（联动）"), imaging);
  exposure_auto_checkbox_->setChecked(true);
  exposure_input_ = new QDoubleSpinBox(imaging);
  exposure_input_->setDecimals(0);
  exposure_input_->setRange(0.0, 100.0);
  exposure_input_->setSingleStep(1.0);
  exposure_input_->setSuffix(QStringLiteral(" %"));
  exposure_input_->setKeyboardTracking(false);
  gain_input_ = new QDoubleSpinBox(imaging);
  gain_input_->setDecimals(0);
  gain_input_->setRange(0.0, 100.0);
  gain_input_->setSingleStep(1.0);
  gain_input_->setSuffix(QStringLiteral(" %"));
  gain_input_->setKeyboardTracking(false);
  camera_query_button_ = new QPushButton(QStringLiteral("读取相机参数"), imaging);
  camera_apply_button_ = new QPushButton(QStringLiteral("应用曝光 / 增益"), imaging);
  connect(camera_query_button_, &QPushButton::clicked, this, &MainWindow::queryCameraControls);
  connect(camera_apply_button_, &QPushButton::clicked, this, &MainWindow::applyCameraControls);
  connect(exposure_auto_checkbox_, &QCheckBox::toggled, this,
          [this](bool automatic) {
            exposure_input_->setEnabled(!automatic);
            gain_input_->setEnabled(!automatic);
          });
  exposure_input_->setEnabled(false);
  gain_input_->setEnabled(false);
  imaging_layout->addWidget(exposure_auto_checkbox_, 0, 0, 1, 2);
  imaging_layout->addWidget(new QLabel(QStringLiteral("曝光比例"), imaging), 1, 0);
  imaging_layout->addWidget(exposure_input_, 1, 1);
  imaging_layout->addWidget(new QLabel(QStringLiteral("增益比例"), imaging), 2, 0);
  imaging_layout->addWidget(gain_input_, 2, 1);
  imaging_layout->addWidget(camera_query_button_, 3, 0);
  imaging_layout->addWidget(camera_apply_button_, 3, 1);
  controls_layout->addWidget(imaging);

  auto* quality = new QGroupBox(QStringLiteral("路面 ROI 图像质量控制"), controls);
  auto* quality_layout = new QVBoxLayout(quality);
  quality_layout->addWidget(
      createMetricRow(QStringLiteral("控制状态"), &values_["quality_state"]));
  quality_layout->addWidget(
      createMetricRow(QStringLiteral("亮度中位数"), &values_["quality_median"]));
  quality_layout->addWidget(
      createMetricRow(QStringLiteral("清晰度"), &values_["quality_sharpness"]));
  quality_layout->addWidget(
      createMetricRow(QStringLiteral("当前曝光"), &values_["quality_exposure"],
                      QStringLiteral("%")));
  quality_layout->addWidget(
      createMetricRow(QStringLiteral("当前增益"), &values_["quality_gain"]));
  quality_layout->addWidget(
      createMetricRow(QStringLiteral("最近动作"), &values_["quality_action"]));
  auto* quality_buttons = new QHBoxLayout();
  image_quality_enable_button_ = new QPushButton(QStringLiteral("启用智能控制"), quality);
  image_quality_disable_button_ = new QPushButton(QStringLiteral("停用并恢复相机自动"), quality);
  connect(image_quality_enable_button_, &QPushButton::clicked, this,
          &MainWindow::enableImageQualityControl);
  connect(image_quality_disable_button_, &QPushButton::clicked, this,
          &MainWindow::disableImageQualityControl);
  quality_buttons->addWidget(image_quality_enable_button_);
  quality_buttons->addWidget(image_quality_disable_button_);
  quality_layout->addLayout(quality_buttons);
  controls_layout->addWidget(quality);

  auto* mode = new QGroupBox(QStringLiteral("视觉行驶模式（可选）"), controls);
  auto* mode_layout = new QVBoxLayout(mode);
  mode_layout->addWidget(
      createMetricRow(QStringLiteral("安全仲裁状态"), &values_["vision_mode_state"]));
  mode_layout->addWidget(
      createMetricRow(QStringLiteral("相对导航"), &values_["vision_navigation_paused"]));
  mode_layout->addWidget(
      createMetricRow(QStringLiteral("视觉控制器"), &values_["vision_servo_state"]));
  mode_layout->addWidget(
      createMetricRow(QStringLiteral("状态原因"), &values_["vision_mode_reason"]));
  fod_start_button_ = new QPushButton(QStringLiteral("立即单独启动"), mode);
  fod_start_button_->setObjectName(QStringLiteral("visionButton"));
  fod_start_button_->setMinimumHeight(58);
  fod_stop_button_ = new QPushButton(QStringLiteral("退出视觉模式并恢复相对导航"), mode);
  fod_stop_button_->setObjectName(QStringLiteral("dangerButton"));
  fod_stop_button_->setMinimumHeight(52);
  connect(fod_start_button_, &QPushButton::clicked, this, &MainWindow::startFodMode);
  connect(fod_stop_button_, &QPushButton::clicked, this, &MainWindow::stopFodMode);
  mode_layout->addWidget(fod_start_button_);
  mode_layout->addWidget(fod_stop_button_);
  auto* safety = new QLabel(
      QStringLiteral("仅在封闭净空区域、操作员手持物理急停且检测稳定时启动。按钮只调用"
                     "安全模式仲裁器：先屏蔽并暂停局部路线、取消当前子目标、确认停车，再允许"
                     "视觉控制车辆；任何预检失败都会保持停车。"),
      mode);
  safety->setWordWrap(true);
  safety->setStyleSheet(
      QStringLiteral("background:#3d3222;color:#f0cf8a;border:1px solid #8e6a2d;"
                     "border-radius:6px;padding:10px;font-size:10pt;"));
  mode_layout->addWidget(safety);
  controls_layout->addWidget(mode);
  controls_layout->addStretch();
  controls_scroll->setWidget(controls);
  splitter->addWidget(controls_scroll);
  splitter->setCollapsible(0, false);
  splitter->setCollapsible(1, false);
  splitter->setStretchFactor(0, 3);
  splitter->setStretchFactor(1, 1);
  splitter->setSizes({ 1120, 500 });
  root->addWidget(splitter, 1);
  return page;
}

QWidget* MainWindow::buildPlaceholderPage(const QString& title, const QString& subtitle,
                                           const QStringList& planned_items)
{
  auto* page = new QWidget(this);
  auto* root = new QVBoxLayout(page);
  root->setContentsMargins(42, 34, 42, 34);
  auto* heading = new QLabel(title, page);
  heading->setStyleSheet(QStringLiteral("font-size:24pt;font-weight:700;color:#f1f5f9;"));
  auto* subheading = new QLabel(subtitle, page);
  subheading->setStyleSheet(QStringLiteral("font-size:12pt;color:#8fa0b5;"));
  root->addWidget(heading);
  root->addWidget(subheading);
  root->addSpacing(22);
  auto* group = new QGroupBox(QStringLiteral("预留接口与显示区域"), page);
  auto* group_layout = new QVBoxLayout(group);
  for (const QString& item : planned_items)
  {
    auto* row = new QLabel(QStringLiteral("●  ") + item, group);
    row->setStyleSheet(QStringLiteral("font-size:12pt;color:#c3cfdd;padding:10px;"));
    group_layout->addWidget(row);
  }
  root->addWidget(group);
  root->addStretch();
  return page;
}

QWidget* MainWindow::buildCoveragePage()
{
  auto* page = new QWidget(this);
  auto* root = new QVBoxLayout(page);
  root->setContentsMargins(16, 12, 16, 12);

  auto* splitter = new QSplitter(Qt::Horizontal, page);
  coverage_rviz_host_ = new QWidget(splitter);
  coverage_rviz_layout_ = new QVBoxLayout(coverage_rviz_host_);
  coverage_rviz_layout_->setContentsMargins(0, 0, 0, 0);
  auto* route_legend = new QLabel(
      QStringLiteral("路线图例：青色＝覆盖条带预览 · 蓝色＝全局参考路线 · "
                     "红色＝当前局部轨迹 · 绿色＝覆盖执行记录"),
      coverage_rviz_host_);
  route_legend->setWordWrap(true);
  route_legend->setStyleSheet(
      QStringLiteral("color:#aebfd2;background:#121c29;padding:5px 9px;"
                     "border-radius:5px;font-size:10pt;"));
  coverage_rviz_layout_->addWidget(route_legend);
  coverage_rviz_placeholder_ = new QLabel(
      QStringLiteral("实验性全局地图将在静态地图模式下加载\n"
                     "无全局地图时，覆盖清扫功能保持禁用"),
      coverage_rviz_host_);
  coverage_rviz_placeholder_->setAlignment(Qt::AlignCenter);
  coverage_rviz_placeholder_->setStyleSheet(
      QStringLiteral("background:#0b1119;border:1px solid #2b3a4e;border-radius:8px;"
                     "color:#718096;font-size:14pt;"));
  coverage_rviz_layout_->addWidget(coverage_rviz_placeholder_);
  coverage_rviz_host_->setMinimumWidth(650);
  splitter->addWidget(coverage_rviz_host_);

  auto* side = new QScrollArea(splitter);
  side->setObjectName(QStringLiteral("coverageSide"));
  side->setWidgetResizable(true);
  side->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  side->setMinimumWidth(390);
  side->setMaximumWidth(540);
  auto* controls = new QWidget(side);
  controls->setObjectName(QStringLiteral("coverageControls"));
  auto* controls_layout = new QVBoxLayout(controls);
  controls_layout->setContentsMargins(10, 0, 10, 10);
  controls_layout->setSpacing(12);

  auto* selection = new QGroupBox(QStringLiteral("覆盖区域"), controls);
  auto* selection_layout = new QVBoxLayout(selection);
  auto* instructions = new QLabel(
      QStringLiteral("点击“框定”后，在左侧地图用 Publish Point 连续点选任意数量顶点。"
                     "可逐点撤销或随时取消；确认后首尾自动闭合为清扫区域。"),
      selection);
  instructions->setWordWrap(true);
  instructions->setStyleSheet(QStringLiteral("color:#374151;font-size:10pt;"));
  selection_layout->addWidget(instructions);
  coverage_select_button_ = new QPushButton(QStringLiteral("框定覆盖清扫范围"), selection);
  coverage_select_button_->setEnabled(false);
  connect(coverage_select_button_, &QPushButton::clicked,
          this, &MainWindow::beginCoverageSelection);
  selection_layout->addWidget(coverage_select_button_);
  auto* edit_row = new QHBoxLayout();
  coverage_undo_button_ = new QPushButton(QStringLiteral("撤销一点"), selection);
  coverage_selection_cancel_button_ = new QPushButton(QStringLiteral("取消框定"), selection);
  coverage_selection_cancel_button_->setObjectName(QStringLiteral("dangerButton"));
  coverage_undo_button_->setEnabled(false);
  coverage_selection_cancel_button_->setEnabled(false);
  connect(coverage_undo_button_, &QPushButton::clicked,
          this, &MainWindow::undoCoveragePoint);
  connect(coverage_selection_cancel_button_, &QPushButton::clicked,
          this, &MainWindow::cancelCoverageSelection);
  edit_row->addWidget(coverage_undo_button_);
  edit_row->addWidget(coverage_selection_cancel_button_);
  selection_layout->addLayout(edit_row);
  coverage_confirm_button_ = new QPushButton(QStringLiteral("确认区域并生成轨迹"), selection);
  coverage_confirm_button_->setEnabled(false);
  connect(coverage_confirm_button_, &QPushButton::clicked,
          this, &MainWindow::confirmCoverageSelection);
  selection_layout->addWidget(coverage_confirm_button_);
  controls_layout->addWidget(selection);

  auto* parameters = new QGroupBox(QStringLiteral("规划参数"), controls);
  auto* parameters_layout = new QVBoxLayout(parameters);
  auto* width_row = new QWidget(parameters);
  auto* width_layout = new QHBoxLayout(width_row);
  width_layout->setContentsMargins(0, 0, 0, 0);
  width_layout->addWidget(new QLabel(QStringLiteral("有效清扫宽度"), width_row));
  coverage_width_input_ = new ScrollSafeDoubleSpinBox(width_row);
  coverage_width_input_->setRange(0.30, 3.00);
  coverage_width_input_->setDecimals(2);
  coverage_width_input_->setSingleStep(0.05);
  coverage_width_input_->setSuffix(QStringLiteral(" m"));
  coverage_width_input_->setValue(1.00);
  coverage_width_input_->setEnabled(false);
  width_layout->addWidget(coverage_width_input_, 1);
  parameters_layout->addWidget(width_row);
  auto* overlap_row = new QWidget(parameters);
  auto* overlap_layout = new QHBoxLayout(overlap_row);
  overlap_layout->setContentsMargins(0, 0, 0, 0);
  overlap_layout->addWidget(new QLabel(QStringLiteral("相邻轨迹重叠率"), overlap_row));
  coverage_overlap_input_ = new ScrollSafeDoubleSpinBox(overlap_row);
  coverage_overlap_input_->setRange(0.0, 50.0);
  coverage_overlap_input_->setDecimals(0);
  coverage_overlap_input_->setSingleStep(5.0);
  coverage_overlap_input_->setSuffix(QStringLiteral(" %"));
  coverage_overlap_input_->setValue(15.0);
  coverage_overlap_input_->setEnabled(false);
  overlap_layout->addWidget(coverage_overlap_input_, 1);
  parameters_layout->addWidget(overlap_row);
  auto* speed_row = new QWidget(parameters);
  auto* speed_layout = new QHBoxLayout(speed_row);
  speed_layout->setContentsMargins(0, 0, 0, 0);
  speed_layout->addWidget(new QLabel(QStringLiteral("最高前进速度"), speed_row));
  coverage_speed_input_ = new ScrollSafeDoubleSpinBox(speed_row);
  coverage_speed_input_->setRange(0.10, 1.60);
  coverage_speed_input_->setDecimals(2);
  coverage_speed_input_->setSingleStep(0.10);
  coverage_speed_input_->setSuffix(QStringLiteral(" m/s"));
  coverage_speed_input_->setValue(0.80);
  coverage_speed_input_->setEnabled(false);
  coverage_speed_input_->setToolTip(
      QStringLiteral("默认 0.80 m/s；覆盖任务限 1.60 m/s，并在开始前实时核对 VCU 上限"));
  speed_layout->addWidget(coverage_speed_input_, 1);
  parameters_layout->addWidget(speed_row);
  coverage_reverse_checkbox_ = new QCheckBox(
      QStringLiteral("仅在转场允许 TEB 尝试低速倒车"), parameters);
  coverage_reverse_checkbox_->setChecked(true);
  coverage_reverse_checkbox_->setEnabled(false);
  parameters_layout->addWidget(coverage_reverse_checkbox_);
  controls_layout->addWidget(parameters);

  auto* status = new QGroupBox(QStringLiteral("覆盖任务状态"), controls);
  auto* status_layout = new QVBoxLayout(status);
  status_layout->addWidget(createMetricRow(QStringLiteral("全局地图（实验）"),
                                            &values_["coverage_map"]));
  status_layout->addWidget(createMetricRow(QStringLiteral("当前车辆位姿"),
                                            &values_["coverage_pose"]));
  status_layout->addWidget(createMetricRow(QStringLiteral("最近位置里程计"),
                                            &values_["coverage_recent_odom"]));
  status_layout->addWidget(createMetricRow(QStringLiteral("覆盖导航状态"),
                                            &values_["coverage_state"]));
  status_layout->addWidget(createMetricRow(QStringLiteral("已选顶点"),
                                            &values_["coverage_points"]));
  status_layout->addWidget(createMetricRow(QStringLiteral("分段进度"),
                                            &values_["coverage_progress"]));
  status_layout->addWidget(createMetricRow(QStringLiteral("路线约束"),
                                            &values_["coverage_parameters"]));
  status_layout->addWidget(createMetricRow(QStringLiteral("运动学核对"),
                                            &values_["coverage_kinematics"]));
  status_layout->addWidget(createMetricRow(QStringLiteral("底盘执行门"),
                                            &values_["coverage_chassis"]));
  status_layout->addWidget(createMetricRow(QStringLiteral("障碍感知"),
                                            &values_["coverage_avoidance"]));
  status_layout->addWidget(createMetricRow(QStringLiteral("可覆盖 / 框定面积"),
                                            &values_["coverage_area"]));
  status_layout->addWidget(createMetricRow(QStringLiteral("不可覆盖估算"),
                                            &values_["coverage_unreachable"]));
  status_layout->addWidget(createMetricRow(QStringLiteral("覆盖进度估算"),
                                            &values_["coverage_ratio"]));
  status_layout->addWidget(createMetricRow(QStringLiteral("清扫机构"),
                                            &values_["coverage_actuator"]));
  status_layout->addWidget(createMetricRow(QStringLiteral("详情"),
                                            &values_["coverage_detail"]));
  controls_layout->addWidget(status);

  auto* task = new QGroupBox(QStringLiteral("任务控制"), controls);
  auto* task_layout = new QVBoxLayout(task);
  coverage_start_button_ = new QPushButton(QStringLiteral("开始覆盖清扫"), task);
  coverage_start_button_->setObjectName(QStringLiteral("visionButton"));
  coverage_pause_button_ = new QPushButton(QStringLiteral("暂停覆盖清扫"), task);
  coverage_cancel_button_ = new QPushButton(QStringLiteral("取消覆盖清扫"), task);
  coverage_cancel_button_->setObjectName(QStringLiteral("dangerButton"));
  coverage_start_button_->setEnabled(false);
  coverage_pause_button_->setEnabled(false);
  coverage_cancel_button_->setEnabled(false);
  connect(coverage_start_button_, &QPushButton::clicked,
          this, &MainWindow::startCoverage);
  connect(coverage_pause_button_, &QPushButton::clicked,
          this, &MainWindow::toggleCoveragePause);
  connect(coverage_cancel_button_, &QPushButton::clicked,
          this, &MainWindow::cancelCoverageTask);
  task_layout->addWidget(coverage_start_button_);
  auto* task_row = new QHBoxLayout();
  task_row->addWidget(coverage_pause_button_);
  task_row->addWidget(coverage_cancel_button_);
  task_layout->addLayout(task_row);
  auto* safety = new QLabel(
      QStringLiteral("V1 只执行导航覆盖，不控制主刷、边刷、风机或喷淋。开始前后端还会"
                     "复核全局定位、运动门、里程计和 FOD 导航仲裁；任一失败均不发车。"
                     "默认最高前进速度为 0.80 m/s，覆盖任务上限为 1.60 m/s；开始时还会"
                     "在线核对 VCU 轴距、最大转角和 TEB 最小转弯半径。"),
      task);
  safety->setWordWrap(true);
  safety->setStyleSheet(
      QStringLiteral("background:#3d3222;color:#f0cf8a;border:1px solid #8e6a2d;"
                     "border-radius:6px;padding:10px;font-size:10pt;"));
  task_layout->addWidget(safety);
  controls_layout->addWidget(task);
  controls_layout->addStretch();
  side->setWidget(controls);
  splitter->addWidget(side);
  splitter->setCollapsible(0, false);
  splitter->setCollapsible(1, false);
  splitter->setStretchFactor(0, 4);
  splitter->setStretchFactor(1, 1);
  splitter->setSizes({ 1200, 470 });
  root->addWidget(splitter, 1);
  return page;
}

QWidget* MainWindow::buildLogPage()
{
  auto* page = new QWidget(this);
  auto* root = new QVBoxLayout(page);
  root->setContentsMargins(18, 14, 18, 14);
  auto* explanation = new QTextBrowser(page);
  explanation->setHtml(QStringLiteral(
      "<h2>操作台运行原则</h2>"
      "<p>界面整合 RViz、FAST-LIO、相机、YOLO11 与安全模式控制；导航和底盘安全链仍由"
      "现有 ROS 节点执行。</p>"
      "<ul><li>ROS master 或任何业务节点缺失时，界面仍会打开并显示离线状态。</li>"
      "<li>界面只发布局部 PoseStamped 目标和取消 GoalID，从不发布 <code>/cmd_vel</code>。</li>"
      "<li>视觉行驶只调用 <code>/fod_navigation_mode/set_fod_enabled</code>；局部路线暂停、"
      "停车确认与恢复均由安全仲裁器完成。</li>"
      "<li>内嵌 RViz 与综合页相对目标都使用 camera_init 局部坐标。</li>"
      "</ul>"));
  explanation->setMaximumHeight(220);
  root->addWidget(explanation);
  log_events_ = new QPlainTextEdit(page);
  log_events_->setReadOnly(true);
  log_events_->setMaximumBlockCount(1000);
  log_events_->setPlaceholderText(QStringLiteral("操作台事件日志"));
  root->addWidget(log_events_, 1);
  return page;
}

void MainWindow::setStatus(const QString& key, Health health, const QString& state,
                           const QString& detail)
{
  auto it = status_cards_.find(key);
  if (it == status_cards_.end())
    return;
  const QString accent = statusColor(health);
  const QString background = statusBackground(health);
  it->second.frame->setStyleSheet(
      QStringLiteral("QFrame { background:%1; border:1px solid %2; border-radius:7px; }")
          .arg(background, accent));
  it->second.state->setStyleSheet(QStringLiteral("color:%1;border:0;background:transparent;")
                                      .arg(accent));
  it->second.detail->setStyleSheet(QStringLiteral("color:#aab5c3;border:0;background:transparent;"));
  it->second.state->setText(state);
  it->second.detail->setText(detail);
  it->second.frame->setToolTip(state + QStringLiteral(" · ") + detail);
}

void MainWindow::appendEvent(const QString& text, bool warning)
{
  const QString prefix = QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss")) +
                         (warning ? QStringLiteral("  [警告] ") : QStringLiteral("  "));
  const QString line = prefix + text;
  if (overview_events_)
    overview_events_->appendPlainText(line);
  if (log_events_)
    log_events_->appendPlainText(line);
}

void MainWindow::requestMasterProbe()
{
  if (master_probe_watcher_.isRunning() || !ros::ok())
    return;
  master_probe_watcher_.setFuture(QtConcurrent::run([]() {
    MasterProbeResult result;
    result.online = ros::master::check();
    return result;
  }));
}

void MainWindow::handleMasterProbeFinished()
{
  const MasterProbeResult result = master_probe_watcher_.result();
  const bool was_online = previous_probe_online_;
  const bool restored = result.online && !was_online;
  const bool lost = !result.online && was_online;
  // VisualizationFrame::initialize() may briefly process nested Qt events.  Commit
  // the observed state before entering setupRosInterfaces(), otherwise another
  // completed probe can re-enter this slot, treat the same master as "restored",
  // and destroy/recreate RViz while its OpenGL render panel is still initializing.
  previous_probe_online_ = result.online;
  master_online_ = result.online;

  if (restored)
  {
    // A restarted ROS master invalidates RViz's old topic registrations. Rebuild
    // the embedded frame after connectivity returns so its displays subscribe
    // against the new master instead of remaining on a frozen last frame.
    if (rviz_initialized_ && rviz_frame_)
    {
      if (rviz_panels_button_)
        rviz_panels_button_->setEnabled(false);
      rviz_layout_->removeWidget(rviz_frame_);
      coverage_rviz_layout_->removeWidget(rviz_frame_);
      delete rviz_frame_;
      rviz_frame_ = nullptr;
      rviz_initialized_ = false;
      rviz_attached_tab_index_ = -1;
      overview_fitted_map_count_ = 0;
      coverage_fitted_map_count_ = 0;
      overview_initial_pose_tool_active_ = false;
      rviz_follow_after_initial_pose_ = false;
      overview_3d_map_enabled_ = false;
      if (rviz_follow_vehicle_button_)
        rviz_follow_vehicle_button_->setEnabled(false);
      if (rviz_3d_map_button_)
      {
        rviz_3d_map_button_->setChecked(false);
        rviz_3d_map_button_->setEnabled(false);
        rviz_3d_map_button_->setText(QStringLiteral("④ 显示静态三维先验"));
      }
      if (rviz_map_instruction_)
        rviz_map_instruction_->setText(
            QStringLiteral("全局地图加载后：先显示全图，再按车辆真实位置设置初始位姿"));
      const QString placeholder_style =
          QStringLiteral("background:#0b1119;border:1px solid #2b3a4e;border-radius:8px;"
                         "color:#718096;font-size:14pt;");
      if (!rviz_placeholder_)
      {
        rviz_placeholder_ = new QLabel(
            QStringLiteral("ROS master 已恢复，正在重新加载 RViz……"), rviz_host_);
        rviz_placeholder_->setAlignment(Qt::AlignCenter);
        rviz_placeholder_->setStyleSheet(placeholder_style);
        rviz_layout_->addWidget(rviz_placeholder_, 1);
      }
      if (!coverage_rviz_placeholder_)
      {
        coverage_rviz_placeholder_ = new QLabel(
            QStringLiteral("ROS master 已恢复，正在重新加载清扫地图……"),
            coverage_rviz_host_);
        coverage_rviz_placeholder_->setAlignment(Qt::AlignCenter);
        coverage_rviz_placeholder_->setStyleSheet(placeholder_style);
        coverage_rviz_layout_->addWidget(coverage_rviz_placeholder_, 1);
      }
    }
    appendEvent(QStringLiteral("ROS master 已连接，正在注册界面订阅与发布接口。"));
    setupRosInterfaces();
  }
  else if (lost)
  {
    appendEvent(QStringLiteral("ROS master 连接中断；界面继续运行并保留最后一次数据。"), true);
  }
}

void MainWindow::setupRosInterfaces()
{
  if (!master_online_ || !ros::ok())
    return;
  // Starting a spinner calls ros::start(), which registers /rosout. Delay it until the
  // master is reachable; otherwise that registration retries forever before the Qt
  // window can become interactive.
  if (!spinner_)
  {
    spinner_.reset(new ros::AsyncSpinner(2));
    spinner_->start();
  }
  shutdownRosInterfaces();
  node_.reset(new ros::NodeHandle("~"));
  tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_, *node_, false));
  node_->param("enable_rviz", enable_rviz_, true);
  node_->param("static_map_mode", static_map_mode_, false);
  if (rviz_map_controls_)
    rviz_map_controls_->setVisible(static_map_mode_ && enable_rviz_);
  const std::string default_rviz =
      ros::package::getPath("autolabor_operator_gui") + "/config/operator_navigation.rviz";
  node_->param<std::string>(
      "navigation_mode_label", navigation_mode_label_, "FAST_LIO");
  node_->param<std::string>("rviz_config", rviz_config_path_, default_rviz);
  node_->param<std::string>("odom_topic", odom_topic_, "/Odometry");
  node_->param<std::string>("cloud_topic", cloud_topic_, "/cloud_registered_body");
  node_->param<std::string>("imu_topic", imu_topic_, "/livox/imu");
  node_->param<std::string>(
      "rviz_startup_fixed_frame", rviz_startup_fixed_frame_, "map");
  node_->param<std::string>(
      "rviz_navigation_fixed_frame", rviz_navigation_fixed_frame_, "map");
  if (app_subtitle_)
  {
    app_subtitle_->setText(
        QString::fromStdString(navigation_mode_label_) +
        QStringLiteral(" · INDOOR LOCALIZATION CONSOLE"));
  }

  odom_subscriber_ = node_->subscribe(odom_topic_, 20, &MainWindow::odomCallback, this);
  cloud_subscriber_ = node_->subscribe(cloud_topic_, 10, &MainWindow::cloudCallback, this);
  imu_subscriber_ = node_->subscribe(imu_topic_, 200, &MainWindow::imuCallback, this);
  can_subscriber_ = node_->subscribe("/canbus_msg", 100, &MainWindow::canCallback, this);
  scan_subscriber_ = node_->subscribe("/scan", 10, &MainWindow::scanCallback, this);
  navigation_subscriber_ =
      node_->subscribe("/move_base/status", 10, &MainWindow::navigationCallback, this);
  camera_image_subscriber_ =
      node_->subscribe("/fod_camera/image_raw", 1, &MainWindow::cameraImageCallback, this);
  debug_image_subscriber_ =
      node_->subscribe("/fod/debug/image", 1, &MainWindow::debugImageCallback, this);
  detections_subscriber_ =
      node_->subscribe("/fod/detections", 2, &MainWindow::detectionsCallback, this);
  mode_state_subscriber_ = node_->subscribe(
      "/fod_navigation_mode/state", 10, &MainWindow::modeStateCallback, this);
  mode_status_subscriber_ = node_->subscribe(
      "/fod_navigation_mode/status", 10, &MainWindow::modeStatusCallback, this);
  visual_state_subscriber_ = node_->subscribe(
      "/fod_visual_servo/state", 10, &MainWindow::visualStateCallback, this);
  visual_status_subscriber_ = node_->subscribe(
      "/fod_visual_servo/status", 10, &MainWindow::visualStatusCallback, this);
  diagnostics_subscriber_ =
      node_->subscribe("/diagnostics", 30, &MainWindow::diagnosticsCallback, this);
  map_subscriber_ = node_->subscribe("/map", 1, &MainWindow::mapCallback, this);
  coverage_point_subscriber_ = node_->subscribe(
      "/coverage/clicked_point", 20, &MainWindow::coveragePointCallback, this);
  coverage_status_subscriber_ = node_->subscribe(
      "/coverage/status", 10, &MainWindow::coverageStatusCallback, this);

  relative_goal_publisher_ =
      node_->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, false);
  cancel_publisher_ = node_->advertise<actionlib_msgs::GoalID>("/move_base/cancel", 10, false);
  coverage_draft_publisher_ = node_->advertise<visualization_msgs::MarkerArray>(
      "/coverage/ui_markers", 1, true);
  map_display_status_publisher_ = node_->advertise<std_msgs::String>(
      "/autolabor_operator_gui/map_display_status", 1, true);
  map_display_status_.clear();
  publishMapDisplayStatus(static_map_mode_ && enable_rviz_
                              ? "WAITING_RVIZ"
                              : "DISABLED");
  ros_interfaces_ready_ = true;
  appendEvent(QStringLiteral("ROS 接口已注册；FAST-LIO 健康监测开始采样。"));
  // Build only the RViz belonging to the visible tab.  Creating the coverage
  // VisualizationFrame while its tab is hidden produces an unexposed native
  // Ogre surface on Jetson/X11 and can leave both embedded views black.
  const int active_tab = tabs_ ? tabs_->currentIndex() : overview_tab_index_;
  if (active_tab == coverage_tab_index_)
    setupCoverageRviz();
  else if (active_tab == overview_tab_index_)
    setupEmbeddedRviz();
}

void MainWindow::setupEmbeddedRviz()
{
  if (!rviz_layout_ || !coverage_rviz_layout_)
    return;
  const int active_tab = tabs_ ? tabs_->currentIndex() : overview_tab_index_;
  if (active_tab != overview_tab_index_ && active_tab != coverage_tab_index_)
    return;
  if (rviz_initialized_)
  {
    attachRvizToTab(active_tab);
    return;
  }
  if (!enable_rviz_)
  {
    QLabel* placeholder = active_tab == coverage_tab_index_
                              ? coverage_rviz_placeholder_
                              : rviz_placeholder_;
    if (placeholder)
      placeholder->setText(QStringLiteral("已通过 enable_rviz:=false 禁用嵌入式 RViz"));
    return;
  }
  QWidget* target_host = active_tab == coverage_tab_index_
                             ? coverage_rviz_host_
                             : rviz_host_;
  QVBoxLayout* target_layout = active_tab == coverage_tab_index_
                                   ? coverage_rviz_layout_
                                   : rviz_layout_;
  try
  {
    // Keep exactly one VisualizationFrame/VisualizationManager in this
    // process.  Multiple managers share Ogre::Root and the hidden coverage
    // frame used to make both native render surfaces intermittently black.
    rviz_frame_ = new rviz::VisualizationFrame(target_host);
    rviz_frame_->setWindowFlags(Qt::Widget);
    rviz_frame_->setSplashPath(QString());
    rviz_frame_->setShowChooseNewMaster(false);
    rviz_frame_->initialize(QString::fromStdString(rviz_config_path_));
    // The render panel owns a native Ogre surface.  Keep it outside Qt's
    // styled-background painting and explicitly request the first frame after
    // it is attached to the visible tab.
    if (rviz_frame_->getManager() && rviz_frame_->getManager()->getRenderPanel())
    {
      rviz::RenderPanel* render_panel = rviz_frame_->getManager()->getRenderPanel();
      render_panel->setAutoFillBackground(false);
      render_panel->setAttribute(Qt::WA_StyledBackground, false);
    }
    if (rviz_frame_->getManager() && !rviz_startup_fixed_frame_.empty())
    {
      rviz_frame_->getManager()->setFixedFrame(
          QString::fromStdString(rviz_startup_fixed_frame_));
    }
    if (rviz_frame_->getManager() && rviz_frame_->getManager()->getToolManager())
    {
      connect(rviz_frame_->getManager()->getToolManager(),
              &rviz::ToolManager::toolChanged, this,
              &MainWindow::handleOverviewRvizToolChanged);
    }
    bool found_left_dock = false;
    for (QDockWidget* dock : rviz_frame_->findChildren<QDockWidget*>())
    {
      if (rviz_frame_->dockWidgetArea(dock) == Qt::LeftDockWidgetArea)
      {
        dock->hide();
        found_left_dock = true;
      }
    }
    rviz_frame_->setHideButtonVisibility(false);
    if (rviz_panels_button_)
    {
      rviz_panels_button_->setEnabled(found_left_dock);
      rviz_panels_button_->setText(QStringLiteral("显示 RViz 调试面板"));
    }
    if (rviz_frame_->menuBar())
      rviz_frame_->menuBar()->hide();
    if (rviz_frame_->statusBar())
      rviz_frame_->statusBar()->hide();
    if (active_tab == overview_tab_index_ && rviz_placeholder_)
    {
      rviz_layout_->removeWidget(rviz_placeholder_);
      rviz_placeholder_->deleteLater();
      rviz_placeholder_ = nullptr;
    }
    if (active_tab == coverage_tab_index_ && coverage_rviz_placeholder_)
    {
      coverage_rviz_layout_->removeWidget(coverage_rviz_placeholder_);
      coverage_rviz_placeholder_->deleteLater();
      coverage_rviz_placeholder_ = nullptr;
    }
    target_layout->addWidget(rviz_frame_, 1);
    rviz_frame_->show();
    target_layout->activate();
    if (rviz_frame_->getManager() && rviz_frame_->getManager()->getRenderPanel())
    {
      rviz_frame_->getManager()->getRenderPanel()->show();
      rviz_frame_->getManager()->getRenderPanel()->update();
      rviz_frame_->getManager()->queueRender();
    }
    rviz_initialized_ = true;
    rviz_attached_tab_index_ = active_tab;
    overview_fitted_map_count_ = 0;
    coverage_fitted_map_count_ = 0;
    rviz_map_refresh_message_count_ = 0;
    rviz_map_ready_message_count_ = 0;
    rviz_map_refresh_attempts_ = 0;
    rviz_map_refresh_at_ = ros::WallTime();
    rviz_follow_after_initial_pose_ = false;
    overview_3d_map_enabled_ = false;
    if (rviz_3d_map_button_)
    {
      rviz_3d_map_button_->setChecked(false);
      rviz_3d_map_button_->setText(QStringLiteral("④ 显示静态三维先验"));
    }
    appendEvent(
        (active_tab == coverage_tab_index_
             ? QStringLiteral("清扫页共享 RViz 已加载：")
             : QStringLiteral("嵌入式 RViz 已加载：")) +
        QString::fromStdString(rviz_config_path_) +
        QStringLiteral("（启动坐标系 ") +
        QString::fromStdString(rviz_startup_fixed_frame_) +
        QStringLiteral("）"));
  }
  catch (const std::exception& error)
  {
    appendEvent(QStringLiteral("RViz 加载失败：") + QString::fromLocal8Bit(error.what()), true);
    if (rviz_frame_)
    {
      rviz_frame_->deleteLater();
      rviz_frame_ = nullptr;
    }
    rviz_initialized_ = false;
    rviz_attached_tab_index_ = -1;
    QLabel* placeholder = active_tab == coverage_tab_index_
                              ? coverage_rviz_placeholder_
                              : rviz_placeholder_;
    if (placeholder)
      placeholder->setText(QStringLiteral("RViz 加载失败；其他功能仍可使用"));
  }
}

void MainWindow::attachRvizToTab(int tab_index)
{
  if (!rviz_initialized_ || !rviz_frame_ ||
      (tab_index != overview_tab_index_ && tab_index != coverage_tab_index_) ||
      rviz_attached_tab_index_ == tab_index)
    return;

  QWidget* target_host = tab_index == coverage_tab_index_
                             ? coverage_rviz_host_
                             : rviz_host_;
  QVBoxLayout* target_layout = tab_index == coverage_tab_index_
                                   ? coverage_rviz_layout_
                                   : rviz_layout_;
  if (!target_host || !target_layout)
    return;

  rviz_frame_->hide();
  rviz_layout_->removeWidget(rviz_frame_);
  coverage_rviz_layout_->removeWidget(rviz_frame_);
  if (tab_index == overview_tab_index_ && rviz_placeholder_)
  {
    rviz_layout_->removeWidget(rviz_placeholder_);
    rviz_placeholder_->deleteLater();
    rviz_placeholder_ = nullptr;
  }
  if (tab_index == coverage_tab_index_ && coverage_rviz_placeholder_)
  {
    coverage_rviz_layout_->removeWidget(coverage_rviz_placeholder_);
    coverage_rviz_placeholder_->deleteLater();
    coverage_rviz_placeholder_ = nullptr;
  }
  rviz_frame_->setParent(target_host, Qt::Widget);
  target_layout->addWidget(rviz_frame_, 1);
  rviz_frame_->show();
  target_layout->activate();
  rviz_attached_tab_index_ = tab_index;
  overview_fitted_map_count_ = 0;
  coverage_fitted_map_count_ = 0;

  const TelemetrySnapshot data = snapshot();
  if (tab_index == coverage_tab_index_ && data.map_received)
  {
    setOverview3dMapView(false, data);
    if (setRvizFollowVehicleView(data))
      appendEvent(QStringLiteral("清扫页已自动切换到局部跟车视角。"));
  }
  bool resume_coverage_selection = false;
  if (tab_index == coverage_tab_index_)
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    resume_coverage_selection = coverage_selecting_;
  }
  selectRvizTool(
      rviz_frame_,
      resume_coverage_selection ? QStringLiteral("rviz/PublishPoint")
                                : QStringLiteral("rviz/MoveCamera"));
  if (rviz_frame_->getManager())
  {
    rviz_frame_->getManager()->setFixedFrame(QStringLiteral("map"));
    if (rviz_frame_->getManager()->getRenderPanel())
    {
      rviz_frame_->getManager()->getRenderPanel()->show();
      rviz_frame_->getManager()->getRenderPanel()->update();
    }
    rviz_frame_->getManager()->queueRender();
  }
  appendEvent(tab_index == coverage_tab_index_
                  ? QStringLiteral("共享 RViz 已切换到清扫页。")
                  : QStringLiteral("共享 RViz 已返回综合页。"));
}

void MainWindow::publishMapDisplayStatus(const std::string& status)
{
  if (!map_display_status_publisher_ || status == map_display_status_)
    return;
  std_msgs::String message;
  message.data = status;
  map_display_status_publisher_.publish(message);
  map_display_status_ = status;
}

bool MainWindow::ensureStaticMapDisplayReady(const TelemetrySnapshot& data)
{
  if (!static_map_mode_ || !enable_rviz_)
  {
    publishMapDisplayStatus("DISABLED");
    return false;
  }
  if (!rviz_initialized_ || !rviz_frame_ || !rviz_frame_->getManager())
  {
    publishMapDisplayStatus("WAITING_RVIZ");
    return false;
  }
  if (!data.map_received || data.map_message_count == 0)
  {
    publishMapDisplayStatus("WAITING_MAP");
    return false;
  }

  auto* map_display = dynamic_cast<rviz::MapDisplay*>(findDisplayByName(
      rviz_frame_->getManager()->getRootDisplayGroup(),
      QString::fromLatin1(kStaticMapDisplayName)));
  if (!map_display)
  {
    publishMapDisplayStatus("ERROR;reason=display_missing");
    return false;
  }
  if (!map_display->isEnabled())
  {
    publishMapDisplayStatus("ERROR;reason=display_disabled");
    return false;
  }

  if (rviz_map_refresh_message_count_ != data.map_message_count)
  {
    rviz_map_refresh_message_count_ = data.map_message_count;
    rviz_map_ready_message_count_ = 0;
    rviz_map_refresh_attempts_ = 0;
    rviz_map_refresh_at_ = ros::WallTime();
  }

  const bool dimensions_match =
      map_display->getWidth() == static_cast<int>(data.map_width) &&
      map_display->getHeight() == static_cast<int>(data.map_height) &&
      std::abs(map_display->getResolution() - data.map_resolution) <= 1e-6;
  // The GUI's telemetry subscriber and the embedded rviz/Map display are two
  // independent subscribers.  On Jetson/X11 the latter can miss map_server's
  // first latched delivery while VisualizationFrame is being embedded.  Force
  // one clean resubscription per full map, then verify MapDisplay's own loaded
  // dimensions instead of treating the telemetry callback as rendered output.
  if (rviz_map_refresh_attempts_ > 0 && dimensions_match)
  {
    if (rviz_map_ready_message_count_ != data.map_message_count)
    {
      rviz_map_ready_message_count_ = data.map_message_count;
      appendEvent(QStringLiteral(
          "嵌入式 RViz 已确认收到 /map，二维地图纹理与尺寸均已就绪。"));
    }
    publishMapDisplayStatus(
        "READY;width=" + std::to_string(data.map_width) +
        ";height=" + std::to_string(data.map_height) +
        ";resolution=" + std::to_string(data.map_resolution));
    return true;
  }

  const bool refresh_due =
      rviz_map_refresh_attempts_ == 0 ||
      (ros::WallTime::now() - rviz_map_refresh_at_).toSec() >=
          kMapDisplayRefreshIntervalSeconds;
  if (refresh_due &&
      rviz_map_refresh_attempts_ < kMapDisplayMaxRefreshAttempts)
  {
    ++rviz_map_refresh_attempts_;
    rviz_map_refresh_at_ = ros::WallTime::now();
    map_display->setEnabled(false);
    map_display->setEnabled(true);
    rviz_frame_->getManager()->queueRender();
    publishMapDisplayStatus(
        "RESUBSCRIBING;attempt=" + std::to_string(rviz_map_refresh_attempts_));
    if (rviz_map_refresh_attempts_ == 1)
    {
      if (rviz_map_instruction_)
        rviz_map_instruction_->setText(
            QStringLiteral("已收到 /map，正在确认嵌入式 RViz 二维地图显示……"));
      appendEvent(QStringLiteral(
          "检测到嵌入式 RViz 尚未接收锁存地图，正在自动重订阅 /map。"));
    }
    return false;
  }

  if (rviz_map_refresh_attempts_ >= kMapDisplayMaxRefreshAttempts)
  {
    if (map_display_status_ != "ERROR;reason=no_map_after_resubscribe")
      appendEvent(QStringLiteral(
          "嵌入式 RViz 多次重订阅后仍未收到 /map；启动自检将阻止误报就绪。"),
                  true);
    publishMapDisplayStatus("ERROR;reason=no_map_after_resubscribe");
  }
  else
  {
    publishMapDisplayStatus(
        "WAITING_RVIZ_MAP;attempt=" + std::to_string(rviz_map_refresh_attempts_));
  }
  return false;
}

bool MainWindow::fitRvizMapView(rviz::VisualizationFrame* frame,
                                const TelemetrySnapshot& data)
{
  if (!frame || !frame->getManager() || !frame->getManager()->getViewManager() ||
      !data.map_received || data.map_width == 0 || data.map_height == 0 ||
      !std::isfinite(data.map_resolution) || data.map_resolution <= 0.0 ||
      !std::isfinite(data.map_origin_x) || !std::isfinite(data.map_origin_y) ||
      !std::isfinite(data.map_origin_yaw))
    return false;

  rviz::ViewController* view = frame->getManager()->getViewManager()->getCurrent();
  rviz::RenderPanel* render_panel = frame->getManager()->getRenderPanel();
  if (!view || !render_panel || view->getClassId() != QStringLiteral("rviz/TopDownOrtho"))
    return false;

  // A newly added frame reports RViz's 100x30 fallback size until the tab's
  // layout pass completes.  Do not consume map_message_count using that size:
  // refreshUi() will retry and fit the map once the real viewport is ready.
  if (render_panel->width() < 200 || render_panel->height() < 120)
    return false;

  const double map_width_m = data.map_width * data.map_resolution;
  const double map_height_m = data.map_height * data.map_resolution;
  const double cosine = std::cos(data.map_origin_yaw);
  const double sine = std::sin(data.map_origin_yaw);
  const double center_x = data.map_origin_x + cosine * map_width_m * 0.5 -
                          sine * map_height_m * 0.5;
  const double center_y = data.map_origin_y + sine * map_width_m * 0.5 +
                          cosine * map_height_m * 0.5;
  const double axis_width_m = std::abs(cosine) * map_width_m +
                              std::abs(sine) * map_height_m;
  const double axis_height_m = std::abs(sine) * map_width_m +
                               std::abs(cosine) * map_height_m;
  if (!std::isfinite(center_x) || !std::isfinite(center_y) ||
      axis_width_m <= 0.0 || axis_height_m <= 0.0)
    return false;

  // TopDownOrtho Scale is pixels per metre. Leave a 10% border so the
  // operator can see the complete occupancy grid before choosing /initialpose.
  const double width_pixels = render_panel->width();
  const double height_pixels = render_panel->height();
  const double scale = std::max(
      0.05, std::min(1000.0, 0.90 * std::min(width_pixels / axis_width_m,
                                             height_pixels / axis_height_m)));

  frame->getManager()->setFixedFrame(QStringLiteral("map"));
  view->subProp(QStringLiteral("Target Frame"))->setValue(QStringLiteral("map"));
  view->subProp(QStringLiteral("X"))->setValue(center_x);
  view->subProp(QStringLiteral("Y"))->setValue(center_y);
  view->subProp(QStringLiteral("Scale"))->setValue(scale);
  frame->getManager()->queueRender();
  return true;
}

bool MainWindow::setRvizFollowVehicleView(const TelemetrySnapshot& data)
{
  const bool localization_fresh =
      data.coverage_status_received &&
      wallAge(data.coverage_status_received_at) <= 2.0 &&
      data.coverage_status.localized;
  if (!static_map_mode_ || !rviz_initialized_ || !rviz_frame_ ||
      !rviz_frame_->getManager() ||
      !rviz_frame_->getManager()->getViewManager() ||
      !data.map_received ||
      rviz_map_ready_message_count_ != data.map_message_count ||
      !localization_fresh ||
      !tf_buffer_.canTransform("map", "base_link", ros::Time(0)))
    return false;

  // First leave any Orbit/static-prior mode through the same reversible path
  // used by the full-map button, then change only the TopDownOrtho target.
  // Fixed Frame remains map so all map, costmap and path displays retain their
  // navigation semantics while the viewport follows the vehicle.
  if (!setOverview3dMapView(false, data))
    return false;

  rviz::VisualizationManager* manager = rviz_frame_->getManager();
  rviz::ViewController* view = manager->getViewManager()->getCurrent();
  rviz::RenderPanel* render_panel = manager->getRenderPanel();
  if (!view || !render_panel ||
      view->getClassId() != QStringLiteral("rviz/TopDownOrtho") ||
      render_panel->width() < 200 || render_panel->height() < 120)
    return false;

  // Match the 20 x 20 m rolling local costmap with a small border.
  const double scale = std::max(
      5.0, std::min(100.0,
                    0.90 * std::min(render_panel->width(), render_panel->height()) /
                        22.0));
  manager->setFixedFrame(QStringLiteral("map"));
  view->subProp(QStringLiteral("Target Frame"))
      ->setValue(QStringLiteral("base_link"));
  view->subProp(QStringLiteral("X"))->setValue(0.0);
  view->subProp(QStringLiteral("Y"))->setValue(0.0);
  view->subProp(QStringLiteral("Scale"))->setValue(scale);
  if (!selectRvizTool(rviz_frame_, QStringLiteral("rviz/MoveCamera")))
    return false;

  overview_fitted_map_count_ = data.map_message_count;
  coverage_fitted_map_count_ = data.map_message_count;
  if (rviz_map_instruction_)
    rviz_map_instruction_->setText(
        QStringLiteral("局部跟车视角：实时点云、局部代价地图和当前路线随车辆更新"));
  manager->queueRender();
  return true;
}

void MainWindow::updateNavigationPathDisplays(const TelemetrySnapshot& data)
{
  if (!rviz_initialized_ || !rviz_frame_ || !rviz_frame_->getManager())
    return;

  bool goal_active = false;
  if (data.navigation_received && wallAge(data.navigation_received_at) <= 2.0)
  {
    for (const actionlib_msgs::GoalStatus& status : data.navigation.status_list)
    {
      if (status.status == actionlib_msgs::GoalStatus::PENDING ||
          status.status == actionlib_msgs::GoalStatus::ACTIVE ||
          status.status == actionlib_msgs::GoalStatus::PREEMPTING ||
          status.status == actionlib_msgs::GoalStatus::RECALLING)
      {
        goal_active = true;
        break;
      }
    }
  }

  rviz::DisplayGroup* root = rviz_frame_->getManager()->getRootDisplayGroup();
  for (const char* display_name :
       { kTebGlobalPlanDisplayName, kTebLocalPlanDisplayName })
  {
    rviz::Display* display =
        findDisplayByName(root, QString::fromLatin1(display_name));
    if (display && display->isEnabled() != goal_active)
      display->setEnabled(goal_active);
  }
}

bool MainWindow::selectRvizTool(rviz::VisualizationFrame* frame,
                                const QString& class_id)
{
  if (!frame || !frame->getManager() || !frame->getManager()->getToolManager())
    return false;
  rviz::ToolManager* manager = frame->getManager()->getToolManager();
  for (int index = 0; index < manager->numTools(); ++index)
  {
    rviz::Tool* tool = manager->getTool(index);
    if (tool && tool->getClassId() == class_id)
    {
      manager->setCurrentTool(tool);
      if (frame->getManager()->getRenderPanel())
        frame->getManager()->getRenderPanel()->setFocus(Qt::OtherFocusReason);
      return true;
    }
  }
  return false;
}

bool MainWindow::setOverview3dMapView(bool enabled,
                                      const TelemetrySnapshot& data)
{
  if (!static_map_mode_ || !rviz_initialized_ || !rviz_frame_ ||
      !rviz_frame_->getManager() ||
      !rviz_frame_->getManager()->getViewManager() || !data.map_received ||
      rviz_map_ready_message_count_ != data.map_message_count ||
      data.map_width == 0 || data.map_height == 0 ||
      !std::isfinite(data.map_resolution) || data.map_resolution <= 0.0 ||
      !std::isfinite(data.map_origin_x) || !std::isfinite(data.map_origin_y) ||
      !std::isfinite(data.map_origin_yaw))
    return false;

  rviz::VisualizationManager* manager = rviz_frame_->getManager();
  rviz::Display* prior_map = findDisplayByName(
      manager->getRootDisplayGroup(), QString::fromLatin1(kPriorMapDisplayName));
  if (enabled && !prior_map)
    return false;

  rviz::ViewManager* view_manager = manager->getViewManager();
  auto update_controls = [this](bool active, const QString& instruction) {
    overview_3d_map_enabled_ = active;
    if (rviz_3d_map_button_)
    {
      rviz_3d_map_button_->setChecked(active);
      rviz_3d_map_button_->setText(
          active ? QStringLiteral("返回二维地图")
                 : QStringLiteral("④ 显示静态三维先验"));
    }
    if (rviz_map_instruction_)
      rviz_map_instruction_->setText(instruction);
  };
  auto restore_2d = [&]() {
    if (prior_map)
      prior_map->setEnabled(false);
    view_manager->setCurrentViewControllerType(QStringLiteral("rviz/TopDownOrtho"));
    rviz::ViewController* view = view_manager->getCurrent();
    const bool top_down_ready =
        view && view->getClassId() == QStringLiteral("rviz/TopDownOrtho");
    const bool fitted = top_down_ready && fitRvizMapView(rviz_frame_, data);
    update_controls(
        false, QStringLiteral("二维地图已完整显示；可按车辆真实位置设置初始位姿"));
    manager->queueRender();
    return fitted;
  };

  manager->setFixedFrame(QStringLiteral("map"));
  if (!enabled)
  {
    const bool restored = restore_2d();
    if (restored)
      overview_fitted_map_count_ = data.map_message_count;
    return restored;
  }

  // Validate all geometry before changing the display or view controller so a
  // malformed map cannot leave the button and RViz in contradictory states.
  const double map_width_m = data.map_width * data.map_resolution;
  const double map_height_m = data.map_height * data.map_resolution;
  const double cosine = std::cos(data.map_origin_yaw);
  const double sine = std::sin(data.map_origin_yaw);
  const double center_x = data.map_origin_x + cosine * map_width_m * 0.5 -
                          sine * map_height_m * 0.5;
  const double center_y = data.map_origin_y + sine * map_width_m * 0.5 +
                          cosine * map_height_m * 0.5;
  const double distance =
      std::max(8.0, 1.15 * std::hypot(map_width_m, map_height_m));
  if (!std::isfinite(map_width_m) || !std::isfinite(map_height_m) ||
      map_width_m <= 0.0 || map_height_m <= 0.0 ||
      !std::isfinite(center_x) || !std::isfinite(center_y) ||
      !std::isfinite(distance))
    return false;

  view_manager->setCurrentViewControllerType(QStringLiteral("rviz/Orbit"));
  rviz::ViewController* view = view_manager->getCurrent();
  if (!view || view->getClassId() != QStringLiteral("rviz/Orbit"))
  {
    restore_2d();
    return false;
  }

  view->subProp(QStringLiteral("Target Frame"))->setValue(QStringLiteral("map"));
  view->subProp(QStringLiteral("Focal Point"))
      ->subProp(QStringLiteral("X"))
      ->setValue(center_x);
  view->subProp(QStringLiteral("Focal Point"))
      ->subProp(QStringLiteral("Y"))
      ->setValue(center_y);
  view->subProp(QStringLiteral("Focal Point"))
      ->subProp(QStringLiteral("Z"))
      ->setValue(0.0);
  view->subProp(QStringLiteral("Distance"))->setValue(distance);
  view->subProp(QStringLiteral("Pitch"))->setValue(kPi / 4.0);
  view->subProp(QStringLiteral("Yaw"))->setValue(kPi / 4.0);
  if (!selectRvizTool(rviz_frame_, QStringLiteral("rviz/MoveCamera")))
  {
    restore_2d();
    return false;
  }
  prior_map->setEnabled(true);
  update_controls(
      true, QStringLiteral("三维 PCD 显示已开启；拖动鼠标旋转/平移，滚轮缩放"));
  manager->queueRender();
  return true;
}

void MainWindow::toggleOverview3dMap()
{
  rviz_follow_after_initial_pose_ = false;
  const bool requested = rviz_3d_map_button_ && rviz_3d_map_button_->isChecked();
  const TelemetrySnapshot data = snapshot();
  if (!setOverview3dMapView(requested, data))
  {
    if (rviz_3d_map_button_)
      rviz_3d_map_button_->setChecked(overview_3d_map_enabled_);
    QMessageBox::warning(
        this, QStringLiteral("三维地图不可用"),
        QStringLiteral("请确认已使用 --map-set 启动、地图已加载，并且 RViz 配置包含 "
                       "/fast_lio_localization/prior_map。"));
    return;
  }
  appendEvent(requested
                  ? QStringLiteral("已开启三维先验地图显示并切换到可旋转视角；"
                                   "若点云尚未出现，请等待定位器的锁存地图。")
                  : QStringLiteral("已隐藏三维先验地图，并恢复完整二维地图视角。"));
}

void MainWindow::fitOverviewMapView()
{
  rviz_follow_after_initial_pose_ = false;
  const TelemetrySnapshot data = snapshot();
  if (!static_map_mode_ || !rviz_initialized_ || !data.map_received ||
      rviz_map_ready_message_count_ != data.map_message_count)
  {
    QMessageBox::information(
        this, QStringLiteral("全局地图未就绪"),
        QStringLiteral("请使用 --map-set 启动静态地图模式，并等待二维地图加载完成。"));
    return;
  }
  // Always enforce the real RViz state. The debug panel lets operators change
  // the view/display manually, which may not match overview_3d_map_enabled_.
  if (!setOverview3dMapView(false, data))
  {
    QMessageBox::warning(this, QStringLiteral("无法返回二维地图"),
                         QStringLiteral("RViz 三维显示未能安全关闭，请查看调试面板。"));
    return;
  }
  overview_fitted_map_count_ = data.map_message_count;
  if (rviz_map_instruction_)
    rviz_map_instruction_->setText(
        QStringLiteral("整张地图已显示；下一步点击“② 设置初始位姿”"));
  appendEvent(QStringLiteral("综合页 RViz 已重新居中并缩放到完整二维地图。"));
}

void MainWindow::followOverviewVehicle()
{
  const TelemetrySnapshot data = snapshot();
  if (!setRvizFollowVehicleView(data))
  {
    QMessageBox::information(
        this, QStringLiteral("局部跟车视角未就绪"),
        QStringLiteral("请等待二维地图显示为 READY、三维 ICP 状态达到 LOCALIZED，"
                       "并确认 map 到 base_link 的 TF 可用。"));
    return;
  }
  rviz_follow_after_initial_pose_ = false;
  appendEvent(QStringLiteral(
      "已切换到局部跟车视角；Fixed Frame 保持 map，视角 Target Frame 为 base_link。"));
}

void MainWindow::selectInitialPoseTool()
{
  rviz_follow_after_initial_pose_ = false;
  const TelemetrySnapshot data = snapshot();
  if (!static_map_mode_ || !rviz_initialized_ || !data.map_received ||
      rviz_map_ready_message_count_ != data.map_message_count)
  {
    QMessageBox::information(
        this, QStringLiteral("全局地图未就绪"),
        QStringLiteral("请使用 --map-set 启动静态地图模式，并等待二维地图加载完成。"));
    return;
  }
  // SetInitialPose is deliberately a two-dimensional operation even if the
  // operator changed the embedded RViz view through its debug panel.
  if (!setOverview3dMapView(false, data))
  {
    QMessageBox::warning(this, QStringLiteral("无法设置初始位姿"),
                         QStringLiteral("请先关闭三维地图并返回二维视角。"));
    return;
  }
  overview_fitted_map_count_ = data.map_message_count;
  if (!selectRvizTool(rviz_frame_, QStringLiteral("rviz/SetInitialPose")))
  {
    QMessageBox::warning(this, QStringLiteral("初始位姿工具不可用"),
                         QStringLiteral("RViz 配置中未加载 SetInitialPose 工具。"));
    return;
  }
  appendEvent(QStringLiteral(
      "已选择初始位姿工具；请在二维地图上车辆真实位置按下鼠标，并沿车头方向拖动后松开。"));
}

void MainWindow::handleOverviewRvizToolChanged(rviz::Tool* tool)
{
  const TelemetrySnapshot data = snapshot();
  const bool coverage_owns_navigation =
      data.coverage_status_received &&
      wallAge(data.coverage_status_received_at) <= 2.0 &&
      data.coverage_status.active;
  if (tool && tool->getClassId() == QStringLiteral("rviz/SetGoal") &&
      (coverage_owns_navigation ||
       rviz_attached_tab_index_ == coverage_tab_index_))
  {
    // The shared VisualizationFrame also carries the overview SetGoal tool.
    // Letting it stay active on the coverage page used to publish a second
    // simple goal and preempt the coverage manager's action goal.
    QTimer::singleShot(0, this, [this]() {
      selectRvizTool(rviz_frame_, QStringLiteral("rviz/MoveCamera"));
    });
    appendEvent(
        coverage_owns_navigation
            ? QStringLiteral("覆盖任务正在独占 move_base，已阻止新的地图导航目标。")
            : QStringLiteral("清扫页不发送普通导航目标，已切回地图浏览工具。"),
        true);
    return;
  }
  const bool active = tool && tool->getClassId() == QStringLiteral("rviz/SetInitialPose");
  const bool initial_pose_tool_just_finished =
      !active && overview_initial_pose_tool_active_;
  if (rviz_initial_pose_button_)
    rviz_initial_pose_button_->setText(
        active ? QStringLiteral("请在地图按住并拖向车头")
               : QStringLiteral("② 设置初始位姿"));
  if (rviz_map_instruction_)
  {
    if (active)
      rviz_map_instruction_->setText(
          QStringLiteral("在车辆真实位置按下鼠标，沿车头方向拖动后松开"));
    else if (initial_pose_tool_just_finished)
      rviz_map_instruction_->setText(
          QStringLiteral("初始位姿工具已退出；等待三维 ICP LOCALIZED 后自动进入跟车视角"));
  }
  if (initial_pose_tool_just_finished)
    rviz_follow_after_initial_pose_ = true;
  overview_initial_pose_tool_active_ = active;
}

void MainWindow::toggleRvizPanels()
{
  if (!rviz_frame_)
    return;
  QList<QDockWidget*> left_docks;
  bool any_visible = false;
  for (QDockWidget* dock : rviz_frame_->findChildren<QDockWidget*>())
  {
    if (rviz_frame_->dockWidgetArea(dock) != Qt::LeftDockWidgetArea)
      continue;
    left_docks.push_back(dock);
    any_visible = any_visible || dock->isVisible();
  }
  const bool show = !any_visible;
  for (QDockWidget* dock : left_docks)
    dock->setVisible(show);
  if (rviz_panels_button_)
    rviz_panels_button_->setText(show ? QStringLiteral("隐藏 RViz 调试面板")
                                      : QStringLiteral("显示 RViz 调试面板"));
}

void MainWindow::setupCoverageRviz()
{
  if (!coverage_rviz_layout_)
    return;
  if (!static_map_mode_)
  {
    if (coverage_rviz_placeholder_)
      coverage_rviz_placeholder_->setText(
          QStringLiteral("未选择实验性全局地图\n现有无图导航基线保持不变"));
    return;
  }
  if (!enable_rviz_)
  {
    if (coverage_rviz_placeholder_)
      coverage_rviz_placeholder_->setText(
          QStringLiteral("已通过 enable_rviz:=false 禁用清扫地图 RViz"));
    return;
  }
  setupEmbeddedRviz();
  if (rviz_initialized_)
    attachRvizToTab(coverage_tab_index_);
}

void MainWindow::shutdownRosInterfaces()
{
  ros_interfaces_ready_ = false;
  odom_subscriber_.shutdown();
  cloud_subscriber_.shutdown();
  imu_subscriber_.shutdown();
  can_subscriber_.shutdown();
  scan_subscriber_.shutdown();
  navigation_subscriber_.shutdown();
  camera_image_subscriber_.shutdown();
  debug_image_subscriber_.shutdown();
  detections_subscriber_.shutdown();
  mode_state_subscriber_.shutdown();
  mode_status_subscriber_.shutdown();
  visual_state_subscriber_.shutdown();
  visual_status_subscriber_.shutdown();
  diagnostics_subscriber_.shutdown();
  map_subscriber_.shutdown();
  coverage_point_subscriber_.shutdown();
  coverage_status_subscriber_.shutdown();
  relative_goal_publisher_.shutdown();
  cancel_publisher_.shutdown();
  coverage_draft_publisher_.shutdown();
  map_display_status_publisher_.shutdown();
  map_display_status_.clear();
  rviz_map_refresh_message_count_ = 0;
  rviz_map_ready_message_count_ = 0;
  rviz_map_refresh_attempts_ = 0;
  rviz_map_refresh_at_ = ros::WallTime();
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    telemetry_.map_received = false;
    telemetry_.coverage_status_received = false;
  }
  tf_listener_.reset();
  tf_buffer_.clear();
  node_.reset();
}

void MainWindow::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  const ros::WallTime now = ros::WallTime::now();
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  telemetry_.odom_rate_hz =
      updateRateEstimate(telemetry_.odom_rate_hz, telemetry_.odom_received_at, now);
  telemetry_.odom_received = true;
  telemetry_.odom = *msg;
  telemetry_.odom_received_at = now;
  ++telemetry_.odom_message_count;

  OdomHealthSample sample;
  sample.received_at = now;
  sample.x = msg->pose.pose.position.x;
  sample.y = msg->pose.pose.position.y;
  sample.z = msg->pose.pose.position.z;
  sample.yaw = yawFromQuaternion(msg->pose.pose.orientation);
  const auto& linear = msg->twist.twist.linear;
  const auto& angular = msg->twist.twist.angular;
  sample.linear_speed = std::sqrt(linear.x * linear.x + linear.y * linear.y +
                                  linear.z * linear.z);
  sample.angular_speed = std::sqrt(angular.x * angular.x + angular.y * angular.y +
                                   angular.z * angular.z);
  if (!odom_health_history_.empty())
  {
    const OdomHealthSample& previous = odom_health_history_.back();
    const double dx = sample.x - previous.x;
    const double dy = sample.y - previous.y;
    const double dz = sample.z - previous.z;
    sample.pose_step_m = std::sqrt(dx * dx + dy * dy + dz * dz);
    sample.yaw_step_deg =
        angleDistanceRadians(sample.yaw, previous.yaw) * 180.0 / kPi;
  }
  odom_health_history_.push_back(sample);
  while (!odom_health_history_.empty() &&
         (now - odom_health_history_.front().received_at).toSec() > 10.0)
    odom_health_history_.pop_front();

  telemetry_.recent_odom_distance_m = 0.0;
  telemetry_.recent_odom_sample_count = odom_health_history_.size();
  telemetry_.recent_odom_window_seconds =
      odom_health_history_.empty()
          ? 0.0
          : (now - odom_health_history_.front().received_at).toSec();
  for (const OdomHealthSample& history_sample : odom_health_history_)
  {
    if (std::isfinite(history_sample.pose_step_m))
      telemetry_.recent_odom_distance_m += history_sample.pose_step_m;
  }

  telemetry_.recent_pose_step_m = 0.0;
  telemetry_.recent_yaw_step_deg = 0.0;
  for (auto sample_it = odom_health_history_.rbegin();
       sample_it != odom_health_history_.rend(); ++sample_it)
  {
    if ((now - sample_it->received_at).toSec() > 2.0)
      break;
    telemetry_.recent_pose_step_m =
        std::max(telemetry_.recent_pose_step_m, sample_it->pose_step_m);
    telemetry_.recent_yaw_step_deg =
        std::max(telemetry_.recent_yaw_step_deg, sample_it->yaw_step_deg);
  }

  telemetry_.stationary_window_seconds = 0.0;
  telemetry_.stationary_drift_m = 0.0;
  if (sample.linear_speed <= 0.05 && sample.angular_speed <= 0.05)
  {
    auto stationary_begin = odom_health_history_.end();
    for (auto sample_it = odom_health_history_.rbegin();
         sample_it != odom_health_history_.rend(); ++sample_it)
    {
      if (sample_it->linear_speed > 0.05 || sample_it->angular_speed > 0.05)
        break;
      stationary_begin = std::prev(sample_it.base());
    }
    if (stationary_begin != odom_health_history_.end())
    {
      telemetry_.stationary_window_seconds =
          (now - stationary_begin->received_at).toSec();
      double min_x = stationary_begin->x;
      double max_x = stationary_begin->x;
      double min_y = stationary_begin->y;
      double max_y = stationary_begin->y;
      double min_z = stationary_begin->z;
      double max_z = stationary_begin->z;
      for (auto sample_it = stationary_begin; sample_it != odom_health_history_.end();
           ++sample_it)
      {
        min_x = std::min(min_x, sample_it->x);
        max_x = std::max(max_x, sample_it->x);
        min_y = std::min(min_y, sample_it->y);
        max_y = std::max(max_y, sample_it->y);
        min_z = std::min(min_z, sample_it->z);
        max_z = std::max(max_z, sample_it->z);
      }
      const double dx = max_x - min_x;
      const double dy = max_y - min_y;
      const double dz = max_z - min_z;
      telemetry_.stationary_drift_m = std::sqrt(dx * dx + dy * dy + dz * dz);
    }
  }
}

void MainWindow::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  const ros::WallTime now = ros::WallTime::now();
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  telemetry_.cloud_rate_hz =
      updateRateEstimate(telemetry_.cloud_rate_hz, telemetry_.cloud_received_at, now);
  telemetry_.cloud_received = true;
  telemetry_.cloud_received_at = now;
  telemetry_.cloud_point_count =
      static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height);
  ++telemetry_.cloud_message_count;
}

void MainWindow::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  const ros::WallTime now = ros::WallTime::now();
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  telemetry_.imu_rate_hz =
      updateRateEstimate(telemetry_.imu_rate_hz, telemetry_.imu_received_at, now);
  telemetry_.imu_received = true;
  telemetry_.imu_received_at = now;
  ++telemetry_.imu_message_count;
  telemetry_.imu_values_finite =
      std::isfinite(msg->angular_velocity.x) &&
      std::isfinite(msg->angular_velocity.y) &&
      std::isfinite(msg->angular_velocity.z) &&
      std::isfinite(msg->linear_acceleration.x) &&
      std::isfinite(msg->linear_acceleration.y) &&
      std::isfinite(msg->linear_acceleration.z);
}

void MainWindow::canCallback(const autolabor_canbus_driver::CanBusMessage::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  telemetry_.can_received = true;
  telemetry_.can = *msg;
  telemetry_.can_received_at = ros::WallTime::now();
}

void MainWindow::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  telemetry_.scan_received = true;
  telemetry_.scan_sample_count = msg->ranges.size();
  telemetry_.scan_received_at = ros::WallTime::now();
}

void MainWindow::navigationCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  telemetry_.navigation_received = true;
  telemetry_.navigation = *msg;
  telemetry_.navigation_received_at = ros::WallTime::now();
}

void MainWindow::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  if (msg->header.frame_id != "map" || msg->info.width == 0 ||
      msg->info.height == 0 || !std::isfinite(msg->info.resolution) ||
      msg->info.resolution <= 0.0 ||
      !std::isfinite(msg->info.origin.position.x) ||
      !std::isfinite(msg->info.origin.position.y) ||
      !std::isfinite(msg->info.origin.orientation.x) ||
      !std::isfinite(msg->info.origin.orientation.y) ||
      !std::isfinite(msg->info.origin.orientation.z) ||
      !std::isfinite(msg->info.origin.orientation.w) ||
      msg->data.size() !=
          static_cast<std::size_t>(msg->info.width) * msg->info.height)
    return;
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  telemetry_.map_received = true;
  telemetry_.map_received_at = ros::WallTime::now();
  ++telemetry_.map_message_count;
  telemetry_.map_width = msg->info.width;
  telemetry_.map_height = msg->info.height;
  telemetry_.map_resolution = msg->info.resolution;
  telemetry_.map_origin_x = msg->info.origin.position.x;
  telemetry_.map_origin_y = msg->info.origin.position.y;
  telemetry_.map_origin_yaw = yawFromQuaternion(msg->info.origin.orientation);
}

void MainWindow::coveragePointCallback(
    const geometry_msgs::PointStamped::ConstPtr& msg)
{
  if (msg->header.frame_id != "map" || !std::isfinite(msg->point.x) ||
      !std::isfinite(msg->point.y))
    return;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    if (!coverage_selecting_)
      return;
    geometry_msgs::Point point = msg->point;
    point.z = 0.0;
    coverage_draft_points_.push_back(point);
  }
  publishCoverageDraft();
}

void MainWindow::coverageStatusCallback(
    const autolabor_coverage::CoverageStatus::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  telemetry_.coverage_status_received = true;
  telemetry_.coverage_status = *msg;
  telemetry_.coverage_status_received_at = ros::WallTime::now();
}

void MainWindow::cameraImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  const ros::WallTime now = ros::WallTime::now();
  bool convert_preview = false;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    telemetry_.camera_received = true;
    telemetry_.camera_width = msg->width;
    telemetry_.camera_height = msg->height;
    telemetry_.camera_encoding = msg->encoding;
    telemetry_.camera_received_at = now;
    const bool debug_is_fresh = telemetry_.debug_image_received &&
                                !telemetry_.debug_image.isNull() &&
                                wallAge(telemetry_.debug_image_received_at) <= 1.0;
    if (!debug_is_fresh &&
        (last_raw_preview_conversion_.isZero() ||
         (now - last_raw_preview_conversion_).toSec() >= 0.20))
    {
      last_raw_preview_conversion_ = now;
      convert_preview = true;
    }
  }
  if (!convert_preview)
    return;
  QImage preview;
  if (!imageMessageToQImage(*msg, &preview))
    return;
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  telemetry_.raw_preview = preview;
  telemetry_.raw_preview_received_at = now;
}

void MainWindow::debugImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  const ros::WallTime now = ros::WallTime::now();
  bool convert_preview = false;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    telemetry_.debug_image_received = true;
    telemetry_.debug_image_received_at = now;
    if (last_debug_preview_conversion_.isZero() ||
        (now - last_debug_preview_conversion_).toSec() >= 0.10)
    {
      last_debug_preview_conversion_ = now;
      convert_preview = true;
    }
  }
  if (!convert_preview)
    return;
  QImage preview;
  if (!imageMessageToQImage(*msg, &preview))
    return;
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  telemetry_.debug_image = preview;
}

void MainWindow::detectionsCallback(
    const autolabor_fod_msgs::FodDetectionArray::ConstPtr& msg)
{
  const ros::WallTime now = ros::WallTime::now();
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  if (telemetry_.detections_received)
  {
    const double interval = (now - telemetry_.detections_received_at).toSec();
    if (interval > 0.001 && interval < 5.0)
    {
      const double sample_fps = 1.0 / interval;
      const double filtered_fps = telemetry_.detection_fps.received
                                      ? 0.80 * telemetry_.detection_fps.value +
                                            0.20 * sample_fps
                                      : sample_fps;
      telemetry_.detection_fps = { true, filtered_fps, now };
    }
  }
  telemetry_.detections_received = true;
  telemetry_.detections = *msg;
  telemetry_.detections_received_at = now;
}

void MainWindow::modeStateCallback(const std_msgs::String::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  telemetry_.mode_state_received = true;
  telemetry_.mode_state = msg->data;
  telemetry_.mode_state_received_at = ros::WallTime::now();
}

void MainWindow::modeStatusCallback(const std_msgs::String::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  telemetry_.mode_status_received = true;
  telemetry_.mode_status = msg->data;
  telemetry_.mode_status_received_at = ros::WallTime::now();
}

void MainWindow::visualStateCallback(const std_msgs::String::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  telemetry_.visual_state_received = true;
  telemetry_.visual_state = msg->data;
  telemetry_.visual_state_received_at = ros::WallTime::now();
}

void MainWindow::visualStatusCallback(const std_msgs::String::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  telemetry_.visual_status_received = true;
  telemetry_.visual_status = msg->data;
  telemetry_.visual_status_received_at = ros::WallTime::now();
}

void MainWindow::diagnosticsCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg)
{
  const ros::WallTime now = ros::WallTime::now();
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  for (const auto& status : msg->status)
  {
    DiagnosticSnapshot* output = nullptr;
    if (status.name.find("fod_vision/detector") != std::string::npos)
      output = &telemetry_.detector_diagnostic;
    else if (status.name.find("fod_vision/image_quality_controller") !=
             std::string::npos)
      output = &telemetry_.image_quality_diagnostic;
    if (!output)
      continue;
    output->received = true;
    output->level = status.level;
    output->message = status.message;
    output->values.clear();
    for (const auto& value : status.values)
      output->values[value.key] = value.value;
    output->received_at = now;
  }
}

TelemetrySnapshot MainWindow::snapshot() const
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return telemetry_;
}

double MainWindow::wallAge(const ros::WallTime& stamp)
{
  if (stamp.isZero())
    return std::numeric_limits<double>::infinity();
  return std::max(0.0, (ros::WallTime::now() - stamp).toSec());
}

double MainWindow::yawFromQuaternion(const geometry_msgs::Quaternion& q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

QString MainWindow::navigationState(const actionlib_msgs::GoalStatusArray& status)
{
  if (status.status_list.empty())
    return QStringLiteral("空闲");
  const auto& last = status.status_list.back();
  switch (last.status)
  {
    case actionlib_msgs::GoalStatus::PENDING:
      return QStringLiteral("等待执行");
    case actionlib_msgs::GoalStatus::ACTIVE:
      return QStringLiteral("执行中");
    case actionlib_msgs::GoalStatus::PREEMPTED:
      return QStringLiteral("已取消");
    case actionlib_msgs::GoalStatus::SUCCEEDED:
      return QStringLiteral("已到达");
    case actionlib_msgs::GoalStatus::ABORTED:
      return QStringLiteral("已中止");
    case actionlib_msgs::GoalStatus::REJECTED:
      return QStringLiteral("已拒绝");
    case actionlib_msgs::GoalStatus::PREEMPTING:
      return QStringLiteral("取消中");
    case actionlib_msgs::GoalStatus::RECALLING:
      return QStringLiteral("召回中");
    case actionlib_msgs::GoalStatus::RECALLED:
      return QStringLiteral("已召回");
    case actionlib_msgs::GoalStatus::LOST:
      return QStringLiteral("状态丢失");
    default:
      return QStringLiteral("未知");
  }
}

QString MainWindow::ageText(double age_seconds)
{
  if (!std::isfinite(age_seconds))
    return QStringLiteral("未收到");
  if (age_seconds < 1.0)
    return QStringLiteral("%1 ms").arg(age_seconds * 1000.0, 0, 'f', 0);
  return QStringLiteral("%1 s").arg(age_seconds, 0, 'f', 1);
}

bool MainWindow::imageMessageToQImage(const sensor_msgs::Image& message, QImage* image)
{
  if (!image || message.width == 0 || message.height == 0 || message.step == 0 ||
      message.width > static_cast<unsigned int>(std::numeric_limits<int>::max()) ||
      message.height > static_cast<unsigned int>(std::numeric_limits<int>::max()) ||
      message.step > static_cast<unsigned int>(std::numeric_limits<int>::max()))
    return false;
  if (message.height > std::numeric_limits<std::size_t>::max() / message.step ||
      message.data.size() < static_cast<std::size_t>(message.height) * message.step)
    return false;

  const int width = static_cast<int>(message.width);
  const int height = static_cast<int>(message.height);
  const int step = static_cast<int>(message.step);
  if (message.encoding == sensor_msgs::image_encodings::BGR8)
  {
    if (static_cast<std::size_t>(message.step) <
        static_cast<std::size_t>(message.width) * 3U)
      return false;
    const QImage wrapped(message.data.data(), width, height, step, QImage::Format_RGB888);
    *image = wrapped.rgbSwapped();
    return !image->isNull();
  }
  if (message.encoding == sensor_msgs::image_encodings::RGB8)
  {
    if (static_cast<std::size_t>(message.step) <
        static_cast<std::size_t>(message.width) * 3U)
      return false;
    const QImage wrapped(message.data.data(), width, height, step, QImage::Format_RGB888);
    *image = wrapped.copy();
    return !image->isNull();
  }
  if (message.encoding == sensor_msgs::image_encodings::BGRA8)
  {
    if (static_cast<std::size_t>(message.step) <
        static_cast<std::size_t>(message.width) * 4U)
      return false;
    // ZED publishes BGRA8. QImage's RGBA8888 byte layout lets rgbSwapped()
    // correct the channel order before alpha is deliberately discarded.
    const QImage wrapped(message.data.data(), width, height, step, QImage::Format_RGBA8888);
    *image = wrapped.rgbSwapped().convertToFormat(QImage::Format_RGB888);
    return !image->isNull();
  }
  if (message.encoding == sensor_msgs::image_encodings::RGBA8)
  {
    if (static_cast<std::size_t>(message.step) <
        static_cast<std::size_t>(message.width) * 4U)
      return false;
    const QImage wrapped(message.data.data(), width, height, step, QImage::Format_RGBA8888);
    *image = wrapped.convertToFormat(QImage::Format_RGB888);
    return !image->isNull();
  }
  if (message.encoding == sensor_msgs::image_encodings::MONO8)
  {
    if (message.step < message.width)
      return false;
    const QImage wrapped(message.data.data(), width, height, step, QImage::Format_Grayscale8);
    *image = wrapped.copy();
    return !image->isNull();
  }
  return false;
}

MainWindow::FastLioHealthResult MainWindow::evaluateFastLioHealth(
    const TelemetrySnapshot& data) const
{
  FastLioHealthResult result;
  result.position_sigma_m = std::numeric_limits<double>::quiet_NaN();
  result.yaw_sigma_deg = std::numeric_limits<double>::quiet_NaN();

  const double odom_age = wallAge(data.odom_received_at);
  const double cloud_age = wallAge(data.cloud_received_at);
  const double imu_age = wallAge(data.imu_received_at);
  int score = 0;

  auto scoreFreshness = [&result](bool received, double age, double fresh_limit,
                                  int full_points, const QString& stream) {
    if (!received)
    {
      result.findings << stream + QStringLiteral("未收到");
      return 0;
    }
    if (age <= fresh_limit)
      return full_points;
    if (age <= kFastLioCriticalStreamSeconds)
    {
      result.findings << stream + QStringLiteral("延迟偏高");
      return full_points / 2;
    }
    result.findings << stream + QStringLiteral("已中断");
    return 0;
  };
  auto scoreRate = [&result](double rate, double good_rate, double warning_rate,
                             int full_points, const QString& stream) {
    if (rate >= good_rate)
      return full_points;
    if (rate >= warning_rate)
    {
      result.findings << stream + QStringLiteral("频率偏低");
      return full_points / 2;
    }
    result.findings << stream + QStringLiteral("频率过低");
    return 0;
  };

  score += scoreFreshness(data.odom_received, odom_age, kFastLioFreshOdomSeconds,
                          18, QStringLiteral("里程计"));
  score += scoreRate(data.odom_rate_hz, 8.0, 5.0, 10, QStringLiteral("里程计"));
  score += scoreFreshness(data.cloud_received, cloud_age, kFastLioFreshCloudSeconds,
                          12, QStringLiteral("注册点云"));
  score += scoreRate(data.cloud_rate_hz, 8.0, 5.0, 8, QStringLiteral("注册点云"));
  score += scoreFreshness(data.imu_received, imu_age, kFastLioFreshImuSeconds,
                          10, QStringLiteral("IMU"));
  score += scoreRate(data.imu_rate_hz, 100.0, 50.0, 10, QStringLiteral("IMU"));

  bool pose_valid = false;
  bool covariance_available = false;
  if (data.odom_received)
  {
    const auto& position = data.odom.pose.pose.position;
    const auto& orientation = data.odom.pose.pose.orientation;
    const double quaternion_norm =
        std::sqrt(orientation.x * orientation.x + orientation.y * orientation.y +
                  orientation.z * orientation.z + orientation.w * orientation.w);
    pose_valid = std::isfinite(position.x) && std::isfinite(position.y) &&
                 std::isfinite(position.z) && std::isfinite(quaternion_norm) &&
                 quaternion_norm >= 0.95 && quaternion_norm <= 1.05;
    if (pose_valid)
      score += 8;
    else
      result.findings << QStringLiteral("位姿或四元数非法");

    if (data.recent_pose_step_m <= 0.15 && data.recent_yaw_step_deg <= 5.0)
      score += 8;
    else if (data.recent_pose_step_m <= 0.50 && data.recent_yaw_step_deg <= 20.0)
    {
      score += 4;
      result.findings << QStringLiteral("位姿连续性波动");
    }
    else
      result.findings << QStringLiteral("检测到位姿突跳");

    const auto& covariance = data.odom.pose.covariance;
    covariance_available =
        std::isfinite(covariance[0]) && std::isfinite(covariance[7]) &&
        std::isfinite(covariance[35]) && covariance[0] >= 0.0 &&
        covariance[7] >= 0.0 && covariance[35] >= 0.0 &&
        (covariance[0] > 0.0 || covariance[7] > 0.0 || covariance[35] > 0.0);
    if (covariance_available)
    {
      result.position_sigma_m = std::sqrt(covariance[0] + covariance[7]);
      result.yaw_sigma_deg = std::sqrt(covariance[35]) * 180.0 / kPi;
      if (result.position_sigma_m <= 0.15 && result.yaw_sigma_deg <= 5.0)
        score += 5;
      else if (result.position_sigma_m <= 0.50 && result.yaw_sigma_deg <= 15.0)
      {
        score += 2;
        result.findings << QStringLiteral("内部协方差升高");
      }
      else
        result.findings << QStringLiteral("内部协方差过大");
    }
    else
    {
      score += 2;
      result.findings << QStringLiteral("里程计未提供有效协方差");
    }
  }

  if (data.stationary_window_seconds < kFastLioStationaryWindowSeconds)
  {
    // A moving vehicle or a newly started estimator has no valid stationary
    // window yet. Keep this check neutral instead of reporting a false fault.
    score += 6;
  }
  else if (data.stationary_drift_m <= 0.05)
    score += 6;
  else if (data.stationary_drift_m <= 0.15)
  {
    score += 3;
    result.findings << QStringLiteral("静止漂移偏大");
  }
  else
    result.findings << QStringLiteral("静止漂移过大");

  std::string fixed_frame = rviz_navigation_fixed_frame_;
  if (data.odom_received && !data.odom.header.frame_id.empty())
    fixed_frame = data.odom.header.frame_id;
  result.tf_ready = master_online_ && !fixed_frame.empty() &&
                    tf_buffer_.canTransform(fixed_frame, "base_link", ros::Time(0));
  if (result.tf_ready)
    score += 5;
  else
    result.findings << QStringLiteral("定位 TF 不可用");

  if (data.cloud_received && data.cloud_point_count < 500)
    result.findings << QStringLiteral("注册点云点数过少");
  if (data.imu_received && !data.imu_values_finite)
    result.findings << QStringLiteral("IMU 含非法数值");

  result.score = std::max(0, std::min(100, score));
  result.critical_streams_ready =
      data.odom_received && odom_age <= kFastLioCriticalStreamSeconds &&
      data.cloud_received && cloud_age <= kFastLioCriticalStreamSeconds &&
      data.cloud_point_count >= 500 && data.imu_received &&
      imu_age <= kFastLioCriticalStreamSeconds && data.imu_values_finite && pose_valid;

  if (!result.critical_streams_ready)
  {
    result.health = Health::Bad;
    result.state = QStringLiteral("异常");
  }
  else if (result.score >= 85 && result.tf_ready)
  {
    result.health = Health::Good;
    result.state = QStringLiteral("健康");
  }
  else if (result.score >= 65)
  {
    result.health = Health::Warning;
    result.state = QStringLiteral("注意");
  }
  else
  {
    result.health = Health::Bad;
    result.state = QStringLiteral("异常");
  }

  result.summary = result.findings.isEmpty()
                       ? QStringLiteral("链路、连续性、协方差与静止漂移正常")
                       : result.findings.mid(0, 3).join(QStringLiteral("；"));
  return result;
}

void MainWindow::refreshUi()
{
  const TelemetrySnapshot data = snapshot();
  const FastLioHealthResult fastlio = evaluateFastLioHealth(data);
  const ModeStatusView mode_status =
      data.mode_status_received ? parseModeStatus(data.mode_status) : ModeStatusView();
  const QString mode_state = mode_status.valid && !mode_status.state.isEmpty()
                                 ? mode_status.state
                                 : QString::fromStdString(data.mode_state);
  const double mode_age = data.mode_status_received
                              ? wallAge(data.mode_status_received_at)
                              : wallAge(data.mode_state_received_at);
  const bool embedded_map_ready = ensureStaticMapDisplayReady(data);
  const bool overview_map_ready = static_map_mode_ && rviz_initialized_ &&
                                  data.map_received && data.map_message_count > 0 &&
                                  embedded_map_ready;
  if (rviz_fit_map_button_)
    rviz_fit_map_button_->setEnabled(overview_map_ready);
  if (rviz_initial_pose_button_)
    rviz_initial_pose_button_->setEnabled(overview_map_ready);
  const bool vehicle_view_ready =
      overview_map_ready && data.coverage_status_received &&
      wallAge(data.coverage_status_received_at) <= 2.0 &&
      data.coverage_status.localized &&
      tf_buffer_.canTransform("map", "base_link", ros::Time(0));
  if (rviz_follow_vehicle_button_)
    rviz_follow_vehicle_button_->setEnabled(vehicle_view_ready);
  if (rviz_3d_map_button_)
    rviz_3d_map_button_->setEnabled(overview_map_ready);
  if (rviz_follow_after_initial_pose_ && vehicle_view_ready &&
      rviz_attached_tab_index_ == overview_tab_index_ &&
      setRvizFollowVehicleView(data))
  {
    rviz_follow_after_initial_pose_ = false;
    appendEvent(QStringLiteral(
        "三维 ICP 已 LOCALIZED；综合页已自动切换到局部跟车视角。"));
  }
  updateNavigationPathDisplays(data);
  if (overview_map_ready && rviz_attached_tab_index_ == overview_tab_index_ &&
      !overview_3d_map_enabled_ &&
      overview_fitted_map_count_ != data.map_message_count &&
      fitRvizMapView(rviz_frame_, data))
  {
    overview_fitted_map_count_ = data.map_message_count;
    if (rviz_map_instruction_ && !overview_initial_pose_tool_active_)
      rviz_map_instruction_->setText(
          QStringLiteral("二维地图已完整显示；点击“② 设置初始位姿”后按真实位置拖动车头方向"));
    appendEvent(QStringLiteral("二维静态地图已加载，综合页 RViz 已自动显示完整地图。"));
  }
  if (static_map_mode_ && rviz_initialized_ &&
      rviz_attached_tab_index_ == coverage_tab_index_ && data.map_received &&
      embedded_map_ready &&
      coverage_fitted_map_count_ != data.map_message_count &&
      fitRvizMapView(rviz_frame_, data))
  {
    coverage_fitted_map_count_ = data.map_message_count;
  }
  if (rviz_initialized_ && rviz_frame_ && rviz_frame_->getManager() &&
      data.odom_received && wallAge(data.odom_received_at) <= 2.0 &&
      !rviz_navigation_fixed_frame_.empty())
  {
    const QString navigation_frame =
        QString::fromStdString(rviz_navigation_fixed_frame_);
    if (rviz_frame_->getManager()->getFixedFrame() != navigation_frame)
    {
      rviz_frame_->getManager()->setFixedFrame(navigation_frame);
      appendEvent(QStringLiteral("定位里程计 ") +
                  QString::fromStdString(odom_topic_) +
                  QStringLiteral(" 已就绪；RViz 固定坐标系切换为 ") +
                  navigation_frame);
    }
  }
  setStatus("ros", master_online_ ? Health::Good : Health::Bad,
            master_online_ ? QStringLiteral("在线") : QStringLiteral("离线"),
            master_online_ ? QStringLiteral("master 可达") : QStringLiteral("后台重试"));

  const double can_age = wallAge(data.can_received_at);
  if (!data.can_received)
    setStatus("can", Health::Idle, QStringLiteral("无数据"), QStringLiteral("/canbus_msg"));
  else if (can_age <= 1.0)
    setStatus("can", Health::Good, QStringLiteral("正常"), ageText(can_age));
  else
    setStatus("can", Health::Bad, QStringLiteral("超时"), ageText(can_age));

  setStatus("fastlio", fastlio.health,
            QStringLiteral("%1 · %2分").arg(fastlio.state).arg(fastlio.score),
            fastlio.summary);

  const double cloud_age = wallAge(data.cloud_received_at);
  if (!data.cloud_received)
    setStatus("cloud", Health::Idle, QStringLiteral("无数据"),
              QString::fromStdString(cloud_topic_));
  else if (cloud_age > kFastLioCriticalStreamSeconds)
    setStatus("cloud", Health::Bad, QStringLiteral("中断"), ageText(cloud_age));
  else if (cloud_age > kFastLioFreshCloudSeconds || data.cloud_rate_hz < 8.0 ||
           data.cloud_point_count < 500)
    setStatus("cloud", Health::Warning, QStringLiteral("波动"),
              QStringLiteral("%1 Hz · %2 点")
                  .arg(data.cloud_rate_hz, 0, 'f', 1)
                  .arg(data.cloud_point_count));
  else
    setStatus("cloud", Health::Good, QStringLiteral("正常"),
              QStringLiteral("%1 Hz · %2 点")
                  .arg(data.cloud_rate_hz, 0, 'f', 1)
                  .arg(data.cloud_point_count));

  const double imu_age = wallAge(data.imu_received_at);
  if (!data.imu_received)
    setStatus("imu", Health::Idle, QStringLiteral("无数据"),
              QString::fromStdString(imu_topic_));
  else if (imu_age > kFastLioCriticalStreamSeconds || !data.imu_values_finite)
    setStatus("imu", Health::Bad, QStringLiteral("异常"), ageText(imu_age));
  else if (imu_age > kFastLioFreshImuSeconds || data.imu_rate_hz < 100.0)
    setStatus("imu", Health::Warning, QStringLiteral("波动"),
              QStringLiteral("%1 Hz").arg(data.imu_rate_hz, 0, 'f', 0));
  else
    setStatus("imu", Health::Good, QStringLiteral("正常"),
              QStringLiteral("%1 Hz").arg(data.imu_rate_hz, 0, 'f', 0));

  const double scan_age = wallAge(data.scan_received_at);
  if (!data.scan_received)
    setStatus("scan", Health::Idle, QStringLiteral("无数据"), QStringLiteral("/scan"));
  else if (scan_age <= 1.5)
    setStatus("scan", Health::Good, QStringLiteral("正常"),
              QStringLiteral("%1 点").arg(data.scan_sample_count));
  else
    setStatus("scan", Health::Bad, QStringLiteral("超时"), ageText(scan_age));

  const double nav_age = wallAge(data.navigation_received_at);
  const QString nav_state = navigationState(data.navigation);
  if (!data.navigation_received)
    setStatus("nav", Health::Idle, QStringLiteral("未启动"), QStringLiteral("move_base"));
  else if (nav_age > 2.0)
    setStatus("nav", Health::Bad, QStringLiteral("超时"), ageText(nav_age));
  else
    setStatus("nav", Health::Good, nav_state, ageText(nav_age));

  if (!data.mode_state_received && !data.mode_status_received)
    setStatus("mode", Health::Idle, QStringLiteral("未启动"),
              QStringLiteral("视觉模式可选"));
  else if (mode_age > kFreshModeSeconds)
    setStatus("mode", Health::Bad, QStringLiteral("状态超时"), ageText(mode_age));
  else if (mode_state == QStringLiteral("GPS_ACTIVE"))
    setStatus("mode", Health::Good, QStringLiteral("相对导航"), QStringLiteral("视觉待机"));
  else if (mode_state == QStringLiteral("FOD_ACTIVE"))
    setStatus("mode", Health::Warning, QStringLiteral("视觉"), QStringLiteral("局部路线休眠"));
  else if (mode_state == QStringLiteral("FOD_ABORTED") ||
           mode_state == QStringLiteral("FAULT_STOP"))
    setStatus("mode", Health::Bad, QStringLiteral("停车"), modeDisplayName(mode_state));
  else
    setStatus("mode", Health::Warning, QStringLiteral("切换中"), modeDisplayName(mode_state));

  const double camera_age = wallAge(data.camera_received_at);
  if (!data.camera_received)
    setStatus("camera", Health::Idle, QStringLiteral("未启动"), QStringLiteral("可选模块"));
  else if (camera_age > kFreshCameraSeconds)
    setStatus("camera", Health::Bad, QStringLiteral("断流"), ageText(camera_age));
  else
    setStatus("camera", Health::Good, QStringLiteral("在线"),
              QStringLiteral("%1×%2").arg(data.camera_width).arg(data.camera_height));

  const double detections_age = wallAge(data.detections_received_at);
  if (!data.detections_received)
    setStatus("yolo", Health::Idle, QStringLiteral("未启动"), QStringLiteral("YOLO11"));
  else if (detections_age > kFreshDetectionSeconds)
    setStatus("yolo", Health::Bad, QStringLiteral("超时"), ageText(detections_age));
  else
    setStatus("yolo", Health::Good,
              QStringLiteral("%1 目标").arg(data.detections.detections.size()),
              QStringLiteral("%1 ms").arg(data.detections.inference_ms, 0, 'f', 1));

  if (mapper_.state() != QProcess::NotRunning && mapper_stop_requested_)
    setStatus("record", Health::Warning, QStringLiteral("地图保存中"),
              QStringLiteral("正在保存三类静态地图"));
  else if (mapper_.state() != QProcess::NotRunning)
    setStatus("record", Health::Good, QStringLiteral("静态建图中"),
              QStringLiteral("MID360 三维 + 双 LD19 二维"));
  else if (recorder_.state() != QProcess::NotRunning && recorder_stop_requested_)
    setStatus("record", Health::Warning, QStringLiteral("录包保存中"),
              QStringLiteral("正在关闭 bag"));
  else if (recorder_.state() != QProcess::NotRunning)
    setStatus("record", Health::Good, QStringLiteral("录包中"),
              QStringLiteral("仅记录相关 ROS 话题"));
  else if (recorder_error_ || mapper_error_)
    setStatus("record", Health::Bad, QStringLiteral("异常"), QStringLiteral("查看日志"));
  else
    setStatus("record", Health::Idle, QStringLiteral("未录制"), QStringLiteral("可随时启动"));

  values_["overview_fastlio_health"]->setText(
      QStringLiteral("%1 · %2/100").arg(fastlio.state).arg(fastlio.score));
  values_["overview_fastlio_health"]->setStyleSheet(
      QStringLiteral("font-size:20pt;font-weight:750;color:%1;")
          .arg(statusColor(fastlio.health)));
  values_["overview_fastlio_summary"]->setText(fastlio.summary);
  if (data.odom_received)
  {
    values_["overview_xy"]->setText(
        QStringLiteral("%1, %2 m")
            .arg(data.odom.pose.pose.position.x, 0, 'f', 2)
            .arg(data.odom.pose.pose.position.y, 0, 'f', 2));
    values_["overview_yaw"]->setText(
        numberOrDash(yawFromQuaternion(data.odom.pose.pose.orientation) * 180.0 / kPi, 1));
    const auto& linear = data.odom.twist.twist.linear;
    values_["overview_speed"]->setText(
        numberOrDash(std::sqrt(linear.x * linear.x + linear.y * linear.y +
                               linear.z * linear.z), 2));
    const double yaw = yawFromQuaternion(data.odom.pose.pose.orientation);
    const double forward = relative_forward_input_->value();
    const double left = relative_left_input_->value();
    const double target_x = data.odom.pose.pose.position.x +
                            std::cos(yaw) * forward - std::sin(yaw) * left;
    const double target_y = data.odom.pose.pose.position.y +
                            std::sin(yaw) * forward + std::cos(yaw) * left;
    const double target_yaw =
        yaw + relative_yaw_input_->value() * kPi / 180.0;
    values_["relative_goal_preview"]->setText(
        QStringLiteral("X %1 · Y %2 · Yaw %3°")
            .arg(target_x, 0, 'f', 2)
            .arg(target_y, 0, 'f', 2)
            .arg(target_yaw * 180.0 / kPi, 0, 'f', 1));
  }
  else
  {
    values_["overview_xy"]->setText(QStringLiteral("--"));
    values_["overview_yaw"]->setText(QStringLiteral("--"));
    values_["overview_speed"]->setText(QStringLiteral("--"));
    values_["relative_goal_preview"]->setText(QStringLiteral("等待定位"));
  }
  values_["overview_nav"]->setText(data.navigation_received ? nav_state : QStringLiteral("未启动"));

  const bool mode_is_available = data.mode_state_received || data.mode_status_received;
  const bool mode_is_fresh = mode_is_available && mode_age <= kFreshModeSeconds;
  const bool navigation_paused =
      mode_status.valid
          ? mode_status.navigation_paused
          : (mode_is_available && mode_state != QStringLiteral("GPS_ACTIVE"));
  const QString mode_name = mode_is_available ? modeDisplayName(mode_state)
                                               : QStringLiteral("未启动（仅相对导航）");
  values_["overview_mode"]->setText(mode_name);
  values_["overview_navigation_paused"]->setText(
      navigation_paused ? QStringLiteral("休眠 / 目标保留")
                        : QStringLiteral("运行 / 可接收目标"));

  const bool debug_preview_fresh = !data.debug_image.isNull() &&
                                   wallAge(data.debug_image_received_at) <=
                                       kFreshDetectionSeconds;
  const bool raw_preview_fresh = !data.raw_preview.isNull() &&
                                 wallAge(data.raw_preview_received_at) <=
                                     kFreshCameraSeconds;
  QImage selected_preview;
  QString preview_source = QStringLiteral("无新鲜画面");
  if (debug_preview_fresh)
  {
    selected_preview = data.debug_image;
    preview_source = QStringLiteral("YOLO11 标注 /fod/debug/image");
  }
  else if (raw_preview_fresh)
  {
    selected_preview = data.raw_preview;
    preview_source = QStringLiteral("相机原图 /fod_camera/image_raw");
  }
  updateImageLabel(overview_camera_preview_, selected_preview,
                   QStringLiteral("等待相机或 YOLO11 新鲜画面"));
  updateImageLabel(vision_camera_preview_, selected_preview,
                   QStringLiteral("等待 /fod/debug/image 或 /fod_camera/image_raw"));
  values_["vision_image_source"]->setText(preview_source);

  values_["vision_camera_status"]->setText(
      !data.camera_received
          ? QStringLiteral("未收到")
          : (camera_age <= kFreshCameraSeconds
                 ? QStringLiteral("在线 · %1").arg(ageText(camera_age))
                 : QStringLiteral("断流 · %1").arg(ageText(camera_age))));
  values_["vision_camera_format"]->setText(
      data.camera_received
          ? QStringLiteral("%1 × %2 / %3")
                .arg(data.camera_width)
                .arg(data.camera_height)
                .arg(QString::fromStdString(data.camera_encoding))
          : QStringLiteral("--"));
  values_["vision_yolo_status"]->setText(
      !data.detections_received
          ? QStringLiteral("未收到")
          : (detections_age <= kFreshDetectionSeconds
                 ? QStringLiteral("推理正常 · %1").arg(ageText(detections_age))
                 : QStringLiteral("推理超时 · %1").arg(ageText(detections_age))));
  values_["vision_model"]->setText(
      data.detections_received ? QString::fromStdString(data.detections.model_name)
                               : QStringLiteral("--"));
  values_["vision_inference"]->setText(
      data.detections_received ? numberOrDash(data.detections.inference_ms, 1)
                               : QStringLiteral("--"));
  values_["vision_fps"]->setText(
      data.detection_fps.received ? numberOrDash(data.detection_fps.value, 1)
                                  : QStringLiteral("--"));
  values_["vision_detection_count"]->setText(
      data.detections_received ? QString::number(data.detections.detections.size())
                               : QStringLiteral("--"));

  if (vision_detections_)
  {
    if (!data.detections_received)
    {
      vision_detections_->setPlainText(QStringLiteral("尚未收到 /fod/detections"));
    }
    else if (data.detections.detections.empty())
    {
      vision_detections_->setPlainText(
          QStringLiteral("当前帧未检测到目标\n模型：%1\n推理：%2 ms")
              .arg(QString::fromStdString(data.detections.model_name))
              .arg(data.detections.inference_ms, 0, 'f', 1));
    }
    else
    {
      QStringList lines;
      lines << (data.detections.depth_synchronized
                    ? QStringLiteral("ZED 深度：已同步（时差 %1 ms）")
                          .arg(data.detections.depth_sync_delta_sec * 1000.0, 0, 'f', 1)
                    : QStringLiteral("ZED 深度：未同步，视觉伺服禁止选目标"));
      const std::size_t shown = std::min<std::size_t>(12, data.detections.detections.size());
      for (std::size_t index = 0; index < shown; ++index)
      {
        const auto& detection = data.detections.detections[index];
        const QString depth = detection.depth_valid && std::isfinite(detection.depth_m)
                                  ? QStringLiteral("%1 m").arg(detection.depth_m, 0, 'f', 2)
                                  : QStringLiteral("无有效深度");
        lines << QStringLiteral("%1. %2  置信度 %3%  深度 %4  框 [%5, %6, %7 × %8]")
                     .arg(index + 1)
                     .arg(QString::fromStdString(detection.class_name))
                     .arg(detection.confidence * 100.0, 0, 'f', 1)
                     .arg(depth)
                     .arg(detection.bbox.x_offset)
                     .arg(detection.bbox.y_offset)
                     .arg(detection.bbox.width)
                     .arg(detection.bbox.height);
      }
      if (shown < data.detections.detections.size())
        lines << QStringLiteral("……另有 %1 个目标")
                     .arg(data.detections.detections.size() - shown);
      vision_detections_->setPlainText(lines.join(QLatin1Char('\n')));
    }
  }

  const QString visual_state = !mode_status.visual_state.isEmpty()
                                   ? mode_status.visual_state
                                   : QString::fromStdString(data.visual_state);
  values_["vision_mode_state"]->setText(mode_name);
  values_["vision_navigation_paused"]->setText(
      navigation_paused ? QStringLiteral("已休眠（最终目标保留）")
                        : QStringLiteral("活动"));
  values_["vision_servo_state"]->setText(
      visual_state.isEmpty() ? QStringLiteral("--") : visual_state);
  values_["vision_mode_reason"]->setText(
      mode_status.reason.isEmpty() ? QStringLiteral("--") : mode_status.reason);

  const DiagnosticSnapshot& quality = data.image_quality_diagnostic;
  const double quality_age = wallAge(quality.received_at);
  if (!quality.received)
  {
    values_["quality_state"]->setText(QStringLiteral("控制器未启动（可选）"));
  }
  else if (quality_age > 3.0)
  {
    values_["quality_state"]->setText(QStringLiteral("状态超时 · %1").arg(ageText(quality_age)));
  }
  else
  {
    const bool enabled = textIsTrue(diagnosticValue(quality, "enabled", QStringLiteral("false")));
    const bool monitor_only =
        textIsTrue(diagnosticValue(quality, "monitor_only", QStringLiteral("false")));
    values_["quality_state"]->setText(
        !enabled ? QStringLiteral("已停用 / 相机原生自动")
                 : (monitor_only ? QStringLiteral("仅监测") : QStringLiteral("智能控制中")));
  }
  values_["quality_median"]->setText(diagnosticValue(quality, "median"));
  values_["quality_sharpness"]->setText(diagnosticValue(quality, "sharpness"));
  values_["quality_exposure"]->setText(diagnosticValue(quality, "exposure_percent"));
  values_["quality_gain"]->setText(diagnosticValue(quality, "gain_percent"));
  values_["quality_action"]->setText(diagnosticValue(quality, "action"));

  const double odom_age = wallAge(data.odom_received_at);
  values_["fastlio_score"]->setText(QString::number(fastlio.score));
  values_["fastlio_score"]->setStyleSheet(
      QStringLiteral("font-size:30pt;font-weight:800;color:%1;")
          .arg(statusColor(fastlio.health)));
  values_["fastlio_state"]->setText(fastlio.state);
  values_["fastlio_findings"]->setText(
      fastlio.findings.isEmpty() ? QStringLiteral("全部检查通过")
                                 : fastlio.findings.join(QStringLiteral("；")));
  values_["fastlio_odom_stream"]->setText(
      data.odom_received
          ? QStringLiteral("%1 · %2 Hz")
                .arg(ageText(odom_age))
                .arg(data.odom_rate_hz, 0, 'f', 1)
          : QStringLiteral("未收到"));
  values_["fastlio_cloud_stream"]->setText(
      data.cloud_received
          ? QStringLiteral("%1 · %2 Hz")
                .arg(ageText(cloud_age))
                .arg(data.cloud_rate_hz, 0, 'f', 1)
          : QStringLiteral("未收到"));
  values_["fastlio_imu_stream"]->setText(
      data.imu_received
          ? QStringLiteral("%1 · %2 Hz")
                .arg(ageText(imu_age))
                .arg(data.imu_rate_hz, 0, 'f', 0)
          : QStringLiteral("未收到"));
  values_["fastlio_cloud_points"]->setText(
      data.cloud_received ? QString::number(data.cloud_point_count)
                          : QStringLiteral("--"));
  values_["fastlio_tf"]->setText(
      fastlio.tf_ready ? QStringLiteral("连通") : QStringLiteral("不可用"));
  values_["fastlio_position_sigma"]->setText(
      numberOrDash(fastlio.position_sigma_m, 4));
  values_["fastlio_yaw_sigma"]->setText(numberOrDash(fastlio.yaw_sigma_deg, 3));
  values_["fastlio_pose_step"]->setText(
      data.odom_received ? numberOrDash(data.recent_pose_step_m, 4)
                         : QStringLiteral("--"));
  values_["fastlio_yaw_step"]->setText(
      data.odom_received ? numberOrDash(data.recent_yaw_step_deg, 3)
                         : QStringLiteral("--"));
  values_["fastlio_stationary_window"]->setText(
      data.odom_received ? numberOrDash(data.stationary_window_seconds, 1)
                         : QStringLiteral("--"));
  values_["fastlio_stationary_drift"]->setText(
      !data.odom_received
          ? QStringLiteral("--")
          : (data.stationary_window_seconds < kFastLioStationaryWindowSeconds
                 ? QStringLiteral("采集中 · %1")
                       .arg(numberOrDash(data.stationary_drift_m, 4))
                 : numberOrDash(data.stationary_drift_m, 4)));
  if (data.odom_received)
  {
    const auto& position = data.odom.pose.pose.position;
    const auto& linear = data.odom.twist.twist.linear;
    const auto& angular = data.odom.twist.twist.angular;
    const double linear_speed =
        std::sqrt(linear.x * linear.x + linear.y * linear.y + linear.z * linear.z);
    const double angular_speed =
        std::sqrt(angular.x * angular.x + angular.y * angular.y + angular.z * angular.z);
    values_["fastlio_xyz"]->setText(
        QStringLiteral("%1 / %2 / %3 m")
            .arg(position.x, 0, 'f', 3)
            .arg(position.y, 0, 'f', 3)
            .arg(position.z, 0, 'f', 3));
    values_["fastlio_yaw"]->setText(
        numberOrDash(yawFromQuaternion(data.odom.pose.pose.orientation) * 180.0 / kPi, 2));
    values_["fastlio_velocity"]->setText(
        QStringLiteral("%1 m/s · %2 rad/s")
            .arg(linear_speed, 0, 'f', 3)
            .arg(angular_speed, 0, 'f', 3));
    values_["fastlio_frames"]->setText(
        QStringLiteral("%1 → %2")
            .arg(QString::fromStdString(data.odom.header.frame_id),
                 QString::fromStdString(data.odom.child_frame_id)));
  }
  else
  {
    for (const char* key : { "fastlio_xyz", "fastlio_yaw", "fastlio_velocity",
                             "fastlio_frames" })
      values_[key]->setText(QStringLiteral("--"));
  }

  if (fastlio.health != previous_fastlio_health_ && data.odom_received)
  {
    appendEvent(QStringLiteral("FAST-LIO 健康状态变为 %1（%2/100）：%3")
                    .arg(fastlio.state)
                    .arg(fastlio.score)
                    .arg(fastlio.summary),
                fastlio.health != Health::Good);
    previous_fastlio_health_ = fastlio.health;
  }

  values_["test_ros"]->setText(master_online_ ? QStringLiteral("在线") : QStringLiteral("离线"));
  values_["test_fastlio"]->setText(
      QStringLiteral("%1 · %2/100").arg(fastlio.state).arg(fastlio.score));
  values_["test_odom"]->setText(data.odom_received ? ageText(odom_age) : QStringLiteral("未收到"));
  values_["test_goal_subscribers"]->setText(
      ros_interfaces_ready_
          ? QString::number(relative_goal_publisher_.getNumSubscribers())
          : QStringLiteral("--"));
  QString relative_goal_reason;
  const bool can_send_relative_goal =
      relativeGoalReady(data, fastlio, &relative_goal_reason);
  forward_goal_button_->setEnabled(can_send_relative_goal);
  relative_goal_button_->setEnabled(can_send_relative_goal);
  values_["relative_goal_hint"]->setText(
      can_send_relative_goal ? QStringLiteral("就绪，可发送") : relative_goal_reason);

  const bool can_start_fod = master_online_ && ros_interfaces_ready_ && mode_is_fresh &&
                             mode_state == QStringLiteral("GPS_ACTIVE") &&
                             !mode_request_pending_;
  const bool can_stop_fod = master_online_ && ros_interfaces_ready_ &&
                            mode_is_available &&
                            mode_state != QStringLiteral("GPS_ACTIVE") &&
                            !mode_request_pending_;
  for (QPushButton* button : { overview_fod_start_button_, fod_start_button_ })
  {
    if (!button)
      continue;
    button->setEnabled(can_start_fod);
    button->setToolTip(
        can_start_fod
            ? QStringLiteral("最近 FOD 小于 5 m 时接管；5 m 外或 1 秒无识别时保持相对导航")
            : QStringLiteral("需要相对导航活动且模式仲裁服务在线"));
  }
  for (QPushButton* button : { overview_fod_stop_button_, fod_stop_button_ })
    if (button)
      button->setEnabled(can_stop_fod);

  const bool camera_controls_available = master_online_ && ros_interfaces_ready_ &&
                                         !camera_request_pending_;
  camera_query_button_->setEnabled(camera_controls_available);
  camera_apply_button_->setEnabled(camera_controls_available);
  image_quality_enable_button_->setEnabled(master_online_ && ros_interfaces_ready_);
  image_quality_disable_button_->setEnabled(master_online_ && ros_interfaces_ready_);
  if (recorder_.state() == QProcess::NotRunning)
    record_button_->setText(QStringLiteral("开始录包"));
  else if (recorder_stop_requested_)
    record_button_->setText(QStringLiteral("正在保存录包……"));
  else
    record_button_->setText(QStringLiteral("停止录包"));
  record_button_->setEnabled(!recorder_stop_requested_);
  const bool mapping_running = mapper_.state() != QProcess::NotRunning;
  static_map_start_button_->setEnabled(!static_map_mode_ && !mapping_running);
  static_map_stop_button_->setEnabled(!static_map_mode_ && mapping_running &&
                                      !mapper_stop_requested_);
  static_map_start_button_->setToolTip(
      static_map_mode_
          ? QStringLiteral("当前已加载静态地图，不能同时建立新地图")
          : QStringLiteral("同时建立 MID360 三维图和双 LD19 二维图"));

  std::size_t coverage_point_count = 0;
  bool coverage_selecting = false;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    coverage_point_count = coverage_draft_points_.size();
    coverage_selecting = coverage_selecting_;
  }
  const bool coverage_map_ready = master_online_ && ros_interfaces_ready_ &&
                                  static_map_mode_ && data.map_received &&
                                  embedded_map_ready;
  const bool coverage_status_fresh = data.coverage_status_received &&
                                     wallAge(data.coverage_status_received_at) <= 2.0;
  const autolabor_coverage::CoverageStatus& coverage = data.coverage_status;
  const bool coverage_active = data.coverage_status_received && coverage.active;
  if (coverage_active)
    coverage_task_lifecycle_started_ = true;
  const bool coverage_backend_busy = coverage_status_fresh &&
                                     (coverage.state == "PLANNING" ||
                                      coverage.state == "PREPARING");
  const bool coverage_terminal = coverage_status_fresh &&
                                 (coverage.state == "COMPLETED" ||
                                  coverage.state == "COMPLETED_PARTIAL" ||
                                  coverage.state == "CANCELED" ||
                                  coverage.state == "FAILED");
  if (coverage_terminal &&
      (coverage_task_lifecycle_started_ || coverage_cancel_requested_))
  {
    resetCoverageUiState(true);
    coverage_selecting = false;
    coverage_point_count = 0;
    coverage_cancel_requested_ = false;
  }
  values_["coverage_map"]->setText(
      coverage_map_ready
          ? QStringLiteral("已加载 · %1×%2 @ %3 m")
                .arg(data.map_width)
                .arg(data.map_height)
                .arg(data.map_resolution, 0, 'f', 2)
          : (static_map_mode_
                 ? (data.map_received
                        ? QStringLiteral("等待 RViz 二维地图渲染")
                        : QStringLiteral("等待 /map"))
                 : QStringLiteral("未加载，功能禁用")));
  QString coverage_pose = QStringLiteral("等待 /Odometry");
  if (data.odom_received)
  {
    const QString odom_frame = data.odom.header.frame_id.empty()
                                   ? QStringLiteral("未标注坐标系")
                                   : QString::fromStdString(data.odom.header.frame_id);
    coverage_pose = QStringLiteral("%1 · X %2 · Y %3 · Yaw %4°")
                        .arg(odom_frame)
                        .arg(data.odom.pose.pose.position.x, 0, 'f', 2)
                        .arg(data.odom.pose.pose.position.y, 0, 'f', 2)
                        .arg(yawFromQuaternion(data.odom.pose.pose.orientation) *
                                 180.0 / kPi,
                             0, 'f', 1);
    if (coverage_status_fresh && coverage.localized &&
        tf_buffer_.canTransform("map", "base_link", ros::Time(0)))
    {
      try
      {
        const geometry_msgs::TransformStamped map_pose =
            tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
        coverage_pose =
            QStringLiteral("map · X %1 · Y %2 · Yaw %3°")
                .arg(map_pose.transform.translation.x, 0, 'f', 2)
                .arg(map_pose.transform.translation.y, 0, 'f', 2)
                .arg(yawFromQuaternion(map_pose.transform.rotation) * 180.0 / kPi,
                     0, 'f', 1);
      }
      catch (...)
      {
        // Keep the explicitly labelled odometry-frame fallback above.
      }
    }
  }
  values_["coverage_pose"]->setText(coverage_pose);
  values_["coverage_recent_odom"]->setText(
      data.odom_received
          ? QStringLiteral("近 %1 s · %2 点 · %3 m · %4")
                .arg(data.recent_odom_window_seconds, 0, 'f', 1)
                .arg(static_cast<qulonglong>(data.recent_odom_sample_count))
                .arg(data.recent_odom_distance_m, 0, 'f', 2)
                .arg(ageText(wallAge(data.odom_received_at)))
          : QStringLiteral("--"));
  QString coverage_state = QStringLiteral("后端未就绪");
  if (coverage_status_fresh)
  {
    const QString raw = QString::fromStdString(coverage.state);
    const std::map<QString, QString> labels = {
      { QStringLiteral("IDLE"), QStringLiteral("等待框定") },
      { QStringLiteral("PLANNING"), QStringLiteral("正在生成覆盖轨迹") },
      { QStringLiteral("READY"), QStringLiteral("轨迹已就绪") },
      { QStringLiteral("PREPARING"), QStringLiteral("正在复核起点与安全门") },
      { QStringLiteral("GOING_TO_START"), QStringLiteral("前往起点") },
      { QStringLiteral("TRANSITING"), QStringLiteral("转场中") },
      { QStringLiteral("SWEEPING"), QStringLiteral("覆盖路线执行中") },
      { QStringLiteral("WAITING_OBSTACLE"), QStringLiteral("等待动态障碍") },
      { QStringLiteral("PAUSED"), QStringLiteral("已暂停") },
      { QStringLiteral("COMPLETED"), QStringLiteral("已完成") },
      { QStringLiteral("COMPLETED_PARTIAL"), QStringLiteral("部分完成") },
      { QStringLiteral("CANCELED"), QStringLiteral("已取消") },
      { QStringLiteral("FAILED"), QStringLiteral("失败") },
    };
    const auto found = labels.find(raw);
    coverage_state = found == labels.end() ? raw : found->second;
  }
  if (coverage_selecting)
    coverage_state = coverage_plan_pending_ ? QStringLiteral("正在规划")
                                            : QStringLiteral("正在框定区域");
  else if (coverage_plan_pending_)
    coverage_state = QStringLiteral("正在规划");
  values_["coverage_state"]->setText(coverage_state);
  values_["coverage_points"]->setText(QString::number(coverage_point_count));
  values_["coverage_progress"]->setText(
      coverage_status_fresh
          ? QStringLiteral("%1 / %2 · 阻塞 %3")
                .arg(coverage.current_segment)
                .arg(coverage.total_segments)
                .arg(coverage.blocked_segments)
          : QStringLiteral("--"));
  values_["coverage_parameters"]->setText(
      coverage_status_fresh
          ? QStringLiteral("宽 %1 m · 间距 %2 m · R≥%3 m")
                .arg(coverage.operation_width_m, 0, 'f', 2)
                .arg(coverage.lane_spacing_m, 0, 'f', 2)
                .arg(coverage.minimum_turning_radius_m, 0, 'f', 2)
          : QStringLiteral("--"));
  QString coverage_kinematics = QStringLiteral("--");
  if (coverage_status_fresh)
  {
    if (coverage.kinematics_verified)
    {
      coverage_kinematics =
          QStringLiteral("已核对 · L %1 m · 转角需 %2° / 上限 %3°")
              .arg(coverage.chassis_wheelbase_m, 0, 'f', 2)
              .arg(180.0 * coverage.required_steering_angle_rad / kPi, 0, 'f', 1)
              .arg(180.0 * coverage.chassis_max_steering_angle_rad / kPi,
                   0, 'f', 1);
    }
    else
    {
      coverage_kinematics = QStringLiteral("待任务开始时在线核对");
      if (coverage.chassis_max_steering_angle_rad > 0.0F)
        coverage_kinematics = QStringLiteral("核对未通过 · 需查看详情");
    }
  }
  values_["coverage_kinematics"]->setText(coverage_kinematics);
  values_["coverage_kinematics"]->setToolTip(
      coverage_status_fresh
          ? QString::fromStdString(coverage.kinematics_detail)
          : QStringLiteral("等待 /coverage/status"));
  values_["coverage_chassis"]->setText(
      coverage_status_fresh
          ? (coverage.chassis_ready
                 ? QStringLiteral("已就绪 · 无急停或驱动故障")
                 : QStringLiteral("未就绪 · 覆盖启动/恢复禁用"))
          : QStringLiteral("--"));
  values_["coverage_chassis"]->setToolTip(
      coverage_status_fresh
          ? QString::fromStdString(coverage.chassis_detail)
          : QStringLiteral("等待 /coverage/status"));
  values_["coverage_avoidance"]->setText(
      coverage_status_fresh
          ? (coverage.avoidance_ready
                 ? QStringLiteral("已就绪 · MID360 + 前后 LD19")
                 : QStringLiteral("未就绪 · 覆盖启动/恢复禁用"))
          : QStringLiteral("--"));
  values_["coverage_avoidance"]->setToolTip(
      coverage_status_fresh
          ? QString::fromStdString(coverage.avoidance_detail)
          : QStringLiteral("等待 /coverage/status"));
  values_["coverage_area"]->setText(
      coverage_status_fresh
          ? QStringLiteral("%1 / %2 m²")
                .arg(coverage.reachable_area_m2, 0, 'f', 1)
                .arg(coverage.requested_area_m2, 0, 'f', 1)
          : QStringLiteral("--"));
  values_["coverage_unreachable"]->setText(
      coverage_status_fresh
          ? QStringLiteral("%1 m²").arg(coverage.unreachable_area_m2, 0, 'f', 1)
          : QStringLiteral("--"));
  values_["coverage_ratio"]->setText(
      coverage_status_fresh
          ? QStringLiteral("已覆盖估算 %1 / %2 m² · %3 %")
                .arg(coverage.traversed_area_m2, 0, 'f', 1)
                .arg(coverage.reachable_area_m2, 0, 'f', 1)
                .arg(100.0 * coverage.coverage_ratio, 0, 'f', 1)
          : QStringLiteral("--"));
  values_["coverage_actuator"]->setText(
      QStringLiteral("未接入 · 仅执行覆盖导航"));
  values_["coverage_detail"]->setText(
      coverage_status_fresh ? QString::fromStdString(coverage.detail)
                            : QStringLiteral("等待 /coverage/status"));

  const bool coverage_editable = coverage_map_ready && !coverage_active &&
                                 !coverage_backend_busy &&
                                 !coverage_plan_pending_ &&
                                 !coverage_command_pending_ &&
                                 !coverage_cancel_pending_ &&
                                 !coverage_cancel_requested_;
  const bool coverage_has_ready_plan = !coverage_plan_id_.empty() &&
                                       !coverage_active &&
                                       !coverage_backend_busy;
  coverage_width_input_->setEnabled(coverage_editable);
  coverage_overlap_input_->setEnabled(coverage_editable);
  coverage_speed_input_->setEnabled(coverage_editable);
  coverage_reverse_checkbox_->setEnabled(coverage_editable);
  coverage_select_button_->setEnabled(coverage_editable && !coverage_selecting &&
                                      !coverage_has_ready_plan);
  coverage_undo_button_->setEnabled(coverage_selecting && coverage_point_count > 0 &&
                                    !coverage_plan_pending_);
  // Selection cancellation remains available even if /map disappears while
  // the operator is drawing; it never sends a motion command.
  coverage_selection_cancel_button_->setEnabled(coverage_selecting &&
                                                  !coverage_plan_pending_);
  coverage_confirm_button_->setEnabled(coverage_map_ready && coverage_selecting &&
                                        coverage_point_count >= 3 &&
                                        !coverage_plan_pending_);
  const bool coverage_can_start = coverage_editable && !coverage_selecting &&
                                  coverage_status_fresh &&
                                  coverage.state == "READY" && coverage.localized &&
                                  coverage.chassis_ready &&
                                  coverage.avoidance_ready &&
                                  !coverage_plan_id_.empty();
  coverage_start_button_->setEnabled(coverage_can_start);
  coverage_start_button_->setToolTip(
      coverage_can_start
          ? QStringLiteral("后端仍会复核运动门、VCU、FOD 仲裁、定位、里程计、障碍融合和 move_base")
          : (!coverage_map_ready
                 ? QStringLiteral("需要显式以 --map-set 启动实验性全局地图模式")
                 : (coverage_status_fresh && !coverage.chassis_ready
                        ? QString::fromStdString(coverage.chassis_detail)
                        : (coverage_status_fresh && !coverage.avoidance_ready
                               ? QStringLiteral("需要新鲜 /scan 且前后 LD19 已参与融合")
                               : QStringLiteral("需要先生成轨迹并达到 LOCALIZED")))));
  // Once a task is active, pause/cancel are safety controls and stay available
  // even if the map topic subsequently drops out.
  coverage_pause_button_->setEnabled(master_online_ && ros_interfaces_ready_ &&
                                      coverage_active && !coverage_command_pending_);
  const bool coverage_start_preparing =
      (coverage_status_fresh && coverage.state == "PREPARING") ||
      (coverage_command_pending_ && !coverage_active &&
       !coverage_plan_id_.empty());
  const bool coverage_planning =
      coverage_plan_pending_ ||
      (coverage_status_fresh && coverage.state == "PLANNING");
  const bool coverage_cancel_available =
      !coverage_selecting &&
      (coverage_active || coverage_start_preparing || coverage_planning ||
       coverage_has_ready_plan);
  coverage_cancel_button_->setEnabled(master_online_ && ros_interfaces_ready_ &&
                                       coverage_cancel_available &&
                                       !coverage_cancel_pending_ &&
                                       !coverage_cancel_requested_);
  coverage_cancel_button_->setText(
      coverage_active ? QStringLiteral("取消覆盖清扫")
                      : (coverage_planning
                             ? QStringLiteral("取消轨迹生成")
                             : (coverage_start_preparing
                             ? QStringLiteral("取消覆盖启动")
                             : QStringLiteral("取消已生成轨迹"))));
  coverage_pause_button_->setText(
      coverage_status_fresh && coverage.paused ? QStringLiteral("恢复覆盖清扫")
                                               : QStringLiteral("暂停覆盖清扫"));
}

bool MainWindow::relativeGoalReady(const TelemetrySnapshot& data,
                                   const FastLioHealthResult& health,
                                   QString* reason) const
{
  auto reject = [reason](const QString& text) {
    if (reason)
      *reason = text;
    return false;
  };
  if (!master_online_ || !ros_interfaces_ready_)
    return reject(QStringLiteral("ROS 未连接"));
  if (data.coverage_status_received &&
      wallAge(data.coverage_status_received_at) <= 2.0 &&
      data.coverage_status.active)
    return reject(QStringLiteral("覆盖清扫正在独占 move_base，不能发送普通目标"));
  if (health.health != Health::Good)
    return reject(QStringLiteral("FAST-LIO 健康度未达到“健康”"));
  if (!data.odom_received || wallAge(data.odom_received_at) > 0.5)
    return reject(QString::fromStdString(odom_topic_) + QStringLiteral(" 未就绪"));
  if (data.odom.header.frame_id.empty())
    return reject(QStringLiteral("里程计固定坐标系为空"));
  if (!data.navigation_received || wallAge(data.navigation_received_at) > 2.0)
    return reject(QStringLiteral("move_base 未就绪"));
  if (relative_goal_publisher_.getNumSubscribers() == 0)
    return reject(QStringLiteral("/move_base_simple/goal 无订阅者"));

  if (data.mode_state_received || data.mode_status_received)
  {
    const ModeStatusView parsed =
        data.mode_status_received ? parseModeStatus(data.mode_status) : ModeStatusView();
    const QString state = parsed.valid && !parsed.state.isEmpty()
                              ? parsed.state
                              : QString::fromStdString(data.mode_state);
    const double age = data.mode_status_received ? wallAge(data.mode_status_received_at)
                                                  : wallAge(data.mode_state_received_at);
    if (age > kFreshModeSeconds)
      return reject(QStringLiteral("控制模式状态超时"));
    if (state != QStringLiteral("GPS_ACTIVE"))
      return reject(QStringLiteral("相对导航正在休眠，暂不接收新目标"));
    if (parsed.valid && !parsed.move_base_goals_allowed)
      return reject(QStringLiteral("模式仲裁器尚未放行局部目标"));
  }
  if (reason)
    *reason = QStringLiteral("就绪");
  return true;
}

void MainWindow::publishRelativeGoal(double forward_m, double left_m,
                                     double delta_yaw_deg,
                                     const QString& source_description)
{
  const TelemetrySnapshot data = snapshot();
  const FastLioHealthResult health = evaluateFastLioHealth(data);
  QString reason;
  if (!relativeGoalReady(data, health, &reason))
  {
    QMessageBox::warning(this, QStringLiteral("无法发送相对目标"),
                         QStringLiteral("局部目标入口未就绪：") + reason);
    return;
  }
  if (!std::isfinite(forward_m) || !std::isfinite(left_m) ||
      !std::isfinite(delta_yaw_deg))
  {
    QMessageBox::warning(this, QStringLiteral("坐标无效"),
                         QStringLiteral("相对位移或角度不是有限数值。"));
    return;
  }
  if (std::hypot(forward_m, left_m) < 0.01 && std::abs(delta_yaw_deg) < 0.1)
  {
    QMessageBox::information(this, QStringLiteral("目标未发送"),
                             QStringLiteral("相对位移与转角均接近零。"));
    return;
  }

  const double current_yaw = yawFromQuaternion(data.odom.pose.pose.orientation);
  const double target_x = data.odom.pose.pose.position.x +
                          std::cos(current_yaw) * forward_m -
                          std::sin(current_yaw) * left_m;
  const double target_y = data.odom.pose.pose.position.y +
                          std::sin(current_yaw) * forward_m +
                          std::cos(current_yaw) * left_m;
  const double target_yaw = current_yaw + delta_yaw_deg * kPi / 180.0;
  if (!std::isfinite(target_x) || !std::isfinite(target_y) ||
      !std::isfinite(target_yaw))
  {
    QMessageBox::warning(this, QStringLiteral("坐标无效"),
                         QStringLiteral("当前 FAST-LIO 位姿无法换算相对目标。"));
    return;
  }

  geometry_msgs::PoseStamped goal;
  goal.header.stamp = ros::Time::now();
  goal.header.frame_id = data.odom.header.frame_id;
  goal.pose.position.x = target_x;
  goal.pose.position.y = target_y;
  goal.pose.position.z = 0.0;
  goal.pose.orientation.z = std::sin(target_yaw * 0.5);
  goal.pose.orientation.w = std::cos(target_yaw * 0.5);
  relative_goal_publisher_.publish(goal);
  appendEvent(QStringLiteral("已发布%1：Δ前向=%2 m，Δ左向=%3 m，ΔYaw=%4°；"
                             "局部目标=(%5, %6, %7°)，frame=%8")
                  .arg(source_description)
                  .arg(forward_m, 0, 'f', 2)
                  .arg(left_m, 0, 'f', 2)
                  .arg(delta_yaw_deg, 0, 'f', 1)
                  .arg(target_x, 0, 'f', 2)
                  .arg(target_y, 0, 'f', 2)
                  .arg(target_yaw * 180.0 / kPi, 0, 'f', 1)
                  .arg(QString::fromStdString(goal.header.frame_id)));
}

void MainWindow::sendForwardRelativeGoal()
{
  publishRelativeGoal(2.0, 0.0, 0.0, QStringLiteral("车头正前方 2 m 相对目标"));
}

void MainWindow::sendRelativeGoal()
{
  publishRelativeGoal(relative_forward_input_->value(), relative_left_input_->value(),
                      relative_yaw_input_->value(), QStringLiteral("手工相对目标"));
}

void MainWindow::cancelNavigation()
{
  if (!master_online_ || !ros_interfaces_ready_)
  {
    QMessageBox::information(this, QStringLiteral("导航未连接"),
                             QStringLiteral("ROS master 离线，未发布取消消息。"));
    return;
  }
  const TelemetrySnapshot data = snapshot();
  if (data.coverage_status_received &&
      wallAge(data.coverage_status_received_at) <= 2.0 &&
      data.coverage_status.active)
  {
    QMessageBox::information(
        this, QStringLiteral("覆盖任务正在执行"),
        QStringLiteral("普通导航取消会破坏覆盖状态机；请在清扫页使用“取消覆盖清扫”。"));
    return;
  }
  actionlib_msgs::GoalID cancel;
  cancel.stamp = ros::Time(0);
  cancel.id.clear();
  cancel_publisher_.publish(cancel);
  appendEvent(QStringLiteral("已向 /move_base/cancel 发布空 GoalID（取消全部当前目标）。"), true);
}

void MainWindow::selectCoveragePointTool(bool enabled)
{
  if (!rviz_frame_ || rviz_attached_tab_index_ != coverage_tab_index_ ||
      !rviz_frame_->getManager() ||
      !rviz_frame_->getManager()->getToolManager())
    return;
  rviz::ToolManager* manager =
      rviz_frame_->getManager()->getToolManager();
  const QString wanted = enabled ? QStringLiteral("rviz/PublishPoint")
                                 : QStringLiteral("rviz/MoveCamera");
  for (int index = 0; index < manager->numTools(); ++index)
  {
    rviz::Tool* tool = manager->getTool(index);
    if (tool && tool->getClassId() == wanted)
    {
      manager->setCurrentTool(tool);
      return;
    }
  }
}

void MainWindow::publishCoverageDraft()
{
  if (!ros_interfaces_ready_ || !coverage_draft_publisher_)
    return;
  std::vector<geometry_msgs::Point> points;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    points = coverage_draft_points_;
  }
  visualization_msgs::MarkerArray array;
  visualization_msgs::Marker clear;
  clear.action = visualization_msgs::Marker::DELETEALL;
  array.markers.push_back(clear);
  if (!points.empty())
  {
    visualization_msgs::Marker vertices;
    vertices.header.frame_id = "map";
    vertices.header.stamp = ros::Time::now();
    vertices.ns = "coverage_ui_vertices";
    vertices.id = 0;
    vertices.type = visualization_msgs::Marker::SPHERE_LIST;
    vertices.action = visualization_msgs::Marker::ADD;
    vertices.pose.orientation.w = 1.0;
    vertices.scale.x = 0.22;
    vertices.scale.y = 0.22;
    vertices.scale.z = 0.12;
    vertices.color.r = 1.0;
    vertices.color.g = 0.72;
    vertices.color.b = 0.05;
    vertices.color.a = 1.0;
    vertices.points = points;
    array.markers.push_back(vertices);

    visualization_msgs::Marker outline = vertices;
    outline.ns = "coverage_ui_outline";
    outline.id = 1;
    outline.type = visualization_msgs::Marker::LINE_STRIP;
    outline.scale.x = 0.08;
    outline.points = points;
    array.markers.push_back(outline);
  }
  coverage_draft_publisher_.publish(array);
}

void MainWindow::resetCoverageUiState(bool clear_plan_id)
{
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    coverage_selecting_ = false;
    coverage_draft_points_.clear();
  }
  if (clear_plan_id)
  {
    coverage_plan_id_.clear();
    coverage_task_lifecycle_started_ = false;
    ++coverage_plan_generation_;
  }
  publishCoverageDraft();
  selectCoveragePointTool(false);
}

void MainWindow::beginCoverageSelection()
{
  const TelemetrySnapshot data = snapshot();
  if (!master_online_ || !ros_interfaces_ready_ || !static_map_mode_ ||
      !data.map_received || !rviz_initialized_ ||
      rviz_map_ready_message_count_ != data.map_message_count ||
      rviz_attached_tab_index_ != coverage_tab_index_)
  {
    QMessageBox::information(
        this, QStringLiteral("无法框定清扫范围"),
        QStringLiteral("只有通过 --map-set 一键启动并收到 /map 后，覆盖清扫才可用。"));
    return;
  }
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    coverage_selecting_ = true;
    coverage_draft_points_.clear();
  }
  coverage_cancel_requested_ = false;
  publishCoverageDraft();
  selectCoveragePointTool(true);
  appendEvent(QStringLiteral("开始框定覆盖区域：请在清扫页全局地图连续点选顶点。"));
}

void MainWindow::undoCoveragePoint()
{
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    if (!coverage_selecting_ || coverage_draft_points_.empty())
      return;
    coverage_draft_points_.pop_back();
  }
  publishCoverageDraft();
}

void MainWindow::cancelCoverageSelection()
{
  resetCoverageUiState(false);
  appendEvent(QStringLiteral("已取消本次覆盖区域框定。"));
}

void MainWindow::confirmCoverageSelection()
{
  std::vector<geometry_msgs::Point> points;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    if (!coverage_selecting_)
      return;
    points = coverage_draft_points_;
  }
  if (points.size() < 3)
  {
    QMessageBox::information(this, QStringLiteral("顶点不足"),
                             QStringLiteral("至少需要三个顶点才能形成覆盖多边形。"));
    return;
  }
  const double width = coverage_width_input_->value();
  const double overlap = coverage_overlap_input_->value() / 100.0;
  const bool allow_reverse = coverage_reverse_checkbox_->isChecked();
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    coverage_selecting_ = false;
  }
  coverage_plan_pending_ = true;
  const std::uint64_t plan_generation = ++coverage_plan_generation_;
  selectCoveragePointTool(false);
  appendEvent(QStringLiteral("正在按静态地图障碍物裁剪覆盖区域并生成弓字轨迹……"));

  auto* watcher = new QFutureWatcher<CoveragePlanUiResult>(this);
  connect(watcher, &QFutureWatcher<CoveragePlanUiResult>::finished, this,
          [this, watcher, plan_generation]() {
            const CoveragePlanUiResult result = watcher->result();
            if (plan_generation != coverage_plan_generation_)
            {
              watcher->deleteLater();
              return;
            }
            coverage_plan_pending_ = false;
            if (result.success)
            {
              coverage_plan_id_ = result.plan_id;
              {
                std::lock_guard<std::mutex> lock(snapshot_mutex_);
                coverage_draft_points_.clear();
              }
              publishCoverageDraft();
              appendEvent(
                  QStringLiteral("覆盖轨迹已生成：%1 条清扫线，可覆盖 %2 / %3 m²，"
                                 "不可覆盖估算 %4 m²。")
                      .arg(result.swath_count)
                      .arg(result.reachable_area_m2, 0, 'f', 1)
                      .arg(result.requested_area_m2, 0, 'f', 1)
                      .arg(result.unreachable_area_m2, 0, 'f', 1));
            }
            else
            {
              {
                std::lock_guard<std::mutex> lock(snapshot_mutex_);
                coverage_selecting_ = true;
              }
              selectCoveragePointTool(true);
              appendEvent(QStringLiteral("覆盖规划失败：") + result.message, true);
              QMessageBox::warning(this, QStringLiteral("覆盖规划失败"), result.message);
            }
            watcher->deleteLater();
          });
  watcher->setFuture(QtConcurrent::run([points, width, overlap, allow_reverse]() {
    CoveragePlanUiResult result;
    ros::NodeHandle node;
    ros::ServiceClient client =
        node.serviceClient<autolabor_coverage::PlanCoverage>("/coverage/plan", false);
    if (!client.waitForExistence(ros::Duration(2.0)))
    {
      result.message = QStringLiteral("J6M 覆盖规划服务未启动；请检查静态地图 bringup");
      return result;
    }
    autolabor_coverage::PlanCoverage call;
    call.request.region.header.frame_id = "map";
    call.request.region.header.stamp = ros::Time::now();
    for (const geometry_msgs::Point& point : points)
    {
      geometry_msgs::Point32 vertex;
      vertex.x = static_cast<float>(point.x);
      vertex.y = static_cast<float>(point.y);
      vertex.z = 0.0F;
      call.request.region.polygon.points.push_back(vertex);
    }
    call.request.operation_width_m = width;
    call.request.overlap_ratio = overlap;
    call.request.allow_reverse_transit = allow_reverse;
    if (!client.call(call))
    {
      result.message = QStringLiteral("覆盖规划服务调用失败");
      return result;
    }
    result.success = call.response.success;
    result.message = QString::fromStdString(call.response.message);
    result.plan_id = call.response.plan_id;
    result.requested_area_m2 = call.response.requested_area_m2;
    result.reachable_area_m2 = call.response.reachable_area_m2;
    result.unreachable_area_m2 = call.response.unreachable_area_m2;
    result.swath_count = call.response.swath_count;
    return result;
  }));
}

void MainWindow::startCoverage()
{
  if (coverage_plan_id_.empty() || coverage_command_pending_)
    return;
  const double max_speed_mps = coverage_speed_input_->value();
  const QMessageBox::StandardButton answer = QMessageBox::question(
      this, QStringLiteral("确认开始覆盖清扫"),
      QStringLiteral("无人车将先导航到规划器选择的起点，再逐段执行覆盖轨迹。V1 不会启动"
                     "刷盘、风机或喷淋。请确认现场人员远离车辆、实体急停可用且当前定位"
                     "与车辆真实位置一致。\n\n本次最高前进速度：%1 m/s（覆盖任务上限 1.60 m/s）。"
                     "开始前将在线核对 VCU/TEB 运动学参数。"
                     "\n是否开始？")
          .arg(max_speed_mps, 0, 'f', 2),
      QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
  if (answer != QMessageBox::Yes)
    return;
  coverage_cancel_requested_ = false;
  coverage_task_lifecycle_started_ = true;
  coverage_command_pending_ = true;
  coverage_start_button_->setEnabled(false);
  const std::string plan_id = coverage_plan_id_;
  auto* watcher = new QFutureWatcher<QString>(this);
  connect(watcher, &QFutureWatcher<QString>::finished, this,
          [this, watcher]() {
            const QString result = watcher->result();
            const bool success = result.startsWith(QStringLiteral("OK|"));
            const QString message = result.section('|', 1, -1);
            const bool canceled_start =
                coverage_cancel_requested_ ||
                message.contains(QStringLiteral("canceled"), Qt::CaseInsensitive);
            coverage_command_pending_ = false;
            if (!success && !canceled_start)
              coverage_task_lifecycle_started_ = false;
            appendEvent(QStringLiteral("开始覆盖清扫：") + message,
                        !success && !canceled_start);
            if (!success && !canceled_start)
              QMessageBox::warning(this, QStringLiteral("覆盖任务未启动"), message);
            watcher->deleteLater();
          });
  watcher->setFuture(QtConcurrent::run([plan_id, max_speed_mps]() {
    ros::NodeHandle node;
    ros::ServiceClient client =
        node.serviceClient<autolabor_coverage::StartCoverage>("/coverage/start", false);
    if (!client.waitForExistence(ros::Duration(1.0)))
      return QStringLiteral("ERR|覆盖执行服务未启动");
    autolabor_coverage::StartCoverage call;
    call.request.plan_id = plan_id;
    call.request.max_speed_mps = max_speed_mps;
    if (!client.call(call))
      return QStringLiteral("ERR|覆盖执行服务调用失败");
    return QString(call.response.accepted ? QStringLiteral("OK|")
                                          : QStringLiteral("ERR|")) +
           QString::fromStdString(call.response.message);
  }));
}

void MainWindow::callCoveragePause(bool paused)
{
  if (coverage_command_pending_)
    return;
  coverage_command_pending_ = true;
  auto* watcher = new QFutureWatcher<QString>(this);
  connect(watcher, &QFutureWatcher<QString>::finished, this,
          [this, watcher, paused]() {
            const QString result = watcher->result();
            const bool success = result.startsWith(QStringLiteral("OK|"));
            const QString message = result.section('|', 1, -1);
            coverage_command_pending_ = false;
            appendEvent(paused ? QStringLiteral("暂停覆盖清扫：") + message
                               : QStringLiteral("恢复覆盖清扫：") + message,
                        !success);
            if (!success)
              QMessageBox::warning(this, QStringLiteral("覆盖任务控制失败"), message);
            watcher->deleteLater();
          });
  watcher->setFuture(QtConcurrent::run([paused]() {
    ros::NodeHandle node;
    ros::ServiceClient client =
        node.serviceClient<std_srvs::SetBool>("/coverage/set_paused", false);
    if (!client.waitForExistence(ros::Duration(1.0)))
      return QStringLiteral("ERR|覆盖暂停服务未启动");
    std_srvs::SetBool call;
    call.request.data = paused;
    if (!client.call(call))
      return QStringLiteral("ERR|覆盖暂停服务调用失败");
    return QString(call.response.success ? QStringLiteral("OK|")
                                         : QStringLiteral("ERR|")) +
           QString::fromStdString(call.response.message);
  }));
}

void MainWindow::toggleCoveragePause()
{
  const TelemetrySnapshot data = snapshot();
  if (!data.coverage_status_received || !data.coverage_status.active)
    return;
  callCoveragePause(!data.coverage_status.paused);
}

void MainWindow::cancelCoverageTask()
{
  const TelemetrySnapshot data = snapshot();
  const bool active_at_request =
      data.coverage_status_received && data.coverage_status.active;
  const bool local_planning_at_request = coverage_plan_pending_;
  const bool planning_at_request =
      local_planning_at_request ||
      (data.coverage_status_received &&
       data.coverage_status.state == "PLANNING");
  const bool preparing_at_request =
      !active_at_request && !planning_at_request &&
      ((data.coverage_status_received &&
        data.coverage_status.state == "PREPARING") ||
       coverage_command_pending_);
  const bool ready_at_request = !coverage_plan_id_.empty();
  if (coverage_cancel_pending_ ||
      (!active_at_request && !planning_at_request &&
       !preparing_at_request && !ready_at_request))
    return;
  const QString prompt =
      active_at_request
          ? QStringLiteral("将取消当前 move_base 目标并终止本次覆盖任务。是否继续？")
          : (planning_at_request
                 ? QStringLiteral("将取消正在生成的覆盖轨迹并清空当前框定区域。是否继续？")
                 : (preparing_at_request
                 ? QStringLiteral("将取消正在进行的覆盖启动检查，且不会提交导航目标。是否继续？")
                 : QStringLiteral("将丢弃已确认的覆盖区域和生成轨迹。是否继续？")));
  const QMessageBox::StandardButton answer = QMessageBox::question(
      this, QStringLiteral("确认取消覆盖清扫"),
      prompt,
      QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
  if (answer != QMessageBox::Yes)
    return;
  coverage_cancel_pending_ = true;
  coverage_cancel_requested_ = true;
  if (local_planning_at_request)
    ++coverage_plan_generation_;
  auto* watcher = new QFutureWatcher<QString>(this);
  connect(watcher, &QFutureWatcher<QString>::finished, this,
          [this, watcher, local_planning_at_request]() {
            const QString result = watcher->result();
            const bool success = result.startsWith(QStringLiteral("OK|"));
            const QString message = result.section('|', 1, -1);
            const bool asynchronous_cancel =
                success &&
                message.contains(QStringLiteral("requested"), Qt::CaseInsensitive);
            coverage_cancel_pending_ = false;
            appendEvent(QStringLiteral("取消覆盖清扫：") + message, !success);
            if (!success)
            {
              if (local_planning_at_request)
                coverage_plan_pending_ = false;
              coverage_cancel_requested_ = false;
              QMessageBox::warning(this, QStringLiteral("覆盖任务取消失败"), message);
            }
            else if (!asynchronous_cancel)
            {
              // READY/PLANNING/PREPARING cancellation is synchronous and
              // submits no motion.  Invalidate the client request generation
              // so its late future cannot revive a discarded plan or draft.
              if (local_planning_at_request)
                coverage_plan_pending_ = false;
              coverage_command_pending_ = false;
              coverage_cancel_requested_ = false;
              resetCoverageUiState(true);
            }
            watcher->deleteLater();
          });
  watcher->setFuture(QtConcurrent::run([]() {
    ros::NodeHandle node;
    ros::ServiceClient client =
        node.serviceClient<std_srvs::Trigger>("/coverage/cancel", false);
    if (!client.waitForExistence(ros::Duration(1.0)))
      return QStringLiteral("ERR|覆盖取消服务未启动");
    std_srvs::Trigger call;
    if (!client.call(call))
      return QStringLiteral("ERR|覆盖取消服务调用失败");
    return QString(call.response.success ? QStringLiteral("OK|")
                                         : QStringLiteral("ERR|")) +
           QString::fromStdString(call.response.message);
  }));
}

void MainWindow::callSetBoolService(const std::string& service_name, bool enabled,
                                    QPushButton* button, const QString& action_name)
{
  if (!master_online_ || !ros_interfaces_ready_)
  {
    QMessageBox::information(this, QStringLiteral("服务不可用"),
                             QStringLiteral("ROS master 当前离线。"));
    return;
  }
  if (button)
    button->setEnabled(false);
  auto* watcher = new QFutureWatcher<QString>(this);
  connect(watcher, &QFutureWatcher<QString>::finished, this,
          [this, watcher, button, action_name]() {
            const QString result = watcher->result();
            const bool success = result.startsWith(QStringLiteral("OK|"));
            const QString message = result.section('|', 1, -1);
            appendEvent(action_name + QStringLiteral("：") + message, !success);
            if (!success)
              QMessageBox::warning(this, action_name, message);
            if (button)
              button->setEnabled(true);
            watcher->deleteLater();
          });
  watcher->setFuture(QtConcurrent::run([service_name, enabled]() {
    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<std_srvs::SetBool>(service_name, false);
    if (!client.waitForExistence(ros::Duration(0.75)))
      return QStringLiteral("ERR|服务未启动或未注册");
    std_srvs::SetBool call;
    call.request.data = enabled;
    if (!client.call(call))
      return QStringLiteral("ERR|服务调用失败");
    return QString(call.response.success ? QStringLiteral("OK|") : QStringLiteral("ERR|")) +
           QString::fromStdString(call.response.message);
  }));
}

void MainWindow::startFodMode()
{
  const TelemetrySnapshot data = snapshot();
  const ModeStatusView mode_status =
      data.mode_status_received ? parseModeStatus(data.mode_status) : ModeStatusView();
  const QString state = mode_status.valid && !mode_status.state.isEmpty()
                            ? mode_status.state
                            : QString::fromStdString(data.mode_state);
  const double mode_age = data.mode_status_received
                              ? wallAge(data.mode_status_received_at)
                              : wallAge(data.mode_state_received_at);
  if (!master_online_ || !ros_interfaces_ready_ || mode_request_pending_ ||
      state != QStringLiteral("GPS_ACTIVE") || mode_age > kFreshModeSeconds)
  {
    QMessageBox::warning(
        this, QStringLiteral("视觉模式尚未就绪"),
        QStringLiteral("需要相对导航活动且模式仲裁服务在线。"));
    return;
  }

  const QMessageBox::StandardButton answer = QMessageBox::question(
      this, QStringLiteral("确认进入视觉行驶模式"),
      QStringLiteral("系统只会在最近有效深度 FOD 小于 5 m 时暂停相对导航并切入视觉控制；"
                     "FOD 在 5 m 外或连续 1 秒没有识别信息时继续原局部路线。目标沿车体中线"
                     "从图像下方消失后，再直行 0.5 m 通过滚轴并自动恢复相对导航。"
                     "\n\n是否开始判定？"),
      QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
  if (answer != QMessageBox::Yes)
    return;
  requestFodMode(true);
}

void MainWindow::stopFodMode()
{
  if (mode_request_pending_)
    return;
  const QMessageBox::StandardButton answer = QMessageBox::question(
      this, QStringLiteral("确认恢复相对导航"),
      QStringLiteral("安全仲裁器会先停用视觉控制并确认车辆停车，然后恢复保留的局部路线。"
                     "若仍有未完成目标，无人车可能随后继续导航。是否继续？"),
      QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
  if (answer != QMessageBox::Yes)
    return;
  requestFodMode(false);
}

void MainWindow::requestFodMode(bool enabled)
{
  if (!master_online_ || !ros_interfaces_ready_ || mode_request_pending_)
    return;
  mode_request_pending_ = true;
  for (QPushButton* button : { overview_fod_start_button_, fod_start_button_,
                               overview_fod_stop_button_, fod_stop_button_ })
    if (button)
      button->setEnabled(false);
  appendEvent(enabled ? QStringLiteral("正在检查最近 FOD 距离；满足小于 5 m 才切入视觉模式……")
                      : QStringLiteral("正在请求退出视觉模式并安全恢复相对导航……"),
              enabled);

  auto* watcher = new QFutureWatcher<QString>(this);
  connect(watcher, &QFutureWatcher<QString>::finished, this,
          [this, watcher, enabled]() {
            const QString result = watcher->result();
            const bool success = result.startsWith(QStringLiteral("OK|"));
            const QString message = result.section('|', 1, -1);
            mode_request_pending_ = false;
            appendEvent((enabled ? QStringLiteral("进入视觉模式")
                                 : QStringLiteral("恢复相对导航")) +
                            QStringLiteral("：") + message,
                        !success);
            if (!success)
              QMessageBox::warning(this, QStringLiteral("模式切换未完成"), message);
            watcher->deleteLater();
          });
  watcher->setFuture(QtConcurrent::run([enabled]() {
    ros::NodeHandle node;
    ros::ServiceClient client =
        node.serviceClient<std_srvs::SetBool>(kFodModeService, false);
    if (!client.waitForExistence(ros::Duration(1.0)))
      return QStringLiteral("ERR|安全模式仲裁服务未启动；请使用完整室内导航 bringup");
    std_srvs::SetBool call;
    call.request.data = enabled;
    if (!client.call(call))
      return QStringLiteral("ERR|安全模式仲裁服务调用失败");
    return QString(call.response.success ? QStringLiteral("OK|") : QStringLiteral("ERR|")) +
           QString::fromStdString(call.response.message);
  }));
}

void MainWindow::queryCameraControls()
{
  requestCameraControls(false);
}

void MainWindow::applyCameraControls()
{
  const TelemetrySnapshot data = snapshot();
  const QString quality_enabled =
      diagnosticValue(data.image_quality_diagnostic, "enabled", QStringLiteral("false"));
  if (data.image_quality_diagnostic.received && textIsTrue(quality_enabled) &&
      wallAge(data.image_quality_diagnostic.received_at) <= 3.0)
  {
    const QMessageBox::StandardButton answer = QMessageBox::question(
        this, QStringLiteral("智能图像控制正在运行"),
        QStringLiteral("路面 ROI 图像质量控制器会继续调整曝光和增益，可能覆盖这次手工设置。"
                       "若要固定手工值，建议先点击“停用并恢复相机自动”，再应用参数。"
                       "\n\n仍要继续应用吗？"),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (answer != QMessageBox::Yes)
      return;
  }
  requestCameraControls(true);
}

void MainWindow::requestCameraControls(bool apply_changes)
{
  if (!master_online_ || !ros_interfaces_ready_ || camera_request_pending_)
  {
    if (!master_online_ || !ros_interfaces_ready_)
      QMessageBox::information(this, QStringLiteral("相机服务不可用"),
                               QStringLiteral("ROS master 当前离线。"));
    return;
  }
  const bool auto_exposure_gain = exposure_auto_checkbox_->isChecked();
  const double exposure = exposure_input_->value();
  const double gain = gain_input_->value();
  camera_request_pending_ = true;
  camera_query_button_->setEnabled(false);
  camera_apply_button_->setEnabled(false);

  auto* watcher = new QFutureWatcher<CameraControlsResult>(this);
  connect(watcher, &QFutureWatcher<CameraControlsResult>::finished, this,
          [this, watcher, apply_changes]() {
            const CameraControlsResult result = watcher->result();
            camera_request_pending_ = false;
            applyCameraControlsResult(result);
            appendEvent((apply_changes ? QStringLiteral("应用相机参数")
                                       : QStringLiteral("读取相机参数")) +
                            QStringLiteral("：") + result.message,
                        !result.success);
            watcher->deleteLater();
          });
  watcher->setFuture(QtConcurrent::run(
      [apply_changes, auto_exposure_gain, exposure, gain]() {
        CameraControlsResult result;
        ros::NodeHandle node;
        ros::ServiceClient client =
            node.serviceClient<dynamic_reconfigure::Reconfigure>(
                kZedReconfigureService, false);
        if (!client.waitForExistence(ros::Duration(0.75)))
        {
          result.message = QStringLiteral("ZED 参数服务未启动：%1")
                               .arg(QString::fromLatin1(kZedReconfigureService));
          return result;
        }
        const auto update = ros::topic::waitForMessage<dynamic_reconfigure::Config>(
            kZedParameterUpdatesTopic, node, ros::Duration(0.75));
        if (!update)
        {
          result.message = QStringLiteral("未收到 ZED 当前动态参数");
          return result;
        }
        dynamic_reconfigure::Config working = *update;
        const auto apply_config = [&client](dynamic_reconfigure::Config* config) {
          dynamic_reconfigure::Reconfigure call;
          call.request.config = *config;
          if (!client.call(call))
            return false;
          *config = call.response.config;
          return true;
        };
        if (apply_changes)
        {
          if (!setConfigBool(&working, "auto_exposure_gain", auto_exposure_gain) ||
              !apply_config(&working))
          {
            result.message = QStringLiteral("ZED 自动曝光/增益设置失败");
            return result;
          }

          // ZED ROS1 handles one dynamic-reconfigure level per callback.
          // Manual mode must therefore be applied in this exact order.
          if (!auto_exposure_gain)
          {
            if (!setConfigInt(&working, "exposure",
                              static_cast<int>(std::lround(exposure))) ||
                !apply_config(&working))
            {
              result.message = QStringLiteral("ZED 曝光设置失败；请恢复自动模式");
              return result;
            }

            if (!setConfigInt(&working, "gain",
                              static_cast<int>(std::lround(gain))) ||
                !apply_config(&working))
            {
              result.message = QStringLiteral("ZED 增益设置失败；请恢复自动模式");
              return result;
            }
          }
        }

        bool actual_auto = true;
        int actual_exposure = 100;
        int actual_gain = 100;
        if (!configBool(working, "auto_exposure_gain", &actual_auto) ||
            !configInt(working, "exposure", &actual_exposure) ||
            !configInt(working, "gain", &actual_gain))
        {
          result.message = QStringLiteral("ZED 动态参数响应不完整");
          return result;
        }
        result.success = true;
        result.message = apply_changes ? QStringLiteral("已应用并回读 ZED 参数")
                                       : QStringLiteral("读取成功");
        result.auto_exposure_gain = actual_auto;
        result.exposure_percent = actual_exposure;
        result.gain_percent = actual_gain;
        return result;
      }));
}

void MainWindow::applyCameraControlsResult(const CameraControlsResult& result)
{
  if (!result.success)
  {
    QMessageBox::warning(this, QStringLiteral("相机参数操作失败"), result.message);
    return;
  }
  exposure_input_->setRange(0.0, 100.0);
  gain_input_->setRange(0.0, 100.0);
  exposure_input_->setValue(result.exposure_percent);
  gain_input_->setValue(result.gain_percent);
  exposure_auto_checkbox_->setChecked(result.auto_exposure_gain);
}

void MainWindow::enableImageQualityControl()
{
  callSetBoolService(kImageQualityControlService, true, image_quality_enable_button_,
                     QStringLiteral("启用路面 ROI 图像质量控制"));
}

void MainWindow::disableImageQualityControl()
{
  callSetBoolService(kImageQualityControlService, false, image_quality_disable_button_,
                     QStringLiteral("停用图像质量控制并恢复相机自动模式"));
}

void MainWindow::updateImageLabel(QLabel* label, const QImage& image,
                                  const QString& placeholder)
{
  if (!label)
    return;
  if (image.isNull())
  {
    label->setPixmap(QPixmap());
    label->setText(placeholder);
    return;
  }
  const QSize target(std::max(1, label->contentsRect().width() - 4),
                     std::max(1, label->contentsRect().height() - 4));
  label->setText(QString());
  label->setPixmap(QPixmap::fromImage(image).scaled(
      target, Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

void MainWindow::toggleRecording()
{
  if (recorder_.state() != QProcess::NotRunning)
  {
    if (recorder_stop_requested_)
      return;
    recorder_stop_requested_ = true;
    record_button_->setEnabled(false);
    appendEvent(QStringLiteral("正在停止并索引 rosbag，请勿关闭程序……"));
    recorder_.terminate();
    QTimer::singleShot(60000, this, [this]() {
      if (recorder_.state() != QProcess::NotRunning)
      {
        appendEvent(QStringLiteral("录包未在 60 秒内完成保存，执行强制停止。"), true);
        recorder_.kill();
      }
    });
    return;
  }

  recorder_error_ = false;
  recorder_stop_requested_ = false;
  const QString package_path =
      QString::fromStdString(ros::package::getPath("autolabor_operator_gui"));
  const QString workspace_path =
      QDir::cleanPath(QDir(package_path).absoluteFilePath(QStringLiteral("../../..")));
  recorder_.setWorkingDirectory(workspace_path);
  recorder_.setProgram(
      QDir(workspace_path).filePath(QStringLiteral("scripts/record_rosbag.sh")));
  recorder_.setArguments(QStringList() << QStringLiteral("mode1"));
  recorder_.setProcessChannelMode(QProcess::SeparateChannels);
  recorder_.start();
  appendEvent(QStringLiteral("已开始录制相关 ROS 话题；本操作不会触发建图。"));
}

void MainWindow::handleRecorderFinished(int exit_code, QProcess::ExitStatus exit_status)
{
  const bool normal = exit_status == QProcess::NormalExit && exit_code == 0;
  recorder_error_ = !normal;
  appendEvent(recorder_stop_requested_ && normal
                  ? QStringLiteral("录包已结束并完成索引。")
                  : QStringLiteral("录包进程已结束：exit=%1").arg(exit_code),
              !normal);
  recorder_stop_requested_ = false;
}

void MainWindow::handleRecorderError(QProcess::ProcessError error)
{
  if (recorder_stop_requested_ && error == QProcess::Crashed)
    return;
  recorder_error_ = true;
  appendEvent(QStringLiteral("录包进程错误（%1）：%2").arg(static_cast<int>(error)).arg(recorder_.errorString()),
              true);
}

void MainWindow::startStaticMapping()
{
  if (static_map_mode_)
  {
    appendEvent(QStringLiteral("当前已加载静态地图，建图功能已禁用。"), true);
    return;
  }
  if (mapper_.state() != QProcess::NotRunning)
    return;
  mapper_error_ = false;
  mapper_stop_requested_ = false;
  const QString package_path =
      QString::fromStdString(ros::package::getPath("autolabor_operator_gui"));
  const QString workspace_path =
      QDir::cleanPath(QDir(package_path).absoluteFilePath(QStringLiteral("../../..")));
  mapper_.setWorkingDirectory(workspace_path);
  mapper_.setProgram(
      QDir(workspace_path).filePath(QStringLiteral("scripts/global_mapping_session.sh")));
  mapper_.setArguments(QStringList());
  mapper_.setProcessChannelMode(QProcess::SeparateChannels);
  mapper_.start();
  appendEvent(QStringLiteral("已启动三地图静态建图；占据二维图只使用前后 LD19 数据。"));
}

void MainWindow::stopStaticMapping()
{
  if (mapper_.state() == QProcess::NotRunning || mapper_stop_requested_)
    return;
  mapper_stop_requested_ = true;
  static_map_stop_button_->setEnabled(false);
  appendEvent(QStringLiteral("正在停止建图、保存三维/二维地图并生成融合图，请勿关闭程序……"));
  mapper_.terminate();
  QTimer::singleShot(120000, this, [this]() {
    if (mapper_.state() != QProcess::NotRunning)
    {
      appendEvent(QStringLiteral("静态地图未在 120 秒内完成保存，执行强制停止。"), true);
      mapper_.kill();
    }
  });
}

void MainWindow::handleMapperFinished(int exit_code, QProcess::ExitStatus exit_status)
{
  const bool normal = exit_status == QProcess::NormalExit && exit_code == 0;
  mapper_error_ = !normal;
  appendEvent(mapper_stop_requested_ && normal
                  ? QStringLiteral("三类静态地图均已保存，latest 已更新。")
                  : QStringLiteral("静态建图进程已结束：exit=%1").arg(exit_code),
              !normal);
  mapper_stop_requested_ = false;
}

void MainWindow::handleMapperError(QProcess::ProcessError error)
{
  if (mapper_stop_requested_ && error == QProcess::Crashed)
    return;
  mapper_error_ = true;
  appendEvent(QStringLiteral("静态建图进程错误（%1）：%2")
                  .arg(static_cast<int>(error))
                  .arg(mapper_.errorString()),
              true);
}

}  // namespace autolabor_operator_gui
