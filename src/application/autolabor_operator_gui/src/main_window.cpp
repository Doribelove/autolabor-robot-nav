#include <autolabor_operator_gui/main_window.h>

#include <rviz/visualization_frame.h>

#include <ros/master.h>
#include <ros/package.h>
#include <std_srvs/Trigger.h>

#include <QApplication>
#include <QCloseEvent>
#include <QDateTime>
#include <QDockWidget>
#include <QFrame>
#include <QFuture>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMenuBar>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProcessEnvironment>
#include <QPushButton>
#include <QScrollArea>
#include <QSizePolicy>
#include <QSplitter>
#include <QStatusBar>
#include <QTabWidget>
#include <QTextBrowser>
#include <QtConcurrent/QtConcurrentRun>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace autolabor_operator_gui
{
namespace
{
constexpr double kEarthRadiusMetres = 6378137.0;
constexpr double kPi = 3.14159265358979323846;

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
    if (!recorder_.waitForFinished(1500))
      recorder_.kill();
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
    if (!recorder_.waitForFinished(1500))
      recorder_.kill();
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
    QMainWindow, QWidget { background: #101721; color: #e7edf5; font-family: "Noto Sans CJK SC", "Microsoft YaHei", sans-serif; font-size: 13pt; }
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
    QPushButton#recordButton { background: #74425d; border-color: #a25b80; }
    QPlainTextEdit, QTextBrowser { background: #0d141d; border: 1px solid #2c394a; border-radius: 6px; color: #c8d2df; selection-background-color: #245d87; font-size: 12pt; }
    QScrollArea { border: 0; }
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
  auto* subtitle = new QLabel(QStringLiteral("GPS NAVIGATION · FIELD TEST CONSOLE"), title_box);
  subtitle->setObjectName(QStringLiteral("appSubtitle"));
  title_layout->addWidget(title);
  title_layout->addWidget(subtitle);
  title_layout->addStretch();
  top_layout->addWidget(title_box);

  auto* cards_layout = new QHBoxLayout();
  cards_layout->setContentsMargins(0, 0, 0, 0);
  cards_layout->setSpacing(10);
  const std::pair<const char*, const char*> cards[] = {
    { "ros", "ROS" },       { "can", "CAN" },       { "gnss", "GNSS" },
    { "heading", "航向" }, { "scan", "雷达" },     { "nav", "导航" },
    { "rabbit", "RabbitMQ" }, { "record", "录包" }
  };
  for (const auto& card : cards)
    cards_layout->addWidget(createStatusCard(QString::fromLatin1(card.first),
                                             QString::fromUtf8(card.second)), 1);
  top_layout->addLayout(cards_layout);

  root->addWidget(top_bar);

  tabs_ = new QTabWidget(central);
  tabs_->setTabPosition(QTabWidget::North);
  tabs_->setDocumentMode(true);
  tabs_->addTab(buildOverviewPage(), QStringLiteral("综合"));
  tabs_->addTab(buildGpsPage(), QStringLiteral("GPS"));
  tabs_->addTab(buildRabbitPage(), QStringLiteral("远程"));
  tabs_->addTab(buildTestPage(), QStringLiteral("测试"));
  tabs_->addTab(buildPlaceholderPage(
                    QStringLiteral("视觉识别"),
                    QStringLiteral("为摄像头与 YOLO 推理链预留的独立页面"),
                    { QStringLiteral("摄像头在线状态与画面预览"),
                      QStringLiteral("YOLO 检测框、类别、置信度与推理帧率"),
                      QStringLiteral("检测事件截图与导航联动（后续接入）") }),
                QStringLiteral("视觉"));
  tabs_->addTab(buildPlaceholderPage(
                    QStringLiteral("清扫装置"),
                    QStringLiteral("为清扫机构状态与控制接口预留的独立页面"),
                    { QStringLiteral("主刷、边刷、风机与喷淋状态"),
                      QStringLiteral("电机电流、温度、转速与故障码"),
                      QStringLiteral("作业模式和累计清扫里程（后续接入）") }),
                QStringLiteral("清扫"));
  tabs_->addTab(buildLogPage(), QStringLiteral("日志"));
  root->addWidget(tabs_, 1);

  setCentralWidget(central);
}

QFrame* MainWindow::createStatusCard(const QString& key, const QString& title)
{
  auto* frame = new QFrame(this);
  frame->setProperty("class", QStringLiteral("statusCard"));
  frame->setMinimumSize(105, 82);
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
  rviz_placeholder_ = new QLabel(
      QStringLiteral("RViz 将在 ROS master 可用后加载\n未启动导航节点时，其他页面仍可正常使用"),
      rviz_host_);
  rviz_placeholder_->setAlignment(Qt::AlignCenter);
  rviz_placeholder_->setStyleSheet(
      QStringLiteral("background:#0b1119;border:1px solid #2b3a4e;border-radius:8px;"
                     "color:#718096;font-size:14pt;"));
  rviz_layout_->addWidget(rviz_placeholder_);
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

  auto* vehicle = new QGroupBox(QStringLiteral("车辆状态"), side_content);
  auto* vehicle_layout = new QVBoxLayout(vehicle);
  vehicle_layout->addWidget(createMetricRow(QStringLiteral("GPS"), &values_["overview_gps"]));
  vehicle_layout->addWidget(createMetricRow(QStringLiteral("航向"), &values_["overview_heading"],
                                            QStringLiteral("°")));
  vehicle_layout->addWidget(createMetricRow(QStringLiteral("局部坐标"), &values_["overview_xy"]));
  vehicle_layout->addWidget(createMetricRow(QStringLiteral("速度"), &values_["overview_speed"],
                                            QStringLiteral("m/s")));
  side_layout->addWidget(vehicle);

  auto* mission = new QGroupBox(QStringLiteral("当前任务"), side_content);
  auto* mission_layout = new QVBoxLayout(mission);
  mission_layout->addWidget(createMetricRow(QStringLiteral("导航状态"), &values_["overview_nav"]));
  mission_layout->addWidget(createMetricRow(QStringLiteral("远程目标"), &values_["overview_target"]));
  mission_layout->addWidget(createMetricRow(QStringLiteral("定位漂移"), &values_["overview_error"],
                                            QStringLiteral("m")));
  auto* open_test = new QPushButton(QStringLiteral("打开测试控制页"), mission);
  connect(open_test, &QPushButton::clicked, this, [this]() { tabs_->setCurrentIndex(3); });
  mission_layout->addWidget(open_test);
  rviz_panels_button_ = new QPushButton(QStringLiteral("显示 RViz 调试面板"), mission);
  rviz_panels_button_->setEnabled(false);
  connect(rviz_panels_button_, &QPushButton::clicked, this, &MainWindow::toggleRvizPanels);
  mission_layout->addWidget(rviz_panels_button_);
  side_layout->addWidget(mission);

  auto* note = new QLabel(
      QStringLiteral("界面为旁路操作台：不会发布 /cmd_vel。原有 bringup、终端测试和 RabbitMQ "
                     "流程均可独立运行。"),
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

QWidget* MainWindow::buildGpsPage()
{
  auto* page = new QWidget(this);
  auto* root = new QVBoxLayout(page);
  root->setContentsMargins(20, 16, 20, 18);
  root->setSpacing(16);

  auto* grid = new QGridLayout();
  grid->setHorizontalSpacing(16);
  grid->setVerticalSpacing(16);
  auto* position = new QGroupBox(QStringLiteral("GNSS 位置"), page);
  auto* position_layout = new QVBoxLayout(position);
  position_layout->addWidget(createMetricRow(QStringLiteral("纬度"), &values_["gps_lat"]));
  position_layout->addWidget(createMetricRow(QStringLiteral("经度"), &values_["gps_lon"]));
  position_layout->addWidget(createMetricRow(QStringLiteral("海拔"), &values_["gps_alt"],
                                             QStringLiteral("m")));
  position_layout->addWidget(createMetricRow(QStringLiteral("定位状态"), &values_["gps_fix_status"]));
  position_layout->addWidget(createMetricRow(QStringLiteral("协方差水平 σ"), &values_["gps_sigma"],
                                             QStringLiteral("m")));
  position_layout->addWidget(createMetricRow(QStringLiteral("消息年龄"), &values_["gps_age"]));
  grid->addWidget(position, 0, 0);

  auto* pose = new QGroupBox(QStringLiteral("局部姿态与运动"), page);
  auto* pose_layout = new QVBoxLayout(pose);
  pose_layout->addWidget(createMetricRow(QStringLiteral("局部 X"), &values_["gps_x"], QStringLiteral("m")));
  pose_layout->addWidget(createMetricRow(QStringLiteral("局部 Y"), &values_["gps_y"], QStringLiteral("m")));
  pose_layout->addWidget(createMetricRow(QStringLiteral("双天线航向"), &values_["gps_heading"],
                                         QStringLiteral("°")));
  pose_layout->addWidget(createMetricRow(QStringLiteral("里程计 yaw"), &values_["gps_yaw"],
                                         QStringLiteral("°")));
  pose_layout->addWidget(createMetricRow(QStringLiteral("线速度"), &values_["gps_linear"],
                                         QStringLiteral("m/s")));
  pose_layout->addWidget(createMetricRow(QStringLiteral("角速度"), &values_["gps_angular"],
                                         QStringLiteral("rad/s")));
  grid->addWidget(pose, 0, 1);

  auto* error = new QGroupBox(QStringLiteral("静态重复精度（最近窗口）"), page);
  auto* error_layout = new QVBoxLayout(error);
  error_layout->addWidget(createMetricRow(QStringLiteral("当前漂移"), &values_["error_current"],
                                          QStringLiteral("m")));
  error_layout->addWidget(createMetricRow(QStringLiteral("RMS"), &values_["error_rms"],
                                          QStringLiteral("m")));
  error_layout->addWidget(createMetricRow(QStringLiteral("最大漂移"), &values_["error_max"],
                                          QStringLiteral("m")));
  error_layout->addWidget(createMetricRow(QStringLiteral("X 标准差"), &values_["error_std_x"],
                                          QStringLiteral("m")));
  error_layout->addWidget(createMetricRow(QStringLiteral("Y 标准差"), &values_["error_std_y"],
                                          QStringLiteral("m")));
  auto* reset = new QPushButton(QStringLiteral("重置静态误差参考点"), error);
  connect(reset, &QPushButton::clicked, this, &MainWindow::resetStaticError);
  error_layout->addWidget(reset);
  grid->addWidget(error, 1, 0);

  auto* explanation = new QGroupBox(QStringLiteral("精度说明"), page);
  auto* explanation_layout = new QVBoxLayout(explanation);
  auto* text = new QLabel(
      QStringLiteral("此处的漂移、RMS 和标准差来自 /gps/static_error/*，表示车辆静止时的重复精度。"
                     "它不是相对测绘真值的绝对定位误差；要评估绝对准确度，需要录入已知参考坐标。"),
      explanation);
  text->setWordWrap(true);
  text->setStyleSheet(QStringLiteral("color:#b7c4d4;font-size:12pt;"));
  explanation_layout->addWidget(text);
  explanation_layout->addWidget(createMetricRow(QStringLiteral("统计摘要"), &values_["error_summary"]));
  explanation_layout->addStretch();
  grid->addWidget(explanation, 1, 1);
  grid->setColumnStretch(0, 1);
  grid->setColumnStretch(1, 1);
  grid->setRowStretch(0, 1);
  grid->setRowStretch(1, 1);
  root->addLayout(grid, 1);

  auto* scroll = new QScrollArea(this);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setWidgetResizable(true);
  scroll->setWidget(page);
  return scroll;
}

QWidget* MainWindow::buildRabbitPage()
{
  auto* page = new QWidget(this);
  auto* root = new QVBoxLayout(page);
  root->setContentsMargins(20, 16, 20, 18);
  root->setSpacing(16);
  auto* grid = new QGridLayout();
  grid->setHorizontalSpacing(16);
  grid->setVerticalSpacing(16);

  auto* connection = new QGroupBox(QStringLiteral("RabbitMQ 连接"), page);
  auto* connection_layout = new QVBoxLayout(connection);
  connection_layout->addWidget(createMetricRow(QStringLiteral("连接状态"), &values_["rabbit_state"]));
  connection_layout->addWidget(createMetricRow(QStringLiteral("Broker"), &values_["rabbit_broker"]));
  connection_layout->addWidget(createMetricRow(QStringLiteral("虚拟主机"), &values_["rabbit_vhost"]));
  connection_layout->addWidget(createMetricRow(QStringLiteral("队列"), &values_["rabbit_queue"]));
  connection_layout->addWidget(createMetricRow(QStringLiteral("Routing key"), &values_["rabbit_routing"]));
  connection_layout->addWidget(
      createMetricRow(QStringLiteral("Ready 快照 / Consumer"), &values_["rabbit_counts"]));
  connection_layout->addWidget(createMetricRow(QStringLiteral("最后错误"), &values_["rabbit_error"]));
  grid->addWidget(connection, 0, 0);

  auto* statistics = new QGroupBox(QStringLiteral("消息统计"), page);
  auto* statistics_layout = new QVBoxLayout(statistics);
  statistics_layout->addWidget(createMetricRow(QStringLiteral("累计接收"), &values_["rabbit_received"]));
  statistics_layout->addWidget(createMetricRow(QStringLiteral("已 ACK 消息"), &values_["rabbit_accepted"]));
  statistics_layout->addWidget(createMetricRow(QStringLiteral("拒绝 / 解析失败"), &values_["rabbit_rejected"]));
  statistics_layout->addWidget(createMetricRow(QStringLiteral("最后消息"), &values_["rabbit_last_message"]));
  statistics_layout->addWidget(createMetricRow(QStringLiteral("缓存状态"), &values_["rabbit_cache"]));
  grid->addWidget(statistics, 0, 1);

  auto* target = new QGroupBox(QStringLiteral("最新远程目标"), page);
  auto* target_layout = new QVBoxLayout(target);
  target_layout->addWidget(createMetricRow(QStringLiteral("可用"), &values_["target_available"]));
  target_layout->addWidget(createMetricRow(QStringLiteral("命令 / 设备"), &values_["target_command"]));
  target_layout->addWidget(createMetricRow(QStringLiteral("类型"), &values_["target_type"]));
  target_layout->addWidget(createMetricRow(QStringLiteral("纬度"), &values_["target_lat"]));
  target_layout->addWidget(createMetricRow(QStringLiteral("经度"), &values_["target_lon"]));
  target_layout->addWidget(createMetricRow(QStringLiteral("来源时间"), &values_["target_source_time"]));
  target_layout->addWidget(createMetricRow(QStringLiteral("URL"), &values_["target_url"]));
  auto* buttons = new QHBoxLayout();
  rabbit_publish_button_ = new QPushButton(QStringLiteral("确认发送缓存目标"), target);
  rabbit_clear_button_ = new QPushButton(QStringLiteral("清空缓存"), target);
  rabbit_clear_button_->setObjectName(QStringLiteral("dangerButton"));
  connect(rabbit_publish_button_, &QPushButton::clicked, this, &MainWindow::publishRabbitTarget);
  connect(rabbit_clear_button_, &QPushButton::clicked, this, &MainWindow::clearRabbitTarget);
  buttons->addWidget(rabbit_publish_button_);
  buttons->addWidget(rabbit_clear_button_);
  target_layout->addLayout(buttons);
  grid->addWidget(target, 1, 0, 1, 2);
  grid->setColumnStretch(0, 1);
  grid->setColumnStretch(1, 1);
  grid->setRowStretch(0, 1);
  grid->setRowStretch(1, 1);
  root->addLayout(grid, 1);

  auto* note = new QLabel(
      QStringLiteral("RabbitMQ 节点不在线、没有收到消息或目标为空时，不会阻止本操作台启动。"
                     "按钮通过 std_srvs/Trigger 请求桥接节点，原终端输入流程仍然保留。"),
      page);
  note->setWordWrap(true);
  note->setStyleSheet(QStringLiteral("color:#8fa0b5;padding:8px;"));
  root->addWidget(note);

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
  readiness_layout->addWidget(createMetricRow(QStringLiteral("GPS 原点"), &values_["test_origin"]));
  readiness_layout->addWidget(createMetricRow(QStringLiteral("/gps/odom"), &values_["test_odom"]));
  readiness_layout->addWidget(createMetricRow(QStringLiteral("目标转换订阅者"), &values_["test_goal_subscribers"]));

  auto* controls = new QGroupBox(QStringLiteral("基础现场测试"), page);
  auto* controls_layout = new QGridLayout(controls);
  controls_layout->setHorizontalSpacing(14);
  controls_layout->setVerticalSpacing(14);
  forward_goal_button_ = new QPushButton(QStringLiteral("发送车头正前方 8 m GPS 目标"), controls);
  forward_goal_button_->setMinimumHeight(58);
  auto* cancel = new QPushButton(QStringLiteral("取消当前导航目标"), controls);
  cancel->setObjectName(QStringLiteral("dangerButton"));
  cancel->setMinimumHeight(58);
  record_button_ = new QPushButton(QStringLiteral("开始 mode1 录包"), controls);
  record_button_->setObjectName(QStringLiteral("recordButton"));
  record_button_->setMinimumHeight(58);
  connect(forward_goal_button_, &QPushButton::clicked, this, &MainWindow::sendForwardGoal);
  connect(cancel, &QPushButton::clicked, this, &MainWindow::cancelNavigation);
  connect(record_button_, &QPushButton::clicked, this, &MainWindow::toggleRecording);
  controls_layout->addWidget(forward_goal_button_, 0, 0);
  controls_layout->addWidget(cancel, 0, 1);
  controls_layout->addWidget(record_button_, 0, 2);

  auto* roadmap = new QGroupBox(QStringLiteral("后续测试中心"), page);
  auto* roadmap_layout = new QVBoxLayout(roadmap);
  auto* roadmap_text = new QLabel(
      QStringLiteral("本版先提供安全的 8 m 目标、GoalID 取消和一键录包入口。T01～T08、电子围栏、"
                     "随机目标、通过/失败记录与自动报告将在测试管理节点结构化后接入。"),
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

QWidget* MainWindow::buildLogPage()
{
  auto* page = new QWidget(this);
  auto* root = new QVBoxLayout(page);
  root->setContentsMargins(18, 14, 18, 14);
  auto* explanation = new QTextBrowser(page);
  explanation->setHtml(QStringLiteral(
      "<h2>操作台运行原则</h2>"
      "<p>这是一个可选的旁路界面。导航仍由现有 <code>bringup.sh</code> 流程启动；"
      "RabbitMQ 与测试终端也可继续独立使用。</p>"
      "<ul><li>ROS master 或任何业务节点缺失时，界面仍会打开并显示离线状态。</li>"
      "<li>界面只发布 GPS 目标、取消 GoalID 和误差重置，不发布 <code>/cmd_vel</code>。</li>"
      "<li>视觉识别与清扫装置采用后续独立 ROS 接口接入，不与本基础版本强耦合。</li></ul>"));
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
    if (!result.online)
      return result;
    double latitude = 0.0;
    double longitude = 0.0;
    if (ros::param::get("/gps/origin_lat", latitude) &&
        ros::param::get("/gps/origin_lon", longitude) && std::isfinite(latitude) &&
        std::isfinite(longitude))
    {
      result.has_origin = true;
      result.origin_latitude = latitude;
      result.origin_longitude = longitude;
    }
    return result;
  }));
}

void MainWindow::handleMasterProbeFinished()
{
  const MasterProbeResult result = master_probe_watcher_.result();
  const bool restored = result.online && !previous_probe_online_;
  const bool lost = !result.online && previous_probe_online_;
  master_online_ = result.online;
  has_origin_ = result.has_origin;
  origin_latitude_ = result.origin_latitude;
  origin_longitude_ = result.origin_longitude;

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
      delete rviz_frame_;
      rviz_frame_ = nullptr;
      rviz_initialized_ = false;
      rviz_placeholder_ = new QLabel(
          QStringLiteral("ROS master 已恢复，正在重新加载 RViz……"), rviz_host_);
      rviz_placeholder_->setAlignment(Qt::AlignCenter);
      rviz_placeholder_->setStyleSheet(
          QStringLiteral("background:#0b1119;border:1px solid #2b3a4e;border-radius:8px;"
                         "color:#718096;font-size:14pt;"));
      rviz_layout_->addWidget(rviz_placeholder_);
    }
    appendEvent(QStringLiteral("ROS master 已连接，正在注册界面订阅与发布接口。"));
    setupRosInterfaces();
  }
  else if (lost)
  {
    appendEvent(QStringLiteral("ROS master 连接中断；界面继续运行并保留最后一次数据。"), true);
  }
  previous_probe_online_ = result.online;
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
  node_->param("enable_rviz", enable_rviz_, true);
  const std::string default_rviz =
      ros::package::getPath("autolabor_operator_gui") + "/config/operator_navigation.rviz";
  node_->param<std::string>("rviz_config", rviz_config_path_, default_rviz);

  fix_subscriber_ = node_->subscribe("/gps/fix", 10, &MainWindow::fixCallback, this);
  heading_subscriber_ = node_->subscribe("/gps/heading", 10, &MainWindow::headingCallback, this);
  odom_subscriber_ = node_->subscribe("/gps/odom", 20, &MainWindow::odomCallback, this);
  error_current_subscriber_ =
      node_->subscribe("/gps/static_error/current", 10, &MainWindow::errorCurrentCallback, this);
  error_rms_subscriber_ =
      node_->subscribe("/gps/static_error/rms", 10, &MainWindow::errorRmsCallback, this);
  error_max_subscriber_ =
      node_->subscribe("/gps/static_error/max", 10, &MainWindow::errorMaxCallback, this);
  error_std_x_subscriber_ =
      node_->subscribe("/gps/static_error/std_x", 10, &MainWindow::errorStdXCallback, this);
  error_std_y_subscriber_ =
      node_->subscribe("/gps/static_error/std_y", 10, &MainWindow::errorStdYCallback, this);
  error_summary_subscriber_ =
      node_->subscribe("/gps/static_error/summary", 10, &MainWindow::errorSummaryCallback, this);
  can_subscriber_ = node_->subscribe("/canbus_msg", 100, &MainWindow::canCallback, this);
  scan_subscriber_ = node_->subscribe("/scan", 10, &MainWindow::scanCallback, this);
  navigation_subscriber_ =
      node_->subscribe("/move_base/status", 10, &MainWindow::navigationCallback, this);
  rabbit_status_subscriber_ = node_->subscribe(
      "/rabbitmq_bridge/status", 10, &MainWindow::rabbitStatusCallback, this);
  remote_target_subscriber_ = node_->subscribe(
      "/rabbitmq_bridge/latest_target", 10, &MainWindow::remoteTargetCallback, this);

  goal_publisher_ = node_->advertise<sensor_msgs::NavSatFix>("/gps/goal_fix", 10, false);
  cancel_publisher_ = node_->advertise<actionlib_msgs::GoalID>("/move_base/cancel", 10, false);
  error_reset_publisher_ = node_->advertise<std_msgs::Empty>("/gps/static_error/reset", 1, false);
  ros_interfaces_ready_ = true;
  appendEvent(QStringLiteral("ROS 接口已注册；缺失业务节点不会阻塞界面。"));
  setupEmbeddedRviz();
}

void MainWindow::setupEmbeddedRviz()
{
  if (rviz_initialized_ || !rviz_layout_)
    return;
  if (!enable_rviz_)
  {
    if (rviz_placeholder_)
      rviz_placeholder_->setText(QStringLiteral("已通过 enable_rviz:=false 禁用嵌入式 RViz"));
    return;
  }
  try
  {
    rviz_frame_ = new rviz::VisualizationFrame(rviz_host_);
    rviz_frame_->setWindowFlags(Qt::Widget);
    rviz_frame_->setSplashPath(QString());
    rviz_frame_->setShowChooseNewMaster(false);
    rviz_frame_->initialize(QString::fromStdString(rviz_config_path_));
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
    if (rviz_placeholder_)
    {
      rviz_layout_->removeWidget(rviz_placeholder_);
      rviz_placeholder_->deleteLater();
      rviz_placeholder_ = nullptr;
    }
    rviz_layout_->addWidget(rviz_frame_);
    rviz_initialized_ = true;
    appendEvent(QStringLiteral("嵌入式 RViz 已加载：") + QString::fromStdString(rviz_config_path_));
  }
  catch (const std::exception& error)
  {
    appendEvent(QStringLiteral("RViz 加载失败：") + QString::fromLocal8Bit(error.what()), true);
    if (rviz_frame_)
    {
      rviz_frame_->deleteLater();
      rviz_frame_ = nullptr;
    }
    if (rviz_placeholder_)
      rviz_placeholder_->setText(QStringLiteral("RViz 加载失败；其他功能仍可使用"));
  }
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

void MainWindow::shutdownRosInterfaces()
{
  ros_interfaces_ready_ = false;
  fix_subscriber_.shutdown();
  heading_subscriber_.shutdown();
  odom_subscriber_.shutdown();
  error_current_subscriber_.shutdown();
  error_rms_subscriber_.shutdown();
  error_max_subscriber_.shutdown();
  error_std_x_subscriber_.shutdown();
  error_std_y_subscriber_.shutdown();
  error_summary_subscriber_.shutdown();
  can_subscriber_.shutdown();
  scan_subscriber_.shutdown();
  navigation_subscriber_.shutdown();
  rabbit_status_subscriber_.shutdown();
  remote_target_subscriber_.shutdown();
  goal_publisher_.shutdown();
  cancel_publisher_.shutdown();
  error_reset_publisher_.shutdown();
  node_.reset();
}

void MainWindow::fixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  telemetry_.fix_received = true;
  telemetry_.fix = *msg;
  telemetry_.fix_received_at = ros::WallTime::now();
}

void MainWindow::headingCallback(const std_msgs::Float64::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  telemetry_.heading = { true, msg->data, ros::WallTime::now() };
}

void MainWindow::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  telemetry_.odom_received = true;
  telemetry_.odom = *msg;
  telemetry_.odom_received_at = ros::WallTime::now();
}

void MainWindow::errorCurrentCallback(const std_msgs::Float64::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  telemetry_.error_current = { true, msg->data, ros::WallTime::now() };
}

void MainWindow::errorRmsCallback(const std_msgs::Float64::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  telemetry_.error_rms = { true, msg->data, ros::WallTime::now() };
}

void MainWindow::errorMaxCallback(const std_msgs::Float64::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  telemetry_.error_max = { true, msg->data, ros::WallTime::now() };
}

void MainWindow::errorStdXCallback(const std_msgs::Float64::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  telemetry_.error_std_x = { true, msg->data, ros::WallTime::now() };
}

void MainWindow::errorStdYCallback(const std_msgs::Float64::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  telemetry_.error_std_y = { true, msg->data, ros::WallTime::now() };
}

void MainWindow::errorSummaryCallback(const std_msgs::String::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  telemetry_.error_summary = msg->data;
  telemetry_.error_summary_received_at = ros::WallTime::now();
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

void MainWindow::rabbitStatusCallback(
    const autolabor_operator_msgs::RabbitMqStatus::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  telemetry_.rabbit_status_received = true;
  telemetry_.rabbit_status = *msg;
  telemetry_.rabbit_status_received_at = ros::WallTime::now();
}

void MainWindow::remoteTargetCallback(
    const autolabor_operator_msgs::RemoteTarget::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  telemetry_.remote_target_received = true;
  telemetry_.remote_target = *msg;
  telemetry_.remote_target_received_at = ros::WallTime::now();
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

void MainWindow::refreshUi()
{
  const TelemetrySnapshot data = snapshot();
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

  const double fix_age = wallAge(data.fix_received_at);
  if (!data.fix_received)
    setStatus("gnss", Health::Idle, QStringLiteral("无数据"), QStringLiteral("/gps/fix"));
  else if (fix_age > 2.0)
    setStatus("gnss", Health::Bad, QStringLiteral("超时"), ageText(fix_age));
  else if (data.fix.status.status < sensor_msgs::NavSatStatus::STATUS_FIX)
    setStatus("gnss", Health::Warning, QStringLiteral("无定位"), ageText(fix_age));
  else
    setStatus("gnss", Health::Good, QStringLiteral("定位") , ageText(fix_age));

  const double heading_age = wallAge(data.heading.received_at);
  if (!data.heading.received)
    setStatus("heading", Health::Idle, QStringLiteral("无数据"), QStringLiteral("/gps/heading"));
  else if (heading_age <= 2.0)
    setStatus("heading", Health::Good, numberOrDash(data.heading.value, 1) + QStringLiteral("°"),
              ageText(heading_age));
  else
    setStatus("heading", Health::Bad, QStringLiteral("超时"), ageText(heading_age));

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

  const double rabbit_age = wallAge(data.rabbit_status_received_at);
  if (!data.rabbit_status_received)
    setStatus("rabbit", Health::Idle, QStringLiteral("未启动"), QStringLiteral("可选节点"));
  else if (rabbit_age > 5.0)
    setStatus("rabbit", Health::Warning, QStringLiteral("状态过期"), ageText(rabbit_age));
  else if (data.rabbit_status.connected)
    setStatus("rabbit", Health::Good, QStringLiteral("已连接"), data.rabbit_status.queue_name.c_str());
  else
    setStatus("rabbit", Health::Bad, QStringLiteral("断开"),
              QString::fromStdString(data.rabbit_status.connection_state));

  if (recorder_.state() != QProcess::NotRunning)
    setStatus("record", Health::Good, QStringLiteral("录制中"), QStringLiteral("mode1"));
  else if (recorder_error_)
    setStatus("record", Health::Bad, QStringLiteral("异常"), QStringLiteral("查看日志"));
  else
    setStatus("record", Health::Idle, QStringLiteral("未录制"), QStringLiteral("可随时启动"));

  const QString gps_pair = data.fix_received
                               ? QStringLiteral("%1, %2")
                                     .arg(data.fix.latitude, 0, 'f', 7)
                                     .arg(data.fix.longitude, 0, 'f', 7)
                               : QStringLiteral("--");
  values_["overview_gps"]->setText(gps_pair);
  values_["overview_heading"]->setText(data.heading.received ? numberOrDash(data.heading.value, 1)
                                                              : QStringLiteral("--"));
  if (data.odom_received)
  {
    values_["overview_xy"]->setText(
        QStringLiteral("%1, %2 m")
            .arg(data.odom.pose.pose.position.x, 0, 'f', 2)
            .arg(data.odom.pose.pose.position.y, 0, 'f', 2));
    values_["overview_speed"]->setText(numberOrDash(data.odom.twist.twist.linear.x, 2));
  }
  else
  {
    values_["overview_xy"]->setText(QStringLiteral("--"));
    values_["overview_speed"]->setText(QStringLiteral("--"));
  }
  values_["overview_nav"]->setText(data.navigation_received ? nav_state : QStringLiteral("未启动"));
  values_["overview_target"]->setText(
      data.remote_target_received && data.remote_target.available
          ? QStringLiteral("%1, %2")
                .arg(data.remote_target.latitude, 0, 'f', 7)
                .arg(data.remote_target.longitude, 0, 'f', 7)
          : QStringLiteral("无缓存目标"));
  values_["overview_error"]->setText(data.error_current.received
                                          ? numberOrDash(data.error_current.value, 3)
                                          : QStringLiteral("--"));

  values_["gps_lat"]->setText(data.fix_received ? numberOrDash(data.fix.latitude, 9) : QStringLiteral("--"));
  values_["gps_lon"]->setText(data.fix_received ? numberOrDash(data.fix.longitude, 9) : QStringLiteral("--"));
  values_["gps_alt"]->setText(data.fix_received ? numberOrDash(data.fix.altitude, 2) : QStringLiteral("--"));
  values_["gps_fix_status"]->setText(
      !data.fix_received ? QStringLiteral("未收到")
                         : (data.fix.status.status >= sensor_msgs::NavSatStatus::STATUS_FIX
                                ? QStringLiteral("有效定位")
                                : QStringLiteral("无定位")));
  double horizontal_sigma = std::numeric_limits<double>::quiet_NaN();
  if (data.fix_received && data.fix.position_covariance_type != sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN &&
      data.fix.position_covariance[0] >= 0.0 && data.fix.position_covariance[4] >= 0.0)
    horizontal_sigma = std::sqrt(data.fix.position_covariance[0] + data.fix.position_covariance[4]);
  values_["gps_sigma"]->setText(numberOrDash(horizontal_sigma, 3));
  values_["gps_age"]->setText(ageText(fix_age));
  values_["gps_heading"]->setText(data.heading.received ? numberOrDash(data.heading.value, 2)
                                                           : QStringLiteral("--"));
  if (data.odom_received)
  {
    values_["gps_x"]->setText(numberOrDash(data.odom.pose.pose.position.x, 3));
    values_["gps_y"]->setText(numberOrDash(data.odom.pose.pose.position.y, 3));
    values_["gps_yaw"]->setText(numberOrDash(yawFromQuaternion(data.odom.pose.pose.orientation) * 180.0 / kPi, 2));
    values_["gps_linear"]->setText(numberOrDash(data.odom.twist.twist.linear.x, 3));
    values_["gps_angular"]->setText(numberOrDash(data.odom.twist.twist.angular.z, 3));
  }
  else
  {
    values_["gps_x"]->setText(QStringLiteral("--"));
    values_["gps_y"]->setText(QStringLiteral("--"));
    values_["gps_yaw"]->setText(QStringLiteral("--"));
    values_["gps_linear"]->setText(QStringLiteral("--"));
    values_["gps_angular"]->setText(QStringLiteral("--"));
  }
  values_["error_current"]->setText(data.error_current.received ? numberOrDash(data.error_current.value, 4) : QStringLiteral("--"));
  values_["error_rms"]->setText(data.error_rms.received ? numberOrDash(data.error_rms.value, 4) : QStringLiteral("--"));
  values_["error_max"]->setText(data.error_max.received ? numberOrDash(data.error_max.value, 4) : QStringLiteral("--"));
  values_["error_std_x"]->setText(data.error_std_x.received ? numberOrDash(data.error_std_x.value, 4) : QStringLiteral("--"));
  values_["error_std_y"]->setText(data.error_std_y.received ? numberOrDash(data.error_std_y.value, 4) : QStringLiteral("--"));
  values_["error_summary"]->setText(data.error_summary.empty() ? QStringLiteral("--")
                                                                : QString::fromStdString(data.error_summary));

  if (data.rabbit_status_received)
  {
    const auto& r = data.rabbit_status;
    values_["rabbit_state"]->setText(QString::fromStdString(r.connection_state));
    values_["rabbit_broker"]->setText(
        QStringLiteral("%1:%2").arg(QString::fromStdString(r.broker_host)).arg(r.broker_port));
    values_["rabbit_vhost"]->setText(QString::fromStdString(r.virtual_host));
    values_["rabbit_queue"]->setText(QString::fromStdString(r.queue_name));
    values_["rabbit_routing"]->setText(QString::fromStdString(r.routing_key));
    values_["rabbit_counts"]->setText(QStringLiteral("%1 / %2").arg(r.ready_message_count).arg(r.consumer_count));
    values_["rabbit_error"]->setText(r.last_error.empty() ? QStringLiteral("无") : QString::fromStdString(r.last_error));
    values_["rabbit_received"]->setText(QString::number(r.received_message_count));
    values_["rabbit_accepted"]->setText(QString::number(r.accepted_message_count));
    values_["rabbit_rejected"]->setText(QString::number(r.rejected_message_count));
    values_["rabbit_last_message"]->setText(
        r.last_message_stamp.isZero()
            ? QStringLiteral("未收到")
            : QString::number(std::max(0.0, (ros::Time::now() - r.last_message_stamp).toSec()), 'f', 1) +
                  QStringLiteral(" s 前"));
    values_["rabbit_cache"]->setText(r.has_cached_target ? QStringLiteral("已有缓存") : QStringLiteral("空"));
  }
  else
  {
    const char* rabbit_keys[] = { "rabbit_state", "rabbit_broker", "rabbit_vhost", "rabbit_queue",
                                  "rabbit_routing", "rabbit_counts", "rabbit_error", "rabbit_received",
                                  "rabbit_accepted", "rabbit_rejected", "rabbit_last_message", "rabbit_cache" };
    for (const char* key : rabbit_keys)
      values_[key]->setText(QStringLiteral("--"));
  }

  if (data.remote_target_received && data.remote_target.available)
  {
    const auto& t = data.remote_target;
    values_["target_available"]->setText(QStringLiteral("是"));
    values_["target_command"]->setText(
        QStringLiteral("%1 / %2").arg(QString::fromStdString(t.command), QString::fromStdString(t.device)));
    values_["target_type"]->setText(QString::number(t.target_type));
    values_["target_lat"]->setText(numberOrDash(t.latitude, 9));
    values_["target_lon"]->setText(numberOrDash(t.longitude, 9));
    values_["target_source_time"]->setText(QString::fromStdString(t.source_time));
    values_["target_url"]->setText(QString::fromStdString(t.url));
  }
  else
  {
    values_["target_available"]->setText(data.remote_target_received ? QStringLiteral("否")
                                                                     : QStringLiteral("--"));
    const char* target_keys[] = { "target_command", "target_type", "target_lat", "target_lon",
                                  "target_source_time", "target_url" };
    for (const char* key : target_keys)
      values_[key]->setText(QStringLiteral("--"));
  }

  const double odom_age = wallAge(data.odom_received_at);
  values_["test_ros"]->setText(master_online_ ? QStringLiteral("在线") : QStringLiteral("离线"));
  values_["test_origin"]->setText(
      has_origin_ ? QStringLiteral("%1, %2").arg(origin_latitude_, 0, 'f', 8).arg(origin_longitude_, 0, 'f', 8)
                  : QStringLiteral("未设置"));
  values_["test_odom"]->setText(data.odom_received ? ageText(odom_age) : QStringLiteral("未收到"));
  values_["test_goal_subscribers"]->setText(
      ros_interfaces_ready_ ? QString::number(goal_publisher_.getNumSubscribers()) : QStringLiteral("--"));
  const bool can_send_forward = master_online_ && ros_interfaces_ready_ && has_origin_ &&
                                data.odom_received && odom_age <= 2.0 &&
                                data.navigation_received && nav_age <= 2.0 &&
                                goal_publisher_.getNumSubscribers() > 0;
  forward_goal_button_->setEnabled(can_send_forward);
  rabbit_publish_button_->setEnabled(master_online_ && ros_interfaces_ready_ &&
                                     data.rabbit_status_received && rabbit_age <= 5.0 &&
                                     data.rabbit_status.has_cached_target);
  rabbit_clear_button_->setEnabled(master_online_ && ros_interfaces_ready_ &&
                                   data.rabbit_status_received && rabbit_age <= 5.0);
  record_button_->setText(recorder_.state() == QProcess::NotRunning ? QStringLiteral("开始 mode1 录包")
                                                                    : QStringLiteral("停止录包"));
}

void MainWindow::sendForwardGoal()
{
  const TelemetrySnapshot data = snapshot();
  const double odom_age = wallAge(data.odom_received_at);
  const double navigation_age = wallAge(data.navigation_received_at);
  if (!master_online_ || !ros_interfaces_ready_ || !has_origin_ || !data.odom_received ||
      odom_age > 2.0 || !data.navigation_received || navigation_age > 2.0 ||
      goal_publisher_.getNumSubscribers() == 0)
  {
    QMessageBox::warning(this, QStringLiteral("无法发送目标"),
                         QStringLiteral("需要在线 ROS、有效 GPS 原点、2 秒内的 /gps/odom 和 "
                                        "/move_base/status，并且 /gps/goal_fix 必须有订阅者。"));
    return;
  }
  const double x = data.odom.pose.pose.position.x;
  const double y = data.odom.pose.pose.position.y;
  const double yaw = yawFromQuaternion(data.odom.pose.pose.orientation);
  const double target_x = x + 8.0 * std::cos(yaw);
  const double target_y = y + 8.0 * std::sin(yaw);
  const double origin_latitude_rad = origin_latitude_ * kPi / 180.0;
  const double longitude_scale = kEarthRadiusMetres * std::cos(origin_latitude_rad);
  if (!std::isfinite(target_x) || !std::isfinite(target_y) || std::abs(longitude_scale) < 1e-6)
  {
    QMessageBox::warning(this, QStringLiteral("无法发送目标"), QStringLiteral("当前姿态或 GPS 原点无效。"));
    return;
  }

  sensor_msgs::NavSatFix goal;
  goal.header.stamp = ros::Time::now();
  goal.header.frame_id = "gps";
  goal.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
  goal.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  goal.latitude = origin_latitude_ + (target_y / kEarthRadiusMetres) * 180.0 / kPi;
  goal.longitude = origin_longitude_ + (target_x / longitude_scale) * 180.0 / kPi;
  goal.altitude = data.fix_received && std::isfinite(data.fix.altitude) ? data.fix.altitude : 0.0;
  goal_publisher_.publish(goal);
  appendEvent(QStringLiteral("已发布车头正前方 8 m GPS 目标：lat=%1 lon=%2（局部 %3, %4 m）")
                  .arg(goal.latitude, 0, 'f', 10)
                  .arg(goal.longitude, 0, 'f', 10)
                  .arg(target_x, 0, 'f', 2)
                  .arg(target_y, 0, 'f', 2));
}

void MainWindow::cancelNavigation()
{
  if (!master_online_ || !ros_interfaces_ready_)
  {
    QMessageBox::information(this, QStringLiteral("导航未连接"),
                             QStringLiteral("ROS master 离线，未发布取消消息。"));
    return;
  }
  actionlib_msgs::GoalID cancel;
  cancel.stamp = ros::Time(0);
  cancel.id.clear();
  cancel_publisher_.publish(cancel);
  appendEvent(QStringLiteral("已向 /move_base/cancel 发布空 GoalID（取消全部当前目标）。"), true);
}

void MainWindow::resetStaticError()
{
  if (!master_online_ || !ros_interfaces_ready_)
  {
    QMessageBox::information(this, QStringLiteral("节点未连接"),
                             QStringLiteral("ROS master 离线，未发送重置请求。"));
    return;
  }
  error_reset_publisher_.publish(std_msgs::Empty());
  appendEvent(QStringLiteral("已请求重置 GPS 静态误差参考点。"));
}

void MainWindow::publishRabbitTarget()
{
  callTriggerService("/rabbitmq_bridge/publish_latest", rabbit_publish_button_,
                     QStringLiteral("发送 RabbitMQ 缓存目标"));
}

void MainWindow::clearRabbitTarget()
{
  callTriggerService("/rabbitmq_bridge/clear_latest", rabbit_clear_button_,
                     QStringLiteral("清空 RabbitMQ 缓存目标"));
}

void MainWindow::callTriggerService(const std::string& service_name, QPushButton* button,
                                    const QString& action_name)
{
  if (!master_online_ || !ros_interfaces_ready_)
  {
    QMessageBox::information(this, QStringLiteral("服务不可用"), QStringLiteral("ROS master 当前离线。"));
    return;
  }
  button->setEnabled(false);
  auto* watcher = new QFutureWatcher<QString>(this);
  connect(watcher, &QFutureWatcher<QString>::finished, this,
          [this, watcher, button, action_name]() {
            const QString result = watcher->result();
            const bool success = result.startsWith(QStringLiteral("OK|"));
            const QString message = result.section('|', 1);
            appendEvent(action_name + QStringLiteral("：") + message, !success);
            if (!success)
              QMessageBox::warning(this, action_name, message);
            button->setEnabled(true);
            watcher->deleteLater();
          });
  watcher->setFuture(QtConcurrent::run([service_name]() {
    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<std_srvs::Trigger>(service_name, false);
    if (!client.waitForExistence(ros::Duration(0.5)))
      return QStringLiteral("ERR|服务未启动或未注册");
    std_srvs::Trigger call;
    if (!client.call(call))
      return QStringLiteral("ERR|服务调用失败");
    return QString(call.response.success ? QStringLiteral("OK|") : QStringLiteral("ERR|")) +
           QString::fromStdString(call.response.message);
  }));
}

void MainWindow::toggleRecording()
{
  if (recorder_.state() != QProcess::NotRunning)
  {
    recorder_stop_requested_ = true;
    appendEvent(QStringLiteral("正在停止 rosbag 录制……"));
    recorder_.terminate();
    QTimer::singleShot(2000, this, [this]() {
      if (recorder_.state() != QProcess::NotRunning)
      {
        appendEvent(QStringLiteral("录包进程未在 2 秒内退出，执行强制停止。"), true);
        recorder_.kill();
      }
    });
    return;
  }

  recorder_error_ = false;
  recorder_stop_requested_ = false;
  recorder_.setWorkingDirectory(QStringLiteral("/home/robot/robot_ws"));
  recorder_.setProgram(QStringLiteral("/home/robot/robot_ws/scripts/record_rosbag.sh"));
  recorder_.setArguments({ QStringLiteral("mode1") });
  recorder_.setProcessChannelMode(QProcess::SeparateChannels);
  recorder_.start();
  appendEvent(QStringLiteral("已启动 mode1 rosbag 录制子进程。"));
}

void MainWindow::handleRecorderFinished(int exit_code, QProcess::ExitStatus exit_status)
{
  const bool normal = recorder_stop_requested_ ||
                      (exit_status == QProcess::NormalExit &&
                       (exit_code == 0 || exit_code == 130 || exit_code == 143));
  recorder_error_ = !normal;
  appendEvent(recorder_stop_requested_ ? QStringLiteral("录包已由操作员停止。")
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

}  // namespace autolabor_operator_gui
