#include <autolabor_operator_gui/zed_camera_panel.h>

#include <dynamic_reconfigure/Reconfigure.h>
#include <ros/ros.h>
#include <ros/topic.h>
#include <QCheckBox>
#include <QFormLayout>
#include <QFutureWatcher>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QtConcurrent/QtConcurrentRun>

namespace autolabor_operator_gui
{
namespace
{
struct Result
{
  bool success = false;
  QString message;
  QString acquisition;
  dynamic_reconfigure::Config config;
};

void addBool(dynamic_reconfigure::Config& config, const std::string& name, bool value)
{
  dynamic_reconfigure::BoolParameter param;
  param.name = name;
  param.value = value;
  config.bools.push_back(param);
}

void addInt(dynamic_reconfigure::Config& config, const std::string& name, int value)
{
  dynamic_reconfigure::IntParameter param;
  param.name = name;
  param.value = value;
  config.ints.push_back(param);
}
}  // namespace

ZedCameraPanel::ZedCameraPanel(const std::string& camera_node, QWidget* parent)
  : QWidget(parent), camera_node_(camera_node)
{
  setObjectName(QStringLiteral("zedCameraPanel"));
  auto* layout = new QVBoxLayout(this);
  auto* note = new QLabel(QStringLiteral(
      "ZED 原生成像参数 · 自动曝光与增益联动。手动值使用 ZED 0–100 档位。"
      "调整对本次相机会话生效，重启后加载启动配置。"), this);
  note->setWordWrap(true);
  layout->addWidget(note);
  acquisition_ = new QLabel(QStringLiteral("采集配置：待读取"), this);
  acquisition_->setWordWrap(true);
  acquisition_->setObjectName(QStringLiteral("zedAcquisition"));
  layout->addWidget(acquisition_);

  auto* form = new QFormLayout();
  auto_exposure_ = new QCheckBox(QStringLiteral("自动曝光 / 增益"), this);
  auto_exposure_->setObjectName(QStringLiteral("zedAutoExposure"));
  auto_exposure_->setChecked(true);
  form->addRow(auto_exposure_);
  auto addControl = [this, form](const char* name, const QString& label,
                                 int minimum, int maximum, int value) {
    auto* input = new QSpinBox(this);
    input->setObjectName(QString::fromLatin1(name));
    input->setRange(minimum, maximum);
    input->setValue(value);
    input->setKeyboardTracking(false);
    integers_[name] = input;
    form->addRow(label, input);
  };
  addControl("exposure", QStringLiteral("曝光档位"), 0, 100, 100);
  addControl("gain", QStringLiteral("增益档位"), 0, 100, 100);
  auto_whitebalance_ = new QCheckBox(QStringLiteral("自动白平衡"), this);
  auto_whitebalance_->setObjectName(QStringLiteral("zedAutoWhitebalance"));
  auto_whitebalance_->setChecked(true);
  form->addRow(auto_whitebalance_);
  addControl("whitebalance_temperature", QStringLiteral("白平衡色温"), 28, 65, 42);
  integers_.at("whitebalance_temperature")->setSuffix(QStringLiteral(" ×100 K"));
  addControl("brightness", QStringLiteral("亮度"), 0, 8, 4);
  addControl("contrast", QStringLiteral("对比度"), 0, 8, 4);
  addControl("saturation", QStringLiteral("饱和度"), 0, 8, 4);
  addControl("sharpness", QStringLiteral("锐度"), 0, 8, 4);
  addControl("gamma", QStringLiteral("伽马档位"), 1, 9, 8);
  addControl("depth_confidence", QStringLiteral("深度置信度阈值"), 1, 100, 50);
  addControl("depth_texture_conf", QStringLiteral("深度纹理阈值"), 1, 100, 100);
  integers_.at("depth_confidence")->setToolTip(
      QStringLiteral("值越小，深度过滤越严格、有效点越少；不改变 YOLO 检测阈值。"));
  integers_.at("depth_texture_conf")->setToolTip(
      QStringLiteral("值越小，低纹理区域过滤越严格；不改变 YOLO 检测阈值。"));
  layout->addLayout(form);
  auto* buttons = new QHBoxLayout();
  query_ = new QPushButton(QStringLiteral("读取 ZED 参数"), this);
  apply_ = new QPushButton(QStringLiteral("应用 ZED 参数"), this);
  query_->setObjectName(QStringLiteral("zedQuery"));
  apply_->setObjectName(QStringLiteral("zedApply"));
  buttons->addWidget(query_);
  buttons->addWidget(apply_);
  layout->addLayout(buttons);
  automatic_ = new QPushButton(QStringLiteral("恢复自动曝光与白平衡"), this);
  automatic_->setObjectName(QStringLiteral("zedAutomatic"));
  layout->addWidget(automatic_);
  status_ = new QLabel(QStringLiteral("相机参数未读取"), this);
  status_->setObjectName(QStringLiteral("zedStatus"));
  status_->setWordWrap(true);
  layout->addWidget(status_);
  connect(query_, &QPushButton::clicked, this, [this]() { request(false); });
  connect(apply_, &QPushButton::clicked, this, [this]() { request(true); });
  connect(automatic_, &QPushButton::clicked, this, [this]() { request(true, true); });
  connect(auto_exposure_, &QCheckBox::toggled, this, [this]() { updateEnabled(); });
  connect(auto_whitebalance_, &QCheckBox::toggled, this, [this]() { updateEnabled(); });
  updateEnabled();
}

void ZedCameraPanel::setOnline(bool online)
{
  if (!online && online_)
  {
    loaded_ = false;
    status_->setText(QStringLiteral("ROS 已离线，重连后请重新读取相机参数。"));
  }
  online_ = online;
  updateEnabled();
}

void ZedCameraPanel::updateEnabled()
{
  const bool available = online_ && !busy_;
  query_->setEnabled(available);
  apply_->setEnabled(available && loaded_);
  automatic_->setEnabled(available);
  auto_exposure_->setEnabled(available && loaded_);
  auto_whitebalance_->setEnabled(available && loaded_);
  for (const auto& item : integers_)
    item.second->setEnabled(available && loaded_);
  integers_.at("exposure")->setEnabled(available && loaded_ && !auto_exposure_->isChecked());
  integers_.at("gain")->setEnabled(available && loaded_ && !auto_exposure_->isChecked());
  integers_.at("whitebalance_temperature")->setEnabled(
      available && loaded_ && !auto_whitebalance_->isChecked());
}

bool ZedCameraPanel::displayConfig(const dynamic_reconfigure::Config& config)
{
  std::map<std::string, int> integers;
  std::map<std::string, bool> booleans;
  for (const auto& item : config.ints)
    integers[item.name] = item.value;
  for (const auto& item : config.bools)
    booleans[item.name] = item.value;
  // A missing/changed driver schema must not turn unset controls into writes.
  if (!booleans.count("auto_exposure_gain") || !booleans.count("auto_whitebalance"))
    return false;
  for (const auto& item : integers_)
    if (!integers.count(item.first))
      return false;
  for (const auto& item : integers_)
    item.second->setValue(integers.at(item.first));
  auto_exposure_->setChecked(booleans.at("auto_exposure_gain"));
  auto_whitebalance_->setChecked(booleans.at("auto_whitebalance"));
  return true;
}

void ZedCameraPanel::request(bool apply, bool restore_auto)
{
  if (!online_ || busy_ || (apply && !restore_auto && !loaded_))
    return;
  dynamic_reconfigure::Config requested;
  if (apply)
  {
    addBool(requested, "auto_exposure_gain", restore_auto || auto_exposure_->isChecked());
    addBool(requested, "auto_whitebalance", restore_auto || auto_whitebalance_->isChecked());
    if (!restore_auto)
      for (const auto& item : integers_)
        addInt(requested, item.first, item.second->value());
  }
  busy_ = true;
  status_->setText(apply ? QStringLiteral("正在应用 ZED 参数…")
                        : QStringLiteral("正在读取 ZED 参数…"));
  updateEnabled();
  const std::string camera_node = camera_node_;
  auto* watcher = new QFutureWatcher<Result>(this);
  connect(watcher, &QFutureWatcher<Result>::finished, this, [this, watcher]() {
    const Result result = watcher->result();
    busy_ = false;
    loaded_ = online_ && result.success && displayConfig(result.config);
    const QString message = result.success && !loaded_
        ? QStringLiteral("驱动参数格式不匹配或 ROS 已离线，请重新读取。") : result.message;
    status_->setText(message);
    if (!result.acquisition.isEmpty())
      acquisition_->setText(result.acquisition);
    updateEnabled();
    Q_EMIT operationEvent(QStringLiteral("ZED：") + message, !loaded_);
    watcher->deleteLater();
  });
  watcher->setFuture(QtConcurrent::run([camera_node, requested, apply]() {
    Result result;
    try
    {
      ros::NodeHandle node;
      if (apply)
      {
        ros::ServiceClient client = node.serviceClient<dynamic_reconfigure::Reconfigure>(
            camera_node + "/set_parameters", false);
        if (!client.waitForExistence(ros::Duration(0.75)))
        {
          result.message = QStringLiteral("ZED 参数服务未就绪，请连接相机并启动 ZED 驱动。");
          return result;
        }
        dynamic_reconfigure::Reconfigure call;
        call.request.config = requested;
        if (!client.call(call))
        {
          result.message = QStringLiteral("ZED 参数设置失败，请检查驱动日志后重试。");
          return result;
        }
        result.config = call.response.config;
        result.message = QStringLiteral("ZED 驱动已接受参数；画面效果以实时预览为准。");
      }
      else
      {
        const auto config = ros::topic::waitForMessage<dynamic_reconfigure::Config>(
            camera_node + "/parameter_updates", node, ros::Duration(1.0));
        if (!config)
        {
          result.message = QStringLiteral("没有收到 ZED 参数，请连接相机并启动 ZED 驱动。");
          return result;
        }
        result.config = *config;
        result.message = QStringLiteral("已读取 ZED 驱动配置。自动模式下显示的是手动预设档位。");
      }
      std::string resolution, depth;
      int grab = 0;
      double publish = 0.0;
      if (node.getParam(camera_node + "/general/grab_resolution", resolution) &&
          node.getParam(camera_node + "/general/grab_frame_rate", grab) &&
          node.getParam(camera_node + "/general/pub_frame_rate", publish) &&
          node.getParam(camera_node + "/depth/depth_mode", depth))
        result.acquisition = QStringLiteral("%1 · 采集 %2 FPS · 输出上限 %3 FPS · 深度 %4\n"
                                             "以上为配置值；实际处理速度取决于光照与系统负载。")
            .arg(QString::fromStdString(resolution)).arg(grab).arg(publish)
            .arg(QString::fromStdString(depth));
      result.success = true;
    }
    catch (const std::exception& error)
    {
      result.message = QStringLiteral("ZED 参数操作失败：") + QString::fromUtf8(error.what());
    }
    return result;
  }));
}
}  // namespace autolabor_operator_gui
