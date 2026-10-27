#include <autolabor_operator_gui/main_window.h>
#include <QDebug>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPixmap>
#include <QPushButton>
#include <QFutureWatcher>
#include <QtConcurrent/QtConcurrentRun>
#include <std_srvs/Trigger.h>
#include <QTimer>
#include <QVBoxLayout>

namespace autolabor_operator_gui
{
QWidget* MainWindow::buildBevPreviewPage()
{
  auto* page = new QWidget(this);
  page->setObjectName(QStringLiteral("bevPreviewPage"));
  auto* root = new QVBoxLayout(page);
  auto* notice = new QLabel(QStringLiteral(
      "左/右硬件 GDC + 前视 ZED → 地面 BEV → YOLO11-GAM。左右相机已翻正；"
      "外参为安装估算，曝光未同步，BEV 检测效果待样本验证。"), page);
  notice->setWordWrap(true);
  root->addWidget(notice);
  auto* controls = new QHBoxLayout();
  bev_cleaning_start_ = new QPushButton(QStringLiteral("开始视野清扫"), page);
  bev_cleaning_start_->setObjectName(QStringLiteral("startBevCleaningButton"));
  bev_cleaning_stop_ = new QPushButton(QStringLiteral("停止本次清扫"), page);
  bev_cleaning_stop_->setObjectName(QStringLiteral("stopBevCleaningButton"));
  controls->addWidget(bev_cleaning_start_);
  controls->addWidget(bev_cleaning_stop_);
  controls->addStretch();
  root->addLayout(controls);
  bev_cleaning_status_ = new QLabel(QStringLiteral("等待 BEV 清扫后端…"), page);
  bev_cleaning_status_->setObjectName(QStringLiteral("bevCleaningStatus"));
  bev_cleaning_status_->setWordWrap(true);
  root->addWidget(bev_cleaning_status_);
  connect(bev_cleaning_start_, &QPushButton::clicked, this, [this]() { requestBevCleaning(true); });
  connect(bev_cleaning_stop_, &QPushButton::clicked, this, [this]() { requestBevCleaning(false); });
  auto* panels = new QHBoxLayout();
  const auto add = [page, panels](const QString& title, const QString& name, QLabel** out) {
    auto* group = new QGroupBox(title, page);
    auto* box = new QVBoxLayout(group);
    *out = new QLabel(QStringLiteral("等待三路 BEV / YOLO 图像…"), group);
    (*out)->setObjectName(name);
    (*out)->setAlignment(Qt::AlignCenter);
    (*out)->setMinimumSize(360, 360);
    (*out)->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    (*out)->setStyleSheet(QStringLiteral("background:#080d13;color:#dce7f4;"));
    box->addWidget(*out, 1);
    panels->addWidget(group, 1);
  };
  add(QStringLiteral("融合原图 · 上方为前 · 黑色为无图像区域"),
      QStringLiteral("bevRawPreview"), &bev_raw_preview_);
  add(QStringLiteral("YOLO11-GAM 处理后图像 · 五类垃圾"),
      QStringLiteral("bevYoloPreview"), &bev_result_preview_);
  root->addLayout(panels, 1);
  bev_preview_status_ = new QLabel(QStringLiteral("正在连接 BEV 识别服务…"), page);
  bev_preview_status_->setObjectName(QStringLiteral("bevPreviewStatus"));
  bev_preview_status_->setWordWrap(true);
  root->addWidget(bev_preview_status_);
  bev_detections_ = new QPlainTextEdit(page);
  bev_detections_->setObjectName(QStringLiteral("bevDetectionResults"));
  bev_detections_->setReadOnly(true);
  bev_detections_->setMaximumHeight(140);
  root->addWidget(bev_detections_);
  return page;
}

void MainWindow::setupBevPreview()
{
  const auto configured = qgetenv("BEV_PREVIEW_URL");
  if (!configured.isEmpty() && app_subtitle_)
    app_subtitle_->setText(QStringLiteral("THREE-CAMERA BEV · CUDA YOLO · 视野逐步清扫"));
  bev_preview_ = new BevPreviewClient(QUrl(configured.isEmpty()
      ? QStringLiteral("http://127.0.0.1:18884") : QString::fromUtf8(configured)), this);
  connect(bev_preview_, &BevPreviewClient::changed, this, &MainWindow::refreshBevPreview);
  refreshBevPreview();
}

void MainWindow::refreshBevPreview()
{
  if (!bev_preview_ || !bev_raw_preview_) return;
  if (!qgetenv("BEV_PREVIEW_URL").isEmpty() && app_subtitle_)
    app_subtitle_->setText(QStringLiteral("THREE-CAMERA BEV · CUDA YOLO · 视野逐步清扫"));
  const auto& frame = bev_preview_->frame();
  updateImageLabel(bev_raw_preview_, frame.raw, bev_preview_->status());
  updateImageLabel(bev_result_preview_, frame.annotated, bev_preview_->status());
  bev_preview_status_->setText(bev_preview_->status());
  QStringList lines;
  if (!frame.raw.isNull())
  {
    const auto meta = frame.metadata;
    lines << QStringLiteral("%1 · CUDA · 同一融合帧 %2 · 实际识别 %3 fps · 推理 %4 ms")
        .arg(meta["model_name"].toString())
        .arg(meta["source_id"].toObject()["sequence"].toDouble(), 0, 'f', 0)
        .arg(meta["inference_fps"].toDouble(), 0, 'f', 1)
        .arg(meta["inference_ms"].toDouble(), 0, 'f', 1);
    const auto objects = meta["objects"].toArray();
    if (objects.isEmpty()) lines << QStringLiteral("本帧未检出目标（置信度阈值 25%）。");
    const QMap<QString, QString> names = {{"metal", "金属"}, {"plastic", "塑料"},
      {"paper", "纸"}, {"glass", "玻璃"}, {"kitchen_waste", "厨余"}};
    int index = 0;
    for (const auto& item : objects)
    {
      const auto obj = item.toObject();
      const auto name = obj["class_name"].toString();
      lines << QStringLiteral("%1. %2 · 置信度 %3%")
          .arg(++index).arg(names.value(name, name)).arg(obj["confidence"].toDouble()*100, 0, 'f', 1);
      if (index >= 20) break;
    }
    const auto ground = meta["ground_objects"].toArray();
    for (int i = 0; i < qMin(20, ground.size()); ++i)
    {
      const auto point = ground[i].toObject();
      lines << QStringLiteral("BEV 相对位置 %1：前向 %2 m / 左向 %3 m（平面估算）")
          .arg(i+1).arg(point["x"].toDouble(), 0, 'f', 2).arg(point["y"].toDouble(), 0, 'f', 2);
    }
    if (!bev_screenshot_saved_ && !qgetenv("BEV_QT_SCREENSHOT").isEmpty())
    {
      bev_screenshot_saved_ = true;
      const auto path = QString::fromUtf8(qgetenv("BEV_QT_SCREENSHOT"));
      QTimer::singleShot(500, this, [this, path]() { grab().save(path); });
      qInfo() << "BEV_YOLO_FRAME_ACCEPTED" << meta["source_id"].toObject();
    }
  }
  else lines << bev_preview_->status();
  const auto text = lines.join(QLatin1Char('\n'));
  if (bev_detections_->toPlainText() != text) bev_detections_->setPlainText(text);
}

void MainWindow::refreshBevCleaning()
{
  if (!bev_cleaning_status_) return;
  QString json;
  ros::WallTime received;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    json = bev_cleaner_status_json_;
    received = bev_cleaner_received_;
  }
  const auto status = QJsonDocument::fromJson(json.toUtf8()).object();
  const bool fresh = !received.isZero() && (ros::WallTime::now()-received).toSec() < 2.;
  const bool busy = bev_cleaning_request_ != nullptr;
  bev_cleaning_start_->setEnabled(fresh && !busy && !status["active"].toBool() &&
                                  status["visible_count"].toInt() > 0);
  bev_cleaning_stop_->setEnabled(!busy && status["active"].toBool());
  if (busy) return;
  if (!fresh)
  {
    bev_cleaning_status_->setText(QStringLiteral("BEV 清扫后端未连接；三路图像识别可独立显示。"));
    return;
  }
  QString text = QStringLiteral("%1 · %2\n%3\n视野目标 %4 · 清扫轨迹通过 %5 · 未完成 %6")
      .arg(status["state"].toString(), status["reason"].toString(), status["perception"].toString())
      .arg(status["visible_count"].toInt()).arg(status["passed_count"].toInt()).arg(status["skipped_count"].toInt());
  const auto target = status["selected"].toObject();
  if (!target.isEmpty())
    text += QStringLiteral("\n正在处理：%1；完成后重新观察并规划下一处。")
        .arg(target["label"].toString());
  text += QStringLiteral("\n清扫机构未接入自动控制；当前执行清扫路径，拾取效果待验证。安装外参可后续配置。");
  bev_cleaning_status_->setText(text);
}

void MainWindow::requestBevCleaning(bool start)
{
  if (bev_cleaning_request_) return;
  const std::string service = start ? "/bev_cleaner/start" : "/bev_cleaner/cancel";
  bev_cleaning_status_->setText(start ? QStringLiteral("正在提交视野清扫…") : QStringLiteral("正在停止本次清扫…"));
  bev_cleaning_request_ = new QFutureWatcher<QString>(this);
  connect(bev_cleaning_request_, &QFutureWatcher<QString>::finished, this, [this]() {
    const auto result = bev_cleaning_request_->result();
    bev_cleaning_request_->deleteLater();
    bev_cleaning_request_ = nullptr;
    bev_cleaning_status_->setText(result);
  });
  bev_cleaning_request_->setFuture(QtConcurrent::run([service]() {
    if (!ros::service::waitForService(service, ros::Duration(.5)))
      return QStringLiteral("BEV 清扫服务未连接");
    std_srvs::Trigger request;
    if (!ros::service::call(service, request)) return QStringLiteral("BEV 清扫请求失败");
    return QString::fromStdString(request.response.message);
  }));
}
}
