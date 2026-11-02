#include <autolabor_operator_gui/main_window.h>
#include <QDebug>
#include <QGroupBox>
#include <QGridLayout>
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
QWidget* MainWindow::buildVisionBevPanel(QWidget* parent)
{
  auto* panel = new QWidget(parent);
  panel->setObjectName(QStringLiteral("visionBevPanel"));
  auto* root = new QVBoxLayout(panel);
  root->setContentsMargins(0,0,0,0);
  auto* notice = new QLabel(QStringLiteral("四路原图识别与融合定位 · 后视红色区域为车体，不参与识别和定位。"),panel);
  notice->setWordWrap(true);
  root->addWidget(notice);
  auto* grid = new QGridLayout();
  const QStringList cameras{"front","left","right","rear"};
  const QStringList titles{QStringLiteral("前视 · ZED"),QStringLiteral("左视 · 端口0"),
      QStringLiteral("右视 · 端口1"),QStringLiteral("后视 · 端口2")};
  for (int i=0;i<cameras.size();++i)
  {
    auto* group = new QGroupBox(titles[i],panel);
    auto* box = new QVBoxLayout(group);
    auto* image = new QLabel(QStringLiteral("等待原图识别结果…"),group);
    image->setObjectName(QStringLiteral("visionBev_%1").arg(cameras[i]));
    image->setAlignment(Qt::AlignCenter);
    image->setMinimumSize(240,135);
    image->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
    image->setStyleSheet(QStringLiteral("background:#080d13;color:#dce7f4;"));
    box->addWidget(image,1);
    auto* caption = new QLabel(QStringLiteral("等待结果"),group);
    box->addWidget(caption);
    vision_bev_views_[cameras[i]] = image;
    vision_bev_captions_[cameras[i]] = caption;
    grid->addWidget(group,i/2,i%2);
  }
  auto* combined = new QGroupBox(QStringLiteral("融合 BEV · 地面二维定位"),panel);
  auto* combined_box = new QVBoxLayout(combined);
  vision_bev_result_ = new QLabel(QStringLiteral("等待融合识别结果…"),combined);
  vision_bev_result_->setObjectName(QStringLiteral("visionBevCombined"));
  vision_bev_result_->setAlignment(Qt::AlignCenter);
  vision_bev_result_->setMinimumSize(300,300);
  vision_bev_result_->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
  vision_bev_result_->setStyleSheet(QStringLiteral("background:#080d13;color:#dce7f4;"));
  combined_box->addWidget(vision_bev_result_,1);
  auto* axes = new QLabel(QStringLiteral("上方为前 · 左方为左 · 单位：米\n相机组地面坐标；点位为接触点估算。"),combined);
  axes->setWordWrap(true);
  combined_box->addWidget(axes);
  grid->addWidget(combined,0,2,2,1);
  grid->setColumnStretch(0,2);grid->setColumnStretch(1,2);grid->setColumnStretch(2,3);
  grid->setRowStretch(0,1);grid->setRowStretch(1,1);
  root->addLayout(grid,1);
  vision_bev_status_ = new QLabel(QStringLiteral("等待四路识别服务…"),panel);
  vision_bev_status_->setObjectName(QStringLiteral("visionBevStatus"));
  vision_bev_status_->setWordWrap(true);
  root->addWidget(vision_bev_status_);
  vision_bev_points_ = new QPlainTextEdit(panel);
  vision_bev_points_->setObjectName(QStringLiteral("visionBevPoints"));
  vision_bev_points_->setReadOnly(true);
  vision_bev_points_->setMaximumHeight(110);
  root->addWidget(vision_bev_points_);
  return panel;
}

void MainWindow::refreshVisionBevPreview()
{
  if (!bev_preview_ || !vision_bev_panel_) return;
  const auto& frame = bev_preview_->frame();
  if (qgetenv("BEV_PREVIEW_URL").isEmpty() && frame.camera_images.isEmpty() && vision_bev_panel_->isHidden()) return;
  vision_bev_panel_->show();
  if (vision_legacy_panel_) vision_legacy_panel_->hide();
  const auto meta = frame.metadata;
  const auto detections = meta["camera_detections"].toObject();
  const auto sources = meta["source_metadata"].toObject()["sources"].toObject();
  for (auto it=vision_bev_views_.begin();it!=vision_bev_views_.end();++it)
  {
    updateImageLabel(it.value(),frame.camera_images.value(it.key()),bev_preview_->status());
    vision_bev_captions_[it.key()]->setText(frame.camera_images.contains(it.key())
        ? QStringLiteral("%1 个识别框 · 相机帧 %2%3").arg(detections[it.key()].toArray().size())
            .arg(sources[it.key()].toObject()["sequence"].toDouble(),0,'f',0)
            .arg(meta["detection_masks"].toObject().contains(it.key()) ? QStringLiteral(" · 车体遮罩已启用") : QString())
        : QStringLiteral("等待原图识别结果"));
  }
  updateImageLabel(vision_bev_result_,frame.annotated,bev_preview_->status());
  if (frame.raw.isNull())
  {
    vision_bev_status_->setText(bev_preview_->status());
    vision_bev_points_->setPlainText(QStringLiteral("等待新鲜图像和定位结果"));
    return;
  }
  const auto association = meta["ground_association"].toObject();
  vision_bev_status_->setText(QStringLiteral("%1 · %2 fps · 同组图像 %3\n原图框 %4 · 合并重复 %5 · 二维点 %6 · 清扫候选 %7")
      .arg(meta["model_name"].toString()).arg(meta["inference_fps"].toDouble(),0,'f',1)
      .arg(meta["source_id"].toObject()["sequence"].toDouble(),0,'f',0)
      .arg(association["input_boxes"].toInt()).arg(association["merged_duplicates"].toInt())
      .arg(association["unique_targets"].toInt()).arg(meta["ground_objects"].toArray().size()));
  const QMap<QString,QString> names{{"bottle","瓶子"},{"cup","杯子"},{"umbrella","雨伞"},
      {"person","人"},{"chair","椅子"},{"paper","纸"},{"plastic","塑料"},{"metal","金属"}};
  QStringList lines;
  const auto ground = meta["scene_ground_objects"].toArray();
  for (int i=0;i<qMin(30,ground.size());++i)
  {
    const auto point = ground[i].toObject();
    const auto label = point["label"].toString();
    lines << QStringLiteral("%1. %2 · %3% · 前向 x=%4 m / 左向 y=%5 m")
        .arg(i+1).arg(names.value(label,label)).arg(point["confidence"].toDouble()*100,0,'f',1)
        .arg(point["x"].toDouble(),0,'f',2).arg(point["y"].toDouble(),0,'f',2);
  }
  if (lines.isEmpty()) lines << QStringLiteral("本帧没有可定位的地面目标；各路原图框仍在上方显示。");
  const auto text = lines.join(QLatin1Char('\n'));
  if (vision_bev_points_->toPlainText()!=text) vision_bev_points_->setPlainText(text);
}

QWidget* MainWindow::buildBevPreviewPage()
{
  auto* page = new QWidget(this);
  page->setObjectName(QStringLiteral("bevPreviewPage"));
  auto* root = new QVBoxLayout(page);
  auto* notice = new QLabel(QStringLiteral(
      "环视 BEV：四路原图识别 → 地面二维定位 → 跨相机重叠观测合并。"
      "使用彩色图，不使用深度；黑色区域为盲区，曝光同步与定位精度待验证。"), page);
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
    *out = new QLabel(QStringLiteral("等待 BEV / 识别图像…"), group);
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
  add(QStringLiteral("物体二维定位 · 去重后目标"),
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
    app_subtitle_->setText(QStringLiteral("CAMERA BEV · 物体识别 · 视野逐步清扫"));
  bev_preview_ = new BevPreviewClient(QUrl(configured.isEmpty()
      ? QStringLiteral("http://127.0.0.1:18884") : QString::fromUtf8(configured)), this);
  connect(bev_preview_, &BevPreviewClient::changed, this, &MainWindow::refreshBevPreview);
  refreshBevPreview();
}

void MainWindow::refreshBevPreview()
{
  if (!bev_preview_ || !bev_raw_preview_) return;
  refreshVisionBevPreview();
  if (!qgetenv("BEV_PREVIEW_URL").isEmpty() && app_subtitle_)
    app_subtitle_->setText(QStringLiteral("CAMERA BEV · 物体识别 · 视野逐步清扫"));
  const auto& frame = bev_preview_->frame();
  updateImageLabel(bev_raw_preview_, frame.raw, bev_preview_->status());
  updateImageLabel(bev_result_preview_, frame.annotated, bev_preview_->status());
  if (overview_camera_preview_)
    updateImageLabel(overview_camera_preview_, frame.annotated, bev_preview_->status());
  if (overview_bev_status_) overview_bev_status_->setText(bev_preview_->status());
  bev_preview_status_->setText(bev_preview_->status());
  QStringList lines;
  if (!frame.raw.isNull())
  {
    const auto meta = frame.metadata;
    const auto sources = meta["source_metadata"].toObject()["sources"].toObject();
    const bool surround = sources.contains("rear");
    const auto view = QStringLiteral("%1路%2 BEV").arg(sources.size())
        .arg(surround ? QStringLiteral("环视") : QStringLiteral("相机"));
    if (app_subtitle_ && !qgetenv("BEV_PREVIEW_URL").isEmpty())
      app_subtitle_->setText(view + QStringLiteral(" · %1 · 视野逐步清扫").arg(meta["backend"].toString()));
    lines << view + (surround ? QStringLiteral(" · 前 / 左 / 右 / 后；近车盲区保留") : QString());
    lines << QStringLiteral("%1 · %2 · 同组图像 %3 · 实际识别 %4 fps · 处理 %5 ms")
        .arg(meta["model_name"].toString())
        .arg(meta["backend"].toString())
        .arg(meta["source_id"].toObject()["sequence"].toDouble(), 0, 'f', 0)
        .arg(meta["inference_fps"].toDouble(), 0, 'f', 1)
        .arg(meta["inference_ms"].toDouble(), 0, 'f', 1);
    if (meta["inference_image_space"] == "original_upright_cameras")
      lines << QStringLiteral("原图识别 → 地面接触点投影 → 重叠视野合并");
    if (meta["source_metadata"].toObject()["blend_mode"] == "single_source")
      lines << QStringLiteral("BEV 重叠区域单路取图；四路原图均参与识别。");
    const auto association = meta["ground_association"].toObject();
    if (!association.isEmpty())
      lines << QStringLiteral("原图框 %1 · 无效地面观测 %2 · 合并重复 %3 · 二维目标 %4")
          .arg(association["input_boxes"].toInt()).arg(association["rejected"].toInt())
          .arg(association["merged_duplicates"].toInt()).arg(association["unique_targets"].toInt());
    const auto objects = meta["objects"].toArray();
    if (objects.isEmpty()) lines << QStringLiteral("本帧无有效地面目标（识别置信度阈值 %1%）。")
        .arg(meta["confidence_threshold"].toDouble()*100, 0, 'f', 0);
    const QMap<QString, QString> names = {{"metal", "金属"}, {"plastic", "塑料"},
      {"paper", "纸"}, {"glass", "玻璃"}, {"kitchen_waste", "厨余"},
      {"bottle", "瓶子"}, {"cup", "杯子"}, {"person", "人"}, {"chair", "椅子"},
      {"umbrella", "雨伞"}, {"suitcase", "行李箱"}, {"book", "书"}, {"bowl", "碗"},
      {"backpack", "背包"}, {"tv", "显示器"}, {"potted plant", "盆栽"}};
    if (meta["cleaning_candidate_classes"].isArray())
    {
      QStringList allowed;
      for (const auto& name : meta["cleaning_candidate_classes"].toArray())
        allowed << names.value(name.toString(), name.toString());
      lines << QStringLiteral("清扫候选类别：%1 · 本帧候选 %2；其他物体仅显示识别位置。")
          .arg(allowed.isEmpty() ? QStringLiteral("无") : allowed.join(QStringLiteral(" / ")))
          .arg(meta["ground_objects"].toArray().size());
    }
    int index = 0;
    for (const auto& item : objects)
    {
      const auto obj = item.toObject();
      const auto name = obj["class_name"].toString();
      lines << QStringLiteral("%1. %2 · 置信度 %3%")
          .arg(++index).arg(names.value(name, name)).arg(obj["confidence"].toDouble()*100, 0, 'f', 1);
      if (index >= 20) break;
    }
    const auto ground = (meta.contains("scene_ground_objects") ? meta["scene_ground_objects"] : meta["ground_objects"]).toArray();
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
    bev_cleaning_status_->setText(QStringLiteral("BEV 清扫后端未连接；图像识别可独立显示。"));
    return;
  }
  QString text = QStringLiteral("%1 · %2\n%3\n视野目标 %4 · 清扫轨迹通过 %5 · 未完成 %6")
      .arg(status["state"].toString(), status["reason"].toString(), status["perception"].toString())
      .arg(status["visible_count"].toInt()).arg(status["passed_count"].toInt()).arg(status["skipped_count"].toInt());
  if (status["camera_count"].toInt() > 0)
    text += QStringLiteral("\n感知来源：%1路%2 BEV")
        .arg(status["camera_count"].toInt())
        .arg(status["camera_names"].toArray().contains(QStringLiteral("rear"))
                 ? QStringLiteral("环视") : QStringLiteral("相机"));
  if (status.contains("navigation_available"))
    text += QStringLiteral("\n导航：%1").arg(status["navigation_detail"].toString());
  const auto target = status["selected"].toObject();
  if (!target.isEmpty())
    text += QStringLiteral("\n正在处理：%1；完成后重新观察并规划下一处。")
        .arg(target["label"].toString());
  text += QStringLiteral("\n清扫机构未接入自动控制；任务记录为轨迹通过，拾取效果待验证。相机离地高度已用于投影，BEV与底盘的平面外参另行配置。");
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
