#include <autolabor_operator_gui/main_window.h>
#include <QDebug>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPixmap>
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
    app_subtitle_->setText(QStringLiteral("THREE-CAMERA BEV · CUDA YOLO · 图像识别观察"));
  bev_preview_ = new BevPreviewClient(QUrl(configured.isEmpty()
      ? QStringLiteral("http://127.0.0.1:18884") : QString::fromUtf8(configured)), this);
  connect(bev_preview_, &BevPreviewClient::changed, this, &MainWindow::refreshBevPreview);
  refreshBevPreview();
}

void MainWindow::refreshBevPreview()
{
  if (!bev_preview_ || !bev_raw_preview_) return;
  if (!qgetenv("BEV_PREVIEW_URL").isEmpty() && app_subtitle_)
    app_subtitle_->setText(QStringLiteral("THREE-CAMERA BEV · CUDA YOLO · 图像识别观察"));
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
}
