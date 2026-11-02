#include <autolabor_operator_gui/main_window.h>

#include <QButtonGroup>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QUrl>

namespace autolabor_operator_gui
{
QWidget* MainWindow::buildSurroundPortBar(QWidget* parent)
{
  auto* bar = new QWidget(parent);
  bar->setObjectName(QStringLiteral("surroundPortBar"));
  auto* row = new QHBoxLayout(bar);
  row->setContentsMargins(0, 0, 0, 0);
  auto* group = new QButtonGroup(bar);
  group->setExclusive(true);
  row->addWidget(new QLabel(QStringLiteral("相机端口"), bar));
  for (int p=0; p<3; ++p)
  {
    auto* button = new QPushButton(QStringLiteral("端口 %1").arg(p), bar);
    button->setObjectName(QStringLiteral("surroundPort%1Button").arg(p));
    button->setCheckable(true);
    button->setChecked(p == 0);
    button->setMinimumHeight(36);
    button->setStyleSheet(QStringLiteral("QPushButton:checked {background:#047857;color:white;border:2px solid #34d399;}"));
    group->addButton(button, p);
    row->addWidget(button);
    connect(button, &QPushButton::clicked, this, [this, p]() {
      if (surround_preview_) surround_preview_->selectPort(p);
    });
  }
  surround_port_status_ = new QLabel(QStringLiteral("正在连接三路识别……"), bar);
  surround_port_status_->setWordWrap(true);
  row->addWidget(surround_port_status_, 1);
  return bar;
}

void MainWindow::setupSurroundPreview()
{
  if (!surround_mode_) return;
  surround_preview_ = new SurroundPreviewClient(
      QUrl(QString::fromUtf8(qgetenv("SURROUND_PREVIEW_URL"))), this);
  connect(surround_preview_, &SurroundPreviewClient::changed, this, &MainWindow::refreshSurroundPreview);
  refreshSurroundPreview();
}

void MainWindow::refreshSurroundPreview()
{
  if (!surround_preview_) return;
  const auto& frame = surround_preview_->frame();
  const auto& meta = frame.metadata;
  const bool live = surround_preview_->live();
  const int port = surround_preview_->port();
  const QString status = surround_preview_->status();
  const auto show = [this, &status](QLabel* label, const QImage& image) {
    if (!label) return;
    if (image.isNull())
    {
      label->clear(); label->setText(status);
      label->setProperty("surroundImageKey", qlonglong(0));
    }
    else if (label->property("surroundImageKey").toLongLong() != image.cacheKey() ||
             label->property("surroundImageSize").toSize() != label->size())
    {
      updateImageLabel(label, image, status);
      label->setProperty("surroundImageKey", image.cacheKey());
      label->setProperty("surroundImageSize", label->size());
    }
  };
  show(vision_camera_preview_, frame.annotated);
  if (surround_port_status_) surround_port_status_->setText(status);
  values_["vision_image_source"]->setText(
      QStringLiteral("RX1 · 端口 %1 · GDC 透视图 → J6M BPU").arg(port));
  values_["vision_image_source"]->setToolTip(
      status + QStringLiteral(" · 三路持续识别；镜头参数精度待验证。"));
  values_["vision_camera_status"]->setText(live ? QStringLiteral("RX1 · 端口 %1 在线").arg(port) : status);
  values_["vision_camera_format"]->setText(QStringLiteral("960 × 540 / GDC 透视图"));
  values_["vision_yolo_status"]->setText(live ? QStringLiteral("J6M BPU 识别中") : QStringLiteral("等待有效结果"));
  values_["vision_model"]->setText(QStringLiteral("YOLO11-GAM + 五类材质分类"));
  values_["vision_backend_running"]->setText(QStringLiteral("三路环视 / J6M BPU"));
  // Board processing duration, not an invented exposure-to-display latency.
  values_["vision_inference"]->setText(live ?
      QString::number(meta["timing"].toObject()["processing_ms"].toDouble(), 'f', 1) : QStringLiteral("--"));
  values_["vision_fps"]->setText(live ? QString::number(meta["inference_fps"].toDouble(), 'f', 1) : QStringLiteral("--"));
  const auto objects = meta["objects"].toArray();
  setStatus("camera", live ? Health::Good : Health::Warning,
      live ? QStringLiteral("在线") : QStringLiteral("等待画面"),
      QStringLiteral("端口 %1 · GDC 960×540").arg(port));
  setStatus("yolo", live ? Health::Good : Health::Warning,
      live ? QStringLiteral("%1 目标").arg(objects.size()) : QStringLiteral("等待结果"),
      QStringLiteral("J6M BPU"));
  values_["vision_detection_count"]->setText(live ? QString::number(objects.size()) : QStringLiteral("--"));
  QStringList lines;
  if (!live) lines << status;
  else
  {
    lines << QStringLiteral("端口 %1 · 帧 %2 · 板端整帧处理 %3 ms · SDK 读帧至结果 %4 ms")
        .arg(port).arg(meta["source"].toObject()["sequence"].toDouble(), 0, 'f', 0)
        .arg(meta["timing"].toObject()["processing_ms"].toDouble(), 0, 'f', 1)
        .arg(meta["read_to_result_ms"].toDouble(), 0, 'f', 1);
    if (objects.isEmpty()) lines << QStringLiteral("当前帧未检测到目标");
    int index = 0;
    for (const auto& item : objects)
    {
      const auto object = item.toObject();
      const QString classified = object["probabilities"].isNull() ? QStringLiteral("待分类") :
          QStringLiteral("%1 · C:%2%").arg(object["material"].toString())
              .arg(object["classify_confidence"].toDouble()*100, 0, 'f', 1);
      lines << QStringLiteral("%1. 垃圾 D:%2% · %3")
          .arg(++index).arg(object["detect_confidence"].toDouble()*100, 0, 'f', 1).arg(classified);
      if (index >= 20) break;
    }
    lines << QStringLiteral("当前为图像识别观察；距离、跟踪编号与地图坐标未提供。");
  }
  if (vision_detections_ && vision_detections_->toPlainText() != lines.join(QLatin1Char('\n')))
    vision_detections_->setPlainText(lines.join(QLatin1Char('\n')));
}
}  // namespace autolabor_operator_gui
