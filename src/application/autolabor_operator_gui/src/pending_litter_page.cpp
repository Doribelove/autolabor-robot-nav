#include <autolabor_operator_gui/pending_litter_page.h>
#include <autolabor_fod_msgs/StartPendingLitterCleaning.h>
#include <std_srvs/Trigger.h>

#include <QCryptographicHash>
#include <QDataStream>
#include <QGraphicsEllipseItem>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QGraphicsSimpleTextItem>
#include <QtConcurrent/QtConcurrentRun>
#include <QHBoxLayout>
#include <QLabel>
#include <QJsonDocument>
#include <QJsonObject>
#include <QListWidget>
#include <QPushButton>
#include <QShowEvent>
#include <QShortcut>
#include <QSplitter>
#include <QTimer>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <cmath>
#include <algorithm>
#include <ros/service.h>

namespace autolabor_operator_gui
{
namespace
{
constexpr const char* kStartCleaningService =
    "/pending_litter_cleaner/start_cleaning";
constexpr const char* kCancelCleaningService = "/pending_litter_cleaner/cancel";

double mapYaw(const geometry_msgs::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}

PendingLitterMapView::PendingLitterMapView(QWidget* parent) : QGraphicsView(parent)
{
  setObjectName(QStringLiteral("pendingLitterMap"));
  setScene(new QGraphicsScene(this));
  setBackgroundBrush(QColor("#dce3e8"));
  setDragMode(QGraphicsView::ScrollHandDrag);
  setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
  setResizeAnchor(QGraphicsView::AnchorViewCenter);
  setRenderHint(QPainter::Antialiasing);
  setMinimumSize(320, 250);
}

void PendingLitterMapView::setMap(const nav_msgs::OccupancyGrid& map)
{
  scene()->clear();
  markers_.clear();
  geometry_ = map.info;
  yaw_ = mapYaw(map.info.origin.orientation);
  QImage image(map.info.width, map.info.height, QImage::Format_RGB32);
  for (unsigned int y = 0; y < map.info.height; ++y)
  {
    auto* row = reinterpret_cast<QRgb*>(image.scanLine(map.info.height - 1 - y));
    for (unsigned int x = 0; x < map.info.width; ++x)
    {
      const int occupancy = map.data[static_cast<std::size_t>(y) * map.info.width + x];
      const int gray = occupancy < 0 ? 160 : qBound(0, 255 - occupancy * 255 / 100, 255);
      row[x] = qRgb(gray, gray, gray);
    }
  }
  map_item_ = scene()->addPixmap(QPixmap::fromImage(image));
  map_item_->setZValue(-1);
  scene()->setSceneRect(map_item_->boundingRect());
  needs_fit_ = true;
  if (isVisible()) fitMap();
}

QPointF PendingLitterMapView::worldToScene(const QPointF& point) const
{
  const double dx = point.x() - geometry_.origin.position.x;
  const double dy = point.y() - geometry_.origin.position.y;
  const double x = (std::cos(yaw_) * dx + std::sin(yaw_) * dy) / geometry_.resolution;
  const double y = (-std::sin(yaw_) * dx + std::cos(yaw_) * dy) / geometry_.resolution;
  return {x, static_cast<double>(geometry_.height) - y};
}

void PendingLitterMapView::setPoints(const QVector<PendingLitterPoint>& points)
{
  for (auto* marker : markers_) delete marker;
  markers_.clear();
  if (!map_item_) return;
  for (const auto& point : points)
  {
    const QColor color(point.state == "CONFIRMED" ? "#e02020" : "#808890");
    auto* marker = scene()->addEllipse(-4, -4, 8, 8, Qt::NoPen, color);
    marker->setPos(worldToScene(point.position));
    marker->setFlag(QGraphicsItem::ItemIgnoresTransformations);
    marker->setAcceptedMouseButtons(Qt::NoButton);
    marker->setData(0, point.label);
    marker->setZValue(1);
    auto* label = new QGraphicsSimpleTextItem(point.label, marker);
    QFont font;
    font.setPointSize(12);
    font.setBold(true);
    label->setFont(font);
    label->setBrush(color);
    label->setPos(8, -13);
    label->setAcceptedMouseButtons(Qt::NoButton);
    markers_.push_back(marker);
  }
}

void PendingLitterMapView::fitMap()
{
  if (!map_item_) return;
  fitInView(map_item_->boundingRect(), Qt::KeepAspectRatio);
  needs_fit_ = false;
}

void PendingLitterMapView::showEvent(QShowEvent* event)
{
  QGraphicsView::showEvent(event);
  // The tab/splitter completes its final geometry after the show event.
  // Fit then, otherwise the first view can crop the map vertically.
  QTimer::singleShot(0, this, [this]() { if (needs_fit_) fitMap(); });
}

void PendingLitterMapView::wheelEvent(QWheelEvent* event)
{
  if (!map_item_ || event->angleDelta().y() == 0) { event->ignore(); return; }
  const double factor = event->angleDelta().y() > 0 ? 1.2 : 1.0 / 1.2;
  const double scale_value = transform().m11() * factor;
  if (scale_value >= 0.01 && scale_value <= 200.0) scale(factor, factor);
  event->accept();
}

PendingLitterPage::PendingLitterPage(QWidget* parent) : QWidget(parent)
{
  setObjectName(QStringLiteral("pendingLitterPage"));
  auto* layout = new QVBoxLayout(this);
  auto* controls = new QHBoxLayout;
  status_ = new QLabel(QStringLiteral("等待静态地图与有效全局定位"), this);
  status_->setObjectName(QStringLiteral("pendingLitterStatus"));
  status_->setWordWrap(true);
  controls->addWidget(status_, 1);
  fit_button_ = new QPushButton(QStringLiteral("显示整张地图"), this);
  fit_button_->setEnabled(false);
  controls->addWidget(fit_button_);
  auto* columns = new QSplitter(Qt::Horizontal, this);
  auto* splitter = new QSplitter(Qt::Vertical, columns);
  auto* map_panel = new QWidget(splitter);
  auto* map_layout = new QVBoxLayout(map_panel);
  map_layout->setContentsMargins(0, 0, 0, 0);
  map_layout->addLayout(controls);
  map_view_ = new PendingLitterMapView(map_panel);
  map_layout->addWidget(map_view_, 1);
  auto* lower = new QWidget(splitter);
  auto* lower_layout = new QVBoxLayout(lower);
  lower_layout->setContentsMargins(0, 4, 0, 0);
  lower_layout->addWidget(new QLabel(QStringLiteral("待清扫垃圾列表"), lower));
  delete_button_ = new QPushButton(QStringLiteral("删除选中点"), lower);
  delete_button_->setObjectName(QStringLiteral("deletePendingLitterButton"));
  delete_button_->setEnabled(false);
  delete_button_->setToolTip(QStringLiteral("同步删除地图标记；后续重新满足5帧确认及定位条件时可以再次登记"));
  auto* pending_buttons = new QHBoxLayout;
  pending_buttons->addWidget(delete_button_);
  add_button_ = new QPushButton(QStringLiteral("加入清扫"), lower);
  add_button_->setObjectName(QStringLiteral("addPendingLitterToCleaningButton"));
  pending_buttons->addWidget(add_button_);
  lower_layout->addLayout(pending_buttons);
  list_ = new QListWidget(lower);
  list_->setObjectName(QStringLiteral("pendingLitterList"));
  list_->setMinimumHeight(120);
  list_->setSpacing(5);
  lower_layout->addWidget(list_);
  splitter->addWidget(map_panel);
  splitter->addWidget(lower);
  splitter->setStretchFactor(0, 4);
  splitter->setStretchFactor(1, 1);
  splitter->setSizes({620, 180});
  auto* cleaning = new QWidget(columns);
  cleaning->setMinimumWidth(300);
  auto* cleaning_layout = new QVBoxLayout(cleaning);
  cleaning_layout->setContentsMargins(8, 0, 0, 0);
  cleaning_layout->addWidget(new QLabel(QStringLiteral("清扫栏"), cleaning));
  auto* cleaning_buttons = new QHBoxLayout;
  return_button_ = new QPushButton(QStringLiteral("删除"), cleaning);
  return_button_->setObjectName(QStringLiteral("removeCleaningLitterButton"));
  return_button_->setToolTip(QStringLiteral("移出清扫栏，返回待清扫垃圾列表；保留地图标记"));
  clean_button_ = new QPushButton(QStringLiteral("清扫"), cleaning);
  clean_button_->setObjectName(QStringLiteral("cleanLitterButton"));
  clean_button_->setToolTip(QStringLiteral(
      "先由现有导航到达垃圾附近，再由视觉伺服前进覆盖所选地图坐标；仅10cm内的前进轨迹算清扫完成"));
  cancel_clean_button_ = new QPushButton(QStringLiteral("取消清扫"), cleaning);
  cancel_clean_button_->setObjectName(QStringLiteral("cancelLitterCleaningButton"));
  cleaning_buttons->addWidget(return_button_);
  cleaning_buttons->addWidget(clean_button_);
  cleaning_buttons->addWidget(cancel_clean_button_);
  cleaning_layout->addLayout(cleaning_buttons);
  cleaning_notice_ = new QLabel(QStringLiteral(
      "选择已确认点后启动：普通导航到附近，随后视觉伺服接管；倒车经过不计为清扫。"), cleaning);
  cleaning_notice_->setObjectName(QStringLiteral("cleaningLitterNotice"));
  cleaning_notice_->setWordWrap(true);
  cleaning_layout->addWidget(cleaning_notice_);
  cleaning_list_ = new QListWidget(cleaning);
  cleaning_list_->setObjectName(QStringLiteral("cleaningLitterList"));
  cleaning_list_->setWordWrap(true);
  cleaning_list_->setSpacing(5);
  cleaning_layout->addWidget(cleaning_list_, 1);
  columns->addWidget(splitter);
  columns->addWidget(cleaning);
  columns->setStretchFactor(0, 3);
  columns->setStretchFactor(1, 1);
  columns->setSizes({1000, 360});
  layout->addWidget(columns, 1);
  connect(fit_button_, &QPushButton::clicked, map_view_, &PendingLitterMapView::fitMap);
  for (auto* list : {list_, cleaning_list_})
    connect(list, &QListWidget::currentRowChanged, this, [this, list](int) {
      updateButtons();
      const int index = selectedPointIndex(list);
      if (index >= 0)
        map_view_->centerOn(map_view_->worldToScene(registry_.points()[index].position));
    });
  const auto remove_selected = [this]() {
    if (!registry_.removePoint(selectedPointIndex(list_))) return;
    redrawPoints();
    updateStatus();
  };
  connect(delete_button_, &QPushButton::clicked, this, remove_selected);
  auto* shortcut = new QShortcut(QKeySequence(Qt::Key_Delete), list_);
  shortcut->setContext(Qt::WidgetShortcut);
  connect(shortcut, &QShortcut::activated, this, remove_selected);
  connect(add_button_, &QPushButton::clicked, this, [this]() {
    const int index = selectedPointIndex(list_);
    if (index < 0) return;
    const QString label = registry_.points()[index].label;
    cleaning_labels_.insert(label);
    redrawPoints();
    for (int row = 0; row < cleaning_list_->count(); ++row)
      if (cleaning_list_->item(row)->data(Qt::UserRole).toString() == label)
        cleaning_list_->setCurrentRow(row);
    updateStatus();
  });
  connect(return_button_, &QPushButton::clicked, this, [this]() {
    const int index = selectedPointIndex(cleaning_list_);
    if (index < 0) return;
    const QString label = registry_.points()[index].label;
    cleaning_labels_.remove(label);
    redrawPoints();
    for (int row = 0; row < list_->count(); ++row)
      if (list_->item(row)->data(Qt::UserRole).toString() == label)
        list_->setCurrentRow(row);
    updateStatus();
  });
  cleaning_watcher_ = new QFutureWatcher<PendingLitterCleaningUiResult>(this);
  connect(clean_button_, &QPushButton::clicked,
          this, &PendingLitterPage::startCleaningRequest);
  connect(cancel_clean_button_, &QPushButton::clicked,
          this, &PendingLitterPage::cancelCleaningRequest);
  connect(cleaning_watcher_, &QFutureWatcher<PendingLitterCleaningUiResult>::finished,
          this, [this]() {
    const auto result = cleaning_watcher_->result();
    cleaning_request_busy_ = false;
    if (result.cancel_request)
    {
      if (result.transport_ok && result.accepted)
        cleaning_mission_requested_ = false;
    }
    else if (result.transport_ok && result.accepted)
    {
      cleaning_mission_requested_ = true;
    }
    cleaning_notice_->setText(
        (result.transport_ok && result.accepted ? QStringLiteral("已接受：")
                                                : QStringLiteral("未启动："))
        + result.message);
    updateButtons();
  });
  updateButtons();
}

void PendingLitterPage::startCleaningRequest()
{
  const int index = selectedPointIndex(cleaning_list_);
  if (index < 0 || cleaning_request_busy_ || cleaning_mission_requested_) return;
  const PendingLitterPoint point = registry_.points()[index];
  if (point.state != QStringLiteral("CONFIRMED") || point.object_id == 0 ||
      !std::isfinite(point.position.x()) || !std::isfinite(point.position.y()) ||
      !std::isfinite(point.last_seen) || point.last_seen <= 0.0)
  {
    cleaning_notice_->setText(QStringLiteral("未启动：所选点位尚未形成有效的确认身份与地图坐标。"));
    return;
  }
  cleaning_request_busy_ = true;
  cleaning_notice_->setText(QStringLiteral("正在提交所选点位，等待清扫后端确认……"));
  updateButtons();
  cleaning_watcher_->setFuture(QtConcurrent::run([point]() {
    PendingLitterCleaningUiResult result;
    autolabor_fod_msgs::StartPendingLitterCleaning service;
    service.request.object_id = point.object_id;
    service.request.target_position.x = point.position.x();
    service.request.target_position.y = point.position.y();
    service.request.target_position.z = 0.0;
    service.request.target_frame = "map";
    service.request.observed_at.fromSec(point.last_seen);
    service.request.label = point.label.toStdString();
    result.transport_ok = ros::service::call(kStartCleaningService, service);
    result.accepted = result.transport_ok && service.response.success;
    result.message = result.transport_ok
        ? QString::fromStdString(service.response.message)
        : QStringLiteral("无法连接待清扫垃圾后端服务");
    return result;
  }));
}

void PendingLitterPage::cancelCleaningRequest()
{
  if (cleaning_request_busy_ || !cleaning_mission_requested_) return;
  cleaning_request_busy_ = true;
  cleaning_notice_->setText(QStringLiteral("正在请求取消当前待清扫垃圾任务……"));
  updateButtons();
  cleaning_watcher_->setFuture(QtConcurrent::run([]() {
    PendingLitterCleaningUiResult result;
    result.cancel_request = true;
    std_srvs::Trigger service;
    result.transport_ok = ros::service::call(kCancelCleaningService, service);
    result.accepted = result.transport_ok && service.response.success;
    result.message = result.transport_ok
        ? QString::fromStdString(service.response.message)
        : QStringLiteral("无法连接取消服务");
    return result;
  }));
}

void PendingLitterPage::markCleaned(quint64 object_id)
{
  if (object_id == 0) return;
  cleaned_object_ids_.insert(object_id);
  for (int index = registry_.points().size() - 1; index >= 0; --index)
    if (registry_.points()[index].object_id == object_id)
    {
      cleaning_labels_.remove(registry_.points()[index].label);
      registry_.removePoint(index);
    }
  cleaning_mission_requested_ = false;
  cleaning_request_busy_ = false;
  cleaning_notice_->setText(QStringLiteral(
      "清扫完成：车头清扫轨迹已以前进方式在10cm内覆盖所选垃圾坐标。"));
  redrawPoints();
  updateStatus();
}

void PendingLitterPage::updateCleaningStatus(const QString& json_status)
{
  QJsonParseError error;
  const auto document = QJsonDocument::fromJson(json_status.toUtf8(), &error);
  if (error.error != QJsonParseError::NoError || !document.isObject()) return;
  const auto object = document.object();
  if (!object.value(QStringLiteral("active")).isBool() ||
      !object.value(QStringLiteral("state")).isString()) return;
  const QString state = object.value(QStringLiteral("state")).toString();
  const QString reason = object.value(QStringLiteral("reason")).toString();
  const bool active = object.value(QStringLiteral("active")).toBool(false);
  if (active)
  {
    cleaning_mission_requested_ = true;
    cleaning_notice_->setText(QStringLiteral("清扫任务 %1：%2").arg(state, reason));
  }
  else if (state == QStringLiteral("ABORT"))
  {
    cleaning_mission_requested_ = false;
    cleaning_notice_->setText(QStringLiteral("清扫已中止：%1").arg(reason));
  }
  else if (state == QStringLiteral("IDLE"))
  {
    cleaning_mission_requested_ = false;
  }
  else if (state == QStringLiteral("COMPLETE") && cleaning_mission_requested_)
  {
    bool valid_id = false;
    const auto id = object.value(QStringLiteral("last_completed_object_id_text"))
                        .toString().toULongLong(&valid_id);
    if (valid_id && id != 0)
      markCleaned(id);
  }
  updateButtons();
}

void PendingLitterPage::configure(const QString& root, const QString& source, bool persist)
{
  // Legacy arguments retained for launch compatibility. Points are exclusively
  // in memory: even an old persist=true setting cannot resurrect another run.
  Q_UNUSED(root);
  Q_UNUSED(persist);
  if (configured_ && source_ == source) return;
  configured_ = true;
  source_ = source;
  map_key_.clear();
  map_ready_ = false;
  registry_.reset();
  cleaning_labels_.clear();
  cleaned_object_ids_.clear();
  cleaning_mission_requested_ = false;
  redrawPoints();
}

void PendingLitterPage::setAvailable(bool available)
{
  available_ = available;
  if (!available)
  {
    if (registry_.suspend()) redrawPoints();
    current_position_valid_ = false;
  }
  updateStatus();
}

QString PendingLitterPage::mapKey(const nav_msgs::OccupancyGrid& map, const QString& source)
{
  // Message stamps and load times change on restart; geometry/content identify
  // the map. Include the map-set path/mode to isolate different map assets.
  QByteArray geometry;
  QDataStream stream(&geometry, QIODevice::WriteOnly);
  stream.setVersion(QDataStream::Qt_5_9);
  stream << source << QString::fromStdString(map.header.frame_id)
         << quint32(map.info.width) << quint32(map.info.height)
         << double(map.info.resolution) << map.info.origin.position.x
         << map.info.origin.position.y << map.info.origin.position.z
         << map.info.origin.orientation.x << map.info.origin.orientation.y
         << map.info.origin.orientation.z << map.info.origin.orientation.w;
  QCryptographicHash hash(QCryptographicHash::Sha256);
  hash.addData(geometry);
  hash.addData(reinterpret_cast<const char*>(map.data.data()), static_cast<int>(map.data.size()));
  return QString::fromLatin1(hash.result().toHex());
}

void PendingLitterPage::consume(const PendingLitterInput& input, const ros::Time& now,
                               const ros::WallTime& wall_now)
{
  if (input.reset)
  {
    registry_.suspend();
    redrawPoints();
    // Require this connection's latched map before accepting new coordinates.
    // Keep the displayed/persisted points until that map identity is known.
    map_ready_ = false;
    current_position_valid_ = false;
  }
  if (input.map && available_)
  {
    const auto& map = *input.map;
    const auto& q = map.info.origin.orientation;
    const double qnorm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (map.header.frame_id != "map" || map.info.width == 0 || map.info.height == 0 ||
        map.info.width > 16384 || map.info.height > 16384 || map.data.size() > 32 * 1024 * 1024 ||
        map.data.size() != static_cast<std::size_t>(map.info.width) * map.info.height ||
        !std::isfinite(map.info.resolution) || map.info.resolution <= 0.0 ||
        !std::isfinite(map.info.origin.position.x) || !std::isfinite(map.info.origin.position.y) ||
        !std::isfinite(qnorm) || std::abs(qnorm - 1.0) > 0.001 ||
        std::abs(q.x) > 1e-6 || std::abs(q.y) > 1e-6)
    {
      registry_.breakContinuity();
      map_ready_ = false;
      updateStatus();
      return;
    }
    const QString key = mapKey(*input.map, source_);
    if (key != map_key_)
    {
      map_changed_at_ = map_key_.isEmpty() ? ros::Time() : input.received_stamp;
      map_key_ = key;
      stream_key_.clear();
      registry_.reset();
      cleaning_labels_.clear();
      cleaned_object_ids_.clear();
      cleaning_mission_requested_ = false;
      map_view_->setMap(*input.map);
      map_ready_ = true;
      fit_button_->setEnabled(true);
      redrawPoints();
    }
    else
    {
      map_ready_ = true;
      fit_button_->setEnabled(true);
    }
  }
  if (!input.frame)
  {
    if (!last_frame_received_.isZero() && (wall_now-last_frame_received_).toSec() > .5)
    {
      if (registry_.suspend()) redrawPoints();
      current_position_valid_ = false;
      stability_status_ = "STALE";
    }
    updateStatus(); return;
  }
  const auto& frame = *input.frame;
  const double age = (now - frame.source_header.stamp).toSec();
  const double wall_age = (wall_now - input.received_at).toSec();
  if (!available_ || !map_ready_ || frame.backend_id != "detect_and_classify" ||
      frame.header.frame_id != "map" || frame.source_header.frame_id.empty() ||
      frame.source_header.stamp.isZero() || frame.source_header.stamp != frame.header.stamp ||
      frame.calibration_sha256.empty() || frame.positions.size() > 100 ||
      frame.odom_frame != "camera_init" || frame.odom_session_id.empty() ||
      age < -0.02 || age > 0.5 || wall_age < 0.0 || wall_age > 0.5 ||
      frame.source_header.stamp < map_changed_at_)
  {
    if (registry_.suspend()) redrawPoints();
    current_position_valid_ = false;
    stability_status_ = "INVALID_OR_STALE";
    updateStatus();
    return;
  }
  const QString stream = QString::fromStdString(frame.source_header.frame_id + ":" +
                                                 frame.calibration_sha256 + ":" + frame.odom_session_id);
  if (stream != stream_key_)
  {
    registry_.breakContinuity();
    stream_key_ = stream;
    cleaned_object_ids_.clear();
  }
  QVector<LitterObservation> observations;
  last_frame_received_ = input.received_at;
  stability_status_ = QString::fromStdString(frame.stability_status);
  for (const auto& position : frame.positions)
    if (!cleaned_object_ids_.contains(position.object_id) &&
        position.odom_position_valid &&
        std::isfinite(position.odom_position.x) && std::isfinite(position.odom_position.y) &&
        std::isfinite(position.odom_position.z) &&
        std::isfinite(position.depth_m) && position.depth_m > 0.0 && position.depth_sample_count > 0)
      observations.push_back({QPointF(position.position.x, position.position.y),
                              QString::fromStdString(position.material_class),
                              {{position.odom_position.x, position.odom_position.y, position.odom_position.z}},
                              position.object_id});
  current_position_valid_ = !observations.empty();
  std::array<double, 16> matrix;
  std::copy(frame.stable_map_from_odom.begin(), frame.stable_map_from_odom.end(), matrix.begin());
  if (registry_.observe(frame.source_header.stamp.toSec(), observations, stream,
                        matrix, frame.map_stable, frame.correction_generation))
  {
    redrawPoints();
  }
  updateStatus();
}

void PendingLitterPage::redrawPoints()
{
  const auto labelOf = [](QListWidget* list) {
    return list->currentItem() ? list->currentItem()->data(Qt::UserRole).toString() : QString();
  };
  const QString selected = labelOf(list_), cleaning_selected = labelOf(cleaning_list_);
  for (auto* list : {list_, cleaning_list_}) { list->blockSignals(true); list->clear(); }
  for (const auto& point : registry_.points())
  {
    auto* list = cleaning_labels_.contains(point.label) ? cleaning_list_ : list_;
    auto* item = new QListWidgetItem(QStringLiteral("待清扫垃圾%1  类别:%2  position:map (%3, %4)m · %5")
                       .arg(point.label, point.category)
                       .arg(point.position.x(), 0, 'f', 3).arg(point.position.y(), 0, 'f', 3)
                       .arg(point.state == "CONFIRMED" ? QStringLiteral("已确认") : QStringLiteral("待复核")), list);
    item->setData(Qt::UserRole, point.label);
    if (point.label == (list == list_ ? selected : cleaning_selected)) list->setCurrentItem(item);
  }
  for (auto* list : {list_, cleaning_list_}) list->blockSignals(false);
  updateButtons();
  map_view_->setPoints(registry_.points());
}

int PendingLitterPage::selectedPointIndex(QListWidget* list) const
{
  if (!list->currentItem()) return -1;
  const QString label = list->currentItem()->data(Qt::UserRole).toString();
  for (int i = 0; i < registry_.points().size(); ++i)
    if (registry_.points()[i].label == label) return i;
  return -1;
}

void PendingLitterPage::updateButtons()
{
  const bool pending_selected = selectedPointIndex(list_) >= 0;
  const int cleaning_index = selectedPointIndex(cleaning_list_);
  const bool cleaning_selected = cleaning_index >= 0;
  const bool cleaning_confirmed = cleaning_selected &&
      registry_.points()[cleaning_index].state == QStringLiteral("CONFIRMED") &&
      registry_.points()[cleaning_index].object_id != 0;
  delete_button_->setEnabled(pending_selected);
  add_button_->setEnabled(pending_selected && !cleaning_mission_requested_ &&
                          !cleaning_request_busy_);
  return_button_->setEnabled(cleaning_selected && !cleaning_mission_requested_ &&
                             !cleaning_request_busy_);
  clean_button_->setEnabled(cleaning_confirmed && !cleaning_mission_requested_ &&
                            !cleaning_request_busy_);
  cancel_clean_button_->setEnabled(cleaning_mission_requested_ &&
                                   !cleaning_request_busy_);
}

void PendingLitterPage::updateStatus()
{
  int confirmed = 0;
  for (const auto& point : registry_.points()) confirmed += point.state == "CONFIRMED";
  QString text = QStringLiteral("已确认 %1 处 · 待复核 %2 处 · 仅本次会话，重启Qt清空")
                     .arg(confirmed).arg(registry_.points().size()-confirmed);
  text += QStringLiteral(" · 待清扫列表 %1 处 / 清扫栏 %2 处")
              .arg(list_->count()).arg(cleaning_list_->count());
  if (!available_ || !map_ready_) text += QStringLiteral(" · 等待静态地图连接");
  else if (stability_status_ == "LOCALIZATION_OR_TF_UNAVAILABLE")
    text += QStringLiteral(" · 全局定位或TF未就绪");
  else if (stability_status_ == "STALE" || stability_status_ == "INVALID_OR_STALE")
    text += QStringLiteral(" · 等待新鲜有效观测");
  else if (stability_status_ != "STABLE") text += QStringLiteral(" · 地图修正稳定性校核中（约6秒）");
  else if (!current_position_valid_) text += QStringLiteral(" · 等待有效垃圾观测");
  else text += QStringLiteral(" · camera_init 聚类 / 5帧确认 / 地图修正已稳定");
  status_->setText(text);
}
}  // namespace autolabor_operator_gui
