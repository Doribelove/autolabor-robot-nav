#include <autolabor_operator_gui/surround_preview_client.h>

#include <QBuffer>
#include <QImageReader>
#include <QJsonArray>
#include <QJsonDocument>
#include <QNetworkProxy>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QPainter>
#include <QtEndian>
#include <cmath>
#include <memory>

namespace autolabor_operator_gui
{
namespace
{
constexpr int kMaxPacket = 12 + 128 * 1024 + 2 * 1024 * 1024;
const QString kDetector = QStringLiteral("f2f1f7e9753b53b0494495be44af92a0603b3ee6c71e69e0f58a0d50544a8cb8");
const QString kClassifier = QStringLiteral("d4cb80f5f7364d1156ea2275eba9e522991dec309f2c397e74649254fe36a89f");
const QStringList kMaterials = {QStringLiteral("metal"), QStringLiteral("plastic"),
    QStringLiteral("paper"), QStringLiteral("glass"), QStringLiteral("kitchen_waste")};
const QStringList kChinese = {QStringLiteral("金属"), QStringLiteral("塑料"),
    QStringLiteral("纸类"), QStringLiteral("玻璃"), QStringLiteral("厨余")};
bool finite(const QJsonValue& value, double low, double high)
{
  return value.isDouble() && std::isfinite(value.toDouble()) &&
         value.toDouble() >= low && value.toDouble() <= high;
}
bool explicitlyFalse(const QJsonObject& object, const char* key)
{
  return object[key].isBool() && !object[key].toBool();
}
}

bool decodeSurroundPreview(const QByteArray& packet, int expected_port,
                          SurroundPreviewFrame* frame, QString* error)
{
  const auto reject = [error](const QString& why) { *error = why; return false; };
  if (packet.size() < 12 || packet.size() > kMaxPacket || packet.left(4) != "SG01")
    return reject(QStringLiteral("无效的环视报文"));
  const auto* p = reinterpret_cast<const uchar*>(packet.constData());
  const quint32 nj = qFromBigEndian<quint32>(p + 4);
  const quint32 ni = qFromBigEndian<quint32>(p + 8);
  if (!nj || nj > 128*1024 || !ni || ni > 2*1024*1024 || 12ull+nj+ni != quint64(packet.size()))
    return reject(QStringLiteral("图像报文长度不符"));
  QJsonParseError parse_error;
  const auto document = QJsonDocument::fromJson(packet.mid(12, nj), &parse_error);
  if (parse_error.error != QJsonParseError::NoError || !document.isObject())
    return reject(QStringLiteral("识别元数据损坏"));
  const auto meta = document.object();
  const auto source = meta["source"].toObject();
  bool stamp_ok = false, hal_ok = false;
  source["raw_stamp"].toString().toULongLong(&stamp_ok);
  source["hal_received_ns"].toString().toULongLong(&hal_ok);
  if (expected_port < 0 || expected_port > 2 || source["port"].toInt(-1) != expected_port ||
      !meta["image_source"].isObject() || source != meta["image_source"].toObject() ||
      source["session"].toString().isEmpty() || source["session"].toString().size() > 128 ||
      source["frame_id"].toString() != QStringLiteral("surround_rx1_port%1_rectified_optical").arg(expected_port) ||
      !stamp_ok || !hal_ok || !finite(source["sequence"], 0, 4294967295.0) ||
      std::floor(source["sequence"].toDouble()) != source["sequence"].toDouble())
    return reject(QStringLiteral("相机端口或图像/结果身份不一致"));
  if (meta["schema"] != "j6m.surround.operator.frame.v1" ||
      meta["inference_device"] != "J6M_BPU" || meta["detector_sha256"] != kDetector ||
      meta["classifier_sha256"] != kClassifier || meta["width"].toInt() != 960 ||
      meta["height"].toInt() != 540 || meta["encoding"] != "jpeg_display_copy" ||
      !meta["gdc_applied"].isBool() || !meta["gdc_applied"].toBool() ||
      meta["lut_sha256"].toString().size() != 64 ||
      !explicitlyFalse(meta, "production_ready") || !explicitlyFalse(meta, "motion_eligible") ||
      !explicitlyFalse(meta, "source_clock_verified") || !explicitlyFalse(meta, "accuracy_verified"))
    return reject(QStringLiteral("GDC/BPU 模型或观察接口不匹配"));
  if (!finite(meta["read_age_ms"], 0, 500) || !finite(meta["read_to_result_ms"], 0, 500) ||
      !finite(meta["inference_fps"], 0, 120) || !meta["objects"].isArray() ||
      meta["objects"].toArray().size() > 300 ||
      !finite(meta["timing"].toObject()["processing_ms"], 0, 500))
    return reject(QStringLiteral("结果已过期或识别统计无效"));
  const auto objects = meta["objects"].toArray();
  for (const auto& value : objects)
  {
    const auto obj = value.toObject();
    const auto box = obj["bbox"].toArray();
    if (box.size() != 4 || !finite(box[0], 0, 960) || !finite(box[2], 0, 960) ||
        !finite(box[1], 0, 540) || !finite(box[3], 0, 540) ||
        box[0].toDouble() >= box[2].toDouble() || box[1].toDouble() >= box[3].toDouble() ||
        obj["class_id"].toInt(-1) != 0 || !finite(obj["detect_confidence"], 0, 1))
      return reject(QStringLiteral("无效的检测框"));
    if (obj["probabilities"].isNull())
    {
      if (obj["material"] != "unknown")
        return reject(QStringLiteral("未分类目标携带错误类别"));
      continue;
    }
    const auto probabilities = obj["probabilities"].toArray();
    double sum = 0, maximum = -1;
    int best = -1;
    if (probabilities.size() != 5 || !finite(obj["classify_confidence"], 0, 1))
      return reject(QStringLiteral("无效的材质概率"));
    for (int i=0; i<5; ++i)
    {
      if (!finite(probabilities[i], 0, 1)) return reject(QStringLiteral("无效的材质概率"));
      const double prob = probabilities[i].toDouble();
      sum += prob;
      if (prob > maximum) { maximum = prob; best = i; }
    }
    if (std::abs(sum-1) > .01 || obj["material"] != kMaterials[best] ||
        std::abs(obj["classify_confidence"].toDouble()-maximum) > .001)
      return reject(QStringLiteral("材质类别与概率不一致"));
  }
  QByteArray jpeg = packet.mid(12+nj, ni);
  QBuffer buffer(&jpeg);
  buffer.open(QIODevice::ReadOnly);
  QImageReader reader(&buffer, "JPEG");
  if (reader.size() != QSize(960, 540)) return reject(QStringLiteral("JPEG 尺寸不符"));
  QImage raw = reader.read();
  if (raw.isNull()) return reject(QStringLiteral("JPEG 解码失败"));
  frame->raw = raw;
  frame->annotated = raw.convertToFormat(QImage::Format_RGB32);
  frame->metadata = meta;
  QPainter painter(&frame->annotated);
  painter.setPen(QPen(QColor(30, 240, 170), 2));
  QFont font = painter.font(); font.setPixelSize(19); painter.setFont(font);
  for (const auto& value : objects)
  {
    const auto obj = value.toObject();
    const auto b = obj["bbox"].toArray();
    const QRectF box(b[0].toDouble(), b[1].toDouble(), b[2].toDouble()-b[0].toDouble(),
                     b[3].toDouble()-b[1].toDouble());
    painter.drawRect(box);
    const int material = kMaterials.indexOf(obj["material"].toString());
    QString text = QStringLiteral("%1  D:%2%")
        .arg(material >= 0 ? kChinese[material] : QStringLiteral("垃圾 / 待分类"))
        .arg(obj["detect_confidence"].toDouble()*100, 0, 'f', 1);
    if (material >= 0) text += QStringLiteral(" C:%1%").arg(obj["classify_confidence"].toDouble()*100, 0, 'f', 1);
    const int x = qBound(0, int(box.left()), 960-painter.fontMetrics().horizontalAdvance(text)-8);
    const int y = qBound(23, int(box.top())-6, 535);
    painter.fillRect(QRect(x, y-21, painter.fontMetrics().horizontalAdvance(text)+8, 25), QColor(0,0,0,190));
    painter.drawText(x+4, y, text);
  }
  return true;
}

SurroundPreviewClient::SurroundPreviewClient(const QUrl& url, QObject* parent)
    : QObject(parent), url_(url)
{
  // The launch script owns the SSH forward. Do not route camera data through proxies.
  manager_.setProxy(QNetworkProxy::NoProxy);
  if (url.scheme() != "http" || url.host() != "127.0.0.1" || url.port() <= 0 ||
      !url.userInfo().isEmpty() || url.hasQuery() || url.hasFragment())
  {
    clear(QStringLiteral("需要本机 SSH 预览入口"));
    return;
  }
  connect(&timer_, &QTimer::timeout, this, &SurroundPreviewClient::poll);
  timer_.start(100);
  clear(QStringLiteral("正在连接端口 0……"));
}

void SurroundPreviewClient::selectPort(int port)
{
  if (port < 0 || port > 2 || port == port_) return;
  port_ = port;
  ++generation_;  // Late replies for a previous selection are never displayed.
  auto old = pending_;
  pending_.clear();
  if (old) old->abort();
  clear(QStringLiteral("正在切换至端口 %1……").arg(port_));
  if (timer_.isActive()) poll();
}

void SurroundPreviewClient::clear(const QString& message)
{
  frame_ = SurroundPreviewFrame();
  received_.invalidate();
  status_ = message;
  emit changed();
}

bool SurroundPreviewClient::acceptSource(const QJsonObject& source)
{
  const int p = source["port"].toInt(-1);
  if (p != port_) return false;
  const auto session = source["session"].toString();
  if (retired_[p].contains(session)) return false;
  const auto old = sources_[p];
  if (!old.isEmpty() && old["session"] == source["session"])
  {
    if (source == old) return true; // HTTP age still refers to the original SDK read.
    if (source["sequence"].toDouble() <= old["sequence"].toDouble() ||
        source["raw_stamp"].toString().toULongLong() <= old["raw_stamp"].toString().toULongLong())
      return false;
  }
  else if (!old.isEmpty())
  {
    if (retired_[p].size() >= 1024) return false;
    retired_[p].insert(old["session"].toString());
  }
  sources_[p] = source;
  return true;
}

void SurroundPreviewClient::poll()
{
  if (received_.isValid() && received_.elapsed()+read_age_ms_ > 500)
    clear(QStringLiteral("端口 %1 画面已中断，等待新结果").arg(port_));
  if (pending_) return;
  auto url = url_;
  url.setPath(QStringLiteral("/frame/%1").arg(port_));
  const int port = port_;
  const unsigned generation = generation_;
  auto* reply = manager_.get(QNetworkRequest(url));
  pending_ = reply;
  auto started = std::make_shared<QElapsedTimer>(); started->start();
  auto bytes = std::make_shared<QByteArray>();
  connect(reply, &QNetworkReply::readyRead, this, [reply, bytes]() {
    if (bytes->size()+reply->bytesAvailable() > kMaxPacket) { reply->abort(); return; }
    bytes->append(reply->readAll());
  });
  QTimer::singleShot(500, reply, [reply]() { if (!reply->isFinished()) reply->abort(); });
  connect(reply, &QNetworkReply::finished, this, [this, reply, port, generation, started, bytes]() {
    const auto release = [reply]() { reply->deleteLater(); };
    if (generation != generation_ || port != port_) { release(); return; }
    pending_.clear();
    if (reply->error() != QNetworkReply::NoError || reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt() != 200)
    {
      clear(QStringLiteral("端口 %1 暂无新鲜识别画面").arg(port_));
      release(); return;
    }
    SurroundPreviewFrame candidate;
    QString error;
    if (!decodeSurroundPreview(*bytes, port, &candidate, &error) ||
        candidate.metadata["read_age_ms"].toDouble()+started->elapsed() > 500 ||
        !acceptSource(candidate.metadata["source"].toObject()))
    {
      clear(error.isEmpty() ? QStringLiteral("端口或帧时序不匹配 / 结果过期") : error);
      release(); return;
    }
    read_age_ms_ = candidate.metadata["read_age_ms"].toDouble()+started->elapsed();
    received_.start();
    frame_ = candidate;
    status_ = QStringLiteral("端口 %1 · GDC + J6M BPU · 帧 %2 · %3 fps")
        .arg(port_).arg(candidate.metadata["source"].toObject()["sequence"].toDouble(), 0, 'f', 0)
        .arg(candidate.metadata["inference_fps"].toDouble(), 0, 'f', 1);
    emit changed();
    release();
  });
}
}  // namespace autolabor_operator_gui
