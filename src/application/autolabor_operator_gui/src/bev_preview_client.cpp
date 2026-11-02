#include <autolabor_operator_gui/bev_preview_client.h>
#include <QBuffer>
#include <QCryptographicHash>
#include <QDateTime>
#include <QImageReader>
#include <QJsonArray>
#include <QJsonDocument>
#include <QNetworkProxy>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QRegularExpression>
#include <QStringList>
#include <QtEndian>
#include <cmath>
#include <memory>

namespace autolabor_operator_gui
{
namespace
{
constexpr int kMaxPacket = 16 + 128*1024 + 12*1024*1024;
bool number(const QJsonValue& value, double low, double high)
{
  return value.isDouble() && std::isfinite(value.toDouble()) && value.toDouble() >= low && value.toDouble() <= high;
}
}

bool decodeBevPreview(const QByteArray& packet, BevPreviewFrame* frame, QString* error)
{
  const auto reject = [error](const QString& why) { if (error) *error = why; return false; };
  const bool has_views = packet.left(4) == "BV02";
  if (packet.size() < 16 || packet.size() > kMaxPacket || (!has_views && packet.left(4) != "BV01"))
    return reject(QStringLiteral("BEV 报文无效"));
  const auto* p = reinterpret_cast<const uchar*>(packet.constData());
  const quint32 nj = qFromBigEndian<quint32>(p+4), nr = qFromBigEndian<quint32>(p+8), na = qFromBigEndian<quint32>(p+12);
  if (!nj || nj > 128*1024 || !nr || nr > 2*1024*1024 || !na || na > 2*1024*1024 ||
      16ull+nj+nr+na > quint64(packet.size()) ||
      (!has_views && 16ull+nj+nr+na != quint64(packet.size()))) return reject(QStringLiteral("BEV 图像长度不符"));
  QJsonParseError parse;
  const auto document = QJsonDocument::fromJson(packet.mid(16, nj), &parse);
  if (parse.error != QJsonParseError::NoError || !document.isObject()) return reject(QStringLiteral("BEV 元数据损坏"));
  const auto meta = document.object();
  const auto id = meta["source_id"].toObject();
  if (meta["schema"] != "conbine.bev.yolo.frame.v1" || id["session"].toString().isEmpty() ||
      !number(id["sequence"], 1, 1e12) || id != meta["image_source_id"].toObject() ||
      id != meta["result_source_id"].toObject()) return reject(QStringLiteral("BEV 图像与识别结果不是同一帧"));
  if (meta["width"].toInt() != 600 || meta["height"].toInt() != 600 ||
      (meta["backend"] != "CUDA" && meta["backend"] != "BPU") || meta["model_name"].toString().isEmpty() ||
      !QRegularExpression(QStringLiteral("^[0-9a-f]{64}$")).match(meta["model_sha256"].toString()).hasMatch() ||
      !number(meta["produced_epoch_s"], 1, 1e11) || !number(meta["inference_ms"], 0, 10000) ||
      !number(meta["inference_fps"], 0, 1000) || !meta["objects"].isArray())
    return reject(QStringLiteral("BEV 模型或图像信息无效"));
  for (const auto& item : meta["objects"].toArray())
  {
    const auto obj = item.toObject();
    const auto b = obj["bbox"].toArray();
    if (b.size() != 4 || !number(b[0], 0, 600) || !number(b[1], 0, 600) ||
        !number(b[2], 0, 600) || !number(b[3], 0, 600) || b[0].toDouble() >= b[2].toDouble() ||
        b[1].toDouble() >= b[3].toDouble() || !number(obj["confidence"], 0, 1) ||
        obj["class_name"].toString().isEmpty()) return reject(QStringLiteral("BEV 检测框无效"));
  }
  const auto decode = [](QByteArray jpeg, const QSize& size) {
    QBuffer buffer(&jpeg); buffer.open(QIODevice::ReadOnly);
    QImageReader reader(&buffer, "JPEG");
    return reader.size() == size ? reader.read() : QImage();
  };
  const auto raw = decode(packet.mid(16+nj, nr), QSize(600,600));
  const auto annotated = decode(packet.mid(16+nj+nr, na), QSize(600,600));
  if (raw.isNull() || annotated.isNull()) return reject(QStringLiteral("BEV 图像解码失败"));
  QMap<QString,QImage> camera_images;
  if (has_views)
  {
    const auto rows = meta["camera_previews"].toArray();
    const auto source = meta["source_metadata"].toObject();
    const auto sizes = source["camera_image_sizes"].toObject();
    const auto sources = source["sources"].toObject();
    if (rows.size() < 3 || rows.size() > 4 || rows.size() != sources.size())
      return reject(QStringLiteral("原图识别画面数量不符"));
    int offset = 16+nj+nr+na;
    for (const auto& value : rows)
    {
      const auto row = value.toObject();
      const auto name = row["camera"].toString();
      const int w = row["width"].toInt(), h = row["height"].toInt(), bytes = row["bytes"].toInt();
      const auto size = sizes[name].toArray();
      if (!QStringList{"front","left","right","rear"}.contains(name) || camera_images.contains(name) ||
          !sources.contains(name) || row["source_id"].toObject() != id || size.size() != 2 ||
          w < 2 || w > 1280 || h < 2 || h > 720 || w != size[0].toInt() || h != size[1].toInt() ||
          bytes <= 0 || bytes > 2*1024*1024 || offset+bytes > packet.size())
        return reject(QStringLiteral("原图与BEV不是同一组图像或尺寸不符"));
      const auto jpeg = packet.mid(offset,bytes);
      if (QString::fromLatin1(QCryptographicHash::hash(jpeg,QCryptographicHash::Sha256).toHex()) != row["sha256"].toString())
        return reject(QStringLiteral("原图识别画面校验失败"));
      const auto im = decode(jpeg,QSize(w,h));
      if (im.isNull()) return reject(QStringLiteral("原图识别画面解码失败"));
      camera_images.insert(name,im);
      offset += bytes;
    }
    if (offset != packet.size()) return reject(QStringLiteral("原图识别画面长度不符"));
  }
  frame->raw = raw; frame->annotated = annotated; frame->metadata = meta;
  frame->camera_images = camera_images;
  return true;
}

BevPreviewClient::BevPreviewClient(const QUrl& url, QObject* parent) : QObject(parent), url_(url)
{
  manager_.setProxy(QNetworkProxy::NoProxy);
  if (url.scheme() != "http" || url.host() != "127.0.0.1" || url.port() <= 0 ||
      !url.userInfo().isEmpty() || url.hasQuery() || url.hasFragment())
  { clear(QStringLiteral("需要本机 BEV 识别接口")); return; }
  url_.setPath(QStringLiteral("/vision_frame"));
  connect(&timer_, &QTimer::timeout, this, &BevPreviewClient::poll);
  timer_.start(200);
  clear(QStringLiteral("正在连接 BEV / 识别服务…"));
}

void BevPreviewClient::clear(const QString& message)
{
  frame_ = BevPreviewFrame(); status_ = message; emit changed();
}

void BevPreviewClient::poll()
{
  const double now = QDateTime::currentMSecsSinceEpoch()/1000.;
  if (!frame_.raw.isNull() && now-frame_.metadata["produced_epoch_s"].toDouble() > 2)
    clear(QStringLiteral("BEV 识别画面中断，等待新结果"));
  if (pending_) return;
  auto* reply = manager_.get(QNetworkRequest(url_)); pending_ = reply;
  auto bytes = std::make_shared<QByteArray>();
  connect(reply, &QNetworkReply::readyRead, this, [reply, bytes]() {
    if (bytes->size()+reply->bytesAvailable() > kMaxPacket) { reply->abort(); return; }
    bytes->append(reply->readAll());
  });
  QTimer::singleShot(1000, reply, [reply]() { if (!reply->isFinished()) reply->abort(); });
  connect(reply, &QNetworkReply::finished, this, [this, reply, bytes]() {
    pending_.clear();
    BevPreviewFrame candidate; QString error;
    const double now = QDateTime::currentMSecsSinceEpoch()/1000.;
    if (reply->error() != QNetworkReply::NoError ||
        reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt() != 200)
      clear(QStringLiteral("等待新鲜 BEV / 识别结果"));
    else if (!decodeBevPreview(*bytes, &candidate, &error)) clear(error);
    else if (now-candidate.metadata["produced_epoch_s"].toDouble() > 2 ||
             candidate.metadata["produced_epoch_s"].toDouble()-now > .1)
      clear(QStringLiteral("BEV 结果已过期"));
    else
    {
      const auto previous = frame_.metadata["source_id"].toObject();
      const auto next = candidate.metadata["source_id"].toObject();
      if (previous["session"] != next["session"] || next["sequence"].toDouble() > previous["sequence"].toDouble())
      {
        frame_ = candidate;
        const int cameras = candidate.metadata["source_metadata"].toObject()["sources"].toObject().size();
        status_ = QStringLiteral("%1路 BEV · %2 · %3 目标 · %4 ms")
            .arg(cameras)
            .arg(candidate.metadata["backend"].toString())
            .arg(candidate.metadata["objects"].toArray().size())
            .arg(candidate.metadata["inference_ms"].toDouble(), 0, 'f', 1);
        emit changed();
      }
    }
    reply->deleteLater();
  });
}
}
