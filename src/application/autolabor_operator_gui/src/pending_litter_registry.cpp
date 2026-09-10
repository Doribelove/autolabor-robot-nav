#include <autolabor_operator_gui/pending_litter_registry.h>

#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLockFile>
#include <QMap>
#include <QRegularExpression>
#include <QSaveFile>
#include <algorithm>
#include <cmath>
#include <limits>

namespace autolabor_operator_gui
{
namespace
{
bool finite(const QPointF& p)
{
  return std::isfinite(p.x()) && std::isfinite(p.y());
}
double distanceSquared(const QPointF& a, const QPointF& b)
{
  const QPointF d = a - b;
  return d.x() * d.x() + d.y() * d.y();
}
double median(QVector<double> values)
{
  std::sort(values.begin(), values.end());
  return values[values.size() / 2];
}
QByteArray digest(const QByteArray& bytes)
{
  return QCryptographicHash::hash(bytes, QCryptographicHash::Sha256);
}
bool fail(QString* error, const QString& message)
{
  if (error) *error = message;
  return false;
}
}  // namespace

bool PendingLitterRegistry::nearby(const QPointF& first, const QPointF& second)
{
  return distanceSquared(first, second) <= kMergeDistance * kMergeDistance + 1e-9;
}

QString PendingLitterRegistry::labelForIndex(int index)
{
  QString label;
  for (int value = index + 1; value > 0; value = (value - 1) / 26)
    label.prepend(QChar('a' + (value - 1) % 26));
  return label;
}

void PendingLitterRegistry::breakContinuity() { candidates_.clear(); }

bool PendingLitterRegistry::removePoint(int index)
{
  if (index < 0 || index >= points_.size()) return false;
  points_.removeAt(index);
  breakContinuity();
  return true;
}

bool PendingLitterRegistry::suspend()
{
  breakContinuity();
  bool changed = false;
  for (auto& point : points_)
    if (point.state != "UNCERTAIN") { point.state = "UNCERTAIN"; changed = true; }
  return changed;
}

void PendingLitterRegistry::reset(const QVector<PendingLitterPoint>& points)
{
  candidates_.clear();
  points_ = points;
  next_label_ = points.size();
  for (auto& point : points_) point.state = "UNCERTAIN";
  last_stamp_ = 0.0;
  session_.clear();
}

bool PendingLitterRegistry::observe(
    double stamp, const QVector<LitterObservation>& observations,
    const QString& session, const std::array<double, 16>& matrix,
    bool stable, quint64 generation)
{
  if (!std::isfinite(stamp) || stamp <= 0.0 || session.isEmpty())
  {
    breakContinuity();
    return false;
  }
  bool changed = false;
  if (session != session_)
  {
    session_ = session;
    last_stamp_ = 0;
    breakContinuity();
    for (auto& point : points_) { point.state = "UNCERTAIN"; changed = true; }
  }
  if (generation != generation_)
  {
    generation_ = generation;
    breakContinuity();
  }
  // Replayed / out-of-order frames never supply another vote.
  if (stamp <= last_stamp_) return false;
  if (last_stamp_ > 0.0 && stamp - last_stamp_ > kMaximumFrameGap)
    breakContinuity();
  last_stamp_ = stamp;

  auto xy = [](const std::array<double, 3>& p) { return QPointF(p[0], p[1]); };
  auto project = [&matrix](const std::array<double, 3>& p) {
    return QPointF(matrix[0]*p[0]+matrix[1]*p[1]+matrix[2]*p[2]+matrix[3],
                   matrix[4]*p[0]+matrix[5]*p[1]+matrix[6]*p[2]+matrix[7]);
  };
  auto summarize = [](PendingLitterPoint& point) {
    QVector<double> axes[3];
    QMap<QString, int> votes;
    for (const auto& sample : point.samples)
    {
      for (int i = 0; i < 3; ++i) axes[i].push_back(sample.odom[i]);
      if (!sample.category.trimmed().isEmpty()) ++votes[sample.category.trimmed()];
    }
    for (int i = 0; i < 3; ++i) point.odom[i] = median(axes[i]);
    QVector<double> residuals;
    for (const auto& sample : point.samples)
      residuals.push_back(std::hypot(sample.odom[0]-point.odom[0], sample.odom[1]-point.odom[1]));
    point.spread = median(residuals);
    int most = 0;
    for (auto it = votes.cbegin(); it != votes.cend(); ++it)
      if (it.value() > most) { most = it.value(); point.category = it.key(); }
  };
  // A rigid transform is required even though only map X/Y are displayed.
  if (stable)
  {
    for (double value : matrix) stable &= std::isfinite(value);
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
      {
        double dot = 0;
        for (int k = 0; k < 3; ++k) dot += matrix[4*k+i]*matrix[4*k+j];
        stable &= std::abs(dot-(i == j ? 1.0 : 0.0)) < 1e-5;
      }
    const double det = matrix[0]*(matrix[5]*matrix[10]-matrix[6]*matrix[9])
        - matrix[1]*(matrix[4]*matrix[10]-matrix[6]*matrix[8])
        + matrix[2]*(matrix[4]*matrix[9]-matrix[5]*matrix[8]);
    stable &= std::abs(det-1) < 1e-5 && std::abs(matrix[12]) < 1e-6 &&
              std::abs(matrix[13]) < 1e-6 && std::abs(matrix[14]) < 1e-6 &&
              std::abs(matrix[15]-1) < 1e-6;
  }
  QVector<LitterObservation> remaining;
  for (const auto& observation : observations)
  {
    if (std::isfinite(observation.odom[0]) && std::isfinite(observation.odom[1]) &&
        std::isfinite(observation.odom[2])) remaining.push_back(observation);
  }
  QVector<bool> used(remaining.size(), false), matched(points_.size(), false);
  for (int i = 0; i < remaining.size(); ++i)
  {
    const auto& observation = remaining[i];
    int best = -1;
    double score = std::numeric_limits<double>::infinity();
    for (int j = 0; j < points_.size(); ++j)
    {
      const auto& point = points_[j];
      // Old sessions only re-associate on a stable map; local coordinates from
      // another FAST-LIO origin never participate in distance calculations.
      const bool same_session = point.session == session;
      if (!same_session && !stable) continue;
      const auto a = same_session ? xy(point.anchor) : point.position;
      const auto b = same_session ? xy(observation.odom) : project(observation.odom);
      if (!nearby(a, b)) continue;
      const double distance = distanceSquared(a, b);
      if (distance < score) { best = j; score = distance; }
    }
    if (best < 0) continue;
    used[i] = true;  // Duplicate boxes never seed another candidate.
    if (matched[best]) continue;
    matched[best] = true;
    auto& point = points_[best];
    if (point.session != session || point.samples.empty())
    {
      // Reconfirm restored records with five fresh observations below.
      used[i] = false;
      continue;
    }
    point.samples.push_back(observation);
    if (point.samples.size() > 31) point.samples.removeFirst();
    summarize(point);
    point.last_seen = stamp;
    ++point.observations;
    point.object_id = observation.object_id;
  }
  QVector<Candidate> next;
  for (auto candidate : candidates_)
  {
    int best = -1;
    double best_distance = std::numeric_limits<double>::infinity();
    for (int i = 0; i < remaining.size(); ++i)
    {
      const double distance = distanceSquared(xy(candidate.anchor), xy(remaining[i].odom));
      if (!used[i] && nearby(xy(candidate.anchor), xy(remaining[i].odom)) &&
          distance < best_distance)
      {
        best = i;
        best_distance = distance;
      }
    }
    // A missed result frame breaks this candidate's consecutive streak.
    if (best < 0) continue;
    candidate.samples.push_back(remaining[best]);
    for (int i = 0; i < remaining.size(); ++i)
      if (nearby(xy(candidate.anchor), xy(remaining[i].odom))) used[i] = true;
    if (candidate.samples.size() > 31) candidate.samples.removeFirst();
    if (candidate.samples.size() < kConfirmationFrames || !stable)
    {
      next.push_back(candidate);
      continue;
    }
    PendingLitterPoint fresh;
    fresh.samples = candidate.samples;
    fresh.category = "未知";
    summarize(fresh);
    fresh.position = project(fresh.odom);
    fresh.anchor = candidate.anchor;
    fresh.session = session;
    fresh.object_id = candidate.samples.back().object_id;
    fresh.first_seen = candidate.first_seen;
    fresh.last_seen = stamp;
    fresh.observations = candidate.samples.size();
    int restored = -1;
    double nearest = std::numeric_limits<double>::infinity();
    for (int j = 0; j < points_.size(); ++j)
    {
      const auto& p = points_[j];
      const bool local = p.session == session;
      const QPointF a = local ? xy(p.anchor) : p.position;
      const QPointF b = local ? xy(fresh.odom) : fresh.position;
      const double distance = distanceSquared(a, b);
      if (nearby(a, b) && distance < nearest) { nearest = distance; restored = j; }
    }
    if (restored >= 0)
    {
      fresh.label = points_[restored].label;
      points_[restored] = fresh;
    }
    else { fresh.label = labelForIndex(next_label_++); points_.push_back(fresh); }
    changed = true;
  }
  for (int i = 0; i < remaining.size(); ++i)
  {
    if (used[i]) continue;
    bool registered = false;
    for (const auto& point : points_)
      registered |= point.session == session && !point.samples.empty() &&
                    nearby(xy(point.anchor), xy(remaining[i].odom));
    if (registered) continue;
    next.push_back({remaining[i].odom, {remaining[i]}, stamp});
    // Duplicate boxes in one frame are one observation, never five votes.
    for (int j = i; j < remaining.size(); ++j)
      if (nearby(xy(remaining[i].odom), xy(remaining[j].odom))) used[j] = true;
  }
  candidates_ = std::move(next);
  for (auto& point : points_)
  {
    if (point.session != session || point.samples.empty()) continue;
    const QString state = stable ? "CONFIRMED" : "UNCERTAIN";
    if (state != point.state) { point.state = state; changed = true; }
    if (stable)
    {
      const QPointF projected = project(point.odom);
      // Two-centimetre display/storage hysteresis, always computed from the
      // session anchor. It does not alter navigation TF or raw detections.
      if (distanceSquared(point.position, projected) > .02*.02)
      { point.position = projected; changed = true; }
    }
  }
  return changed;
}

bool PendingLitterStore::open(const QString& root, const QString& map_key,
                             QVector<PendingLitterPoint>* points, QString* error)
{
  loaded_ = false;
  path_.clear();
  points->clear();
  map_key_ = map_key;
  if (!QRegularExpression(QStringLiteral("^[a-f0-9]{64}$")).match(map_key).hasMatch() ||
      root.isEmpty())
    return fail(error, QStringLiteral("待清扫点保存目录或地图标识无效"));
  path_ = QDir(root).filePath(map_key + QStringLiteral(".json"));
  if (QFileInfo(path_).isSymLink())
    return fail(error, QStringLiteral("待清扫点文件不能是符号链接"));
  QFile file(path_);
  existed_ = file.exists();
  fingerprint_.clear();
  if (!existed_) { loaded_ = true; return true; }
  if (!file.open(QIODevice::ReadOnly) || file.size() > 8 * 1024 * 1024)
    return fail(error, QStringLiteral("无法读取待清扫点文件"));
  const QByteArray bytes = file.readAll();
  fingerprint_ = digest(bytes);
  const QJsonDocument document = QJsonDocument::fromJson(bytes);
  const QJsonObject object = document.object();
    if (!document.isObject() || (object.value("schema").toInt() != 1 && object.value("schema").toInt() != 2) ||
      object.value("map_key").toString() != map_key ||
      !object.value("points").isArray())
    return fail(error, QStringLiteral("待清扫点文件损坏或地图不匹配，原文件已保留"));
  QVector<PendingLitterPoint> parsed;
  for (const auto& entry : object.value("points").toArray())
  {
    const auto item = entry.toObject();
    const QPointF position(item.value("x").toDouble(), item.value("y").toDouble());
    if (!entry.isObject() || !item.value("x").isDouble() || !item.value("y").isDouble() ||
        !finite(position) || item.value("label").toString() !=
                                PendingLitterRegistry::labelForIndex(parsed.size()) ||
        !item.value("category").isString())
      return fail(error, QStringLiteral("待清扫点记录无效，原文件已保留"));
    PendingLitterPoint point{item.value("label").toString(), item.value("category").toString(), position};
    point.state = "UNCERTAIN";
    if (object.value("schema").toInt() == 2)
    {
      const auto odom = item.value("odom").toArray();
      const auto anchor = item.value("anchor").toArray();
      if (odom.size() != 3 || anchor.size() != 3 || !item.value("session").isString())
        return fail(error, QStringLiteral("待清扫点局部坐标记录损坏，原文件已保留"));
      for (int i = 0; i < 3; ++i)
      {
        if (!odom[i].isDouble() || !anchor[i].isDouble() ||
            !std::isfinite(odom[i].toDouble()) || !std::isfinite(anchor[i].toDouble()))
          return fail(error, QStringLiteral("待清扫点局部坐标无效，原文件已保留"));
        point.odom[i] = odom[i].toDouble();
        point.anchor[i] = anchor[i].toDouble();
      }
      point.session = item.value("session").toString();
      point.object_id = item.value("object_id").toString().toULongLong();
      point.first_seen = item.value("first_seen").toDouble();
      point.last_seen = item.value("last_seen").toDouble();
      point.observations = item.value("observations").toString().toULongLong();
      point.spread = item.value("spread_m").toDouble();
    }
    parsed.push_back(point);
  }
  *points = parsed;
  // Preserve exact pre-migration evidence before the first schema-2 save.
  if (object.value("schema").toInt() == 1)
  {
    const QString backup = path_ + ".schema1.bak";
    QFile saved(backup);
    if (saved.exists())
    {
      if (!saved.open(QIODevice::ReadOnly) || saved.readAll() != bytes)
        return fail(error, QStringLiteral("旧点库备份冲突，禁止覆盖原文件"));
    }
    else if (!QFile::copy(path_, backup))
      return fail(error, QStringLiteral("无法备份旧点库，禁止覆盖原文件"));
  }
  loaded_ = true;
  return true;
}

bool PendingLitterStore::save(const QVector<PendingLitterPoint>& points, QString* error)
{
  if (!loaded_) return fail(error, QStringLiteral("待清扫点文件未成功加载，禁止覆盖"));
  if (!QDir().mkpath(QFileInfo(path_).absolutePath()))
    return fail(error, QStringLiteral("无法创建待清扫点目录"));
  if (QFileInfo(path_).isSymLink() || QFileInfo(path_ + ".lock").isSymLink())
    return fail(error, QStringLiteral("待清扫点文件不能是符号链接"));
  QLockFile lock(path_ + ".lock");
  if (!lock.tryLock(0)) return fail(error, QStringLiteral("待清扫点文件正被另一界面使用"));
  QFile previous(path_);
  if (previous.exists() != existed_ ||
      (existed_ && (!previous.open(QIODevice::ReadOnly) ||
                   digest(previous.readAll()) != fingerprint_)))
    return fail(error, QStringLiteral("待清扫点文件已被修改，本次未覆盖"));
  QJsonArray array;
  for (const auto& point : points)
  {
    if (!finite(point.position)) return fail(error, QStringLiteral("待清扫点坐标无效"));
    array.append(QJsonObject{{"label", point.label}, {"category", point.category},
                             {"x", point.position.x()}, {"y", point.position.y()},
                             {"state", point.state}, {"session", point.session},
                             {"odom", QJsonArray{point.odom[0], point.odom[1], point.odom[2]}},
                             {"anchor", QJsonArray{point.anchor[0], point.anchor[1], point.anchor[2]}},
                             {"object_id", QString::number(point.object_id)},
                             {"first_seen", point.first_seen}, {"last_seen", point.last_seen},
                             {"observations", QString::number(point.observations)},
                             {"spread_m", point.spread}});
  }
  const QByteArray bytes = QJsonDocument(QJsonObject{
      {"schema", 2}, {"map_key", map_key_}, {"frame", "map"}, {"points", array}}).toJson();
  QSaveFile file(path_);
  if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() || !file.commit())
    return fail(error, QStringLiteral("保存待清扫点失败，本次点位仍保留在界面中"));
  fingerprint_ = digest(bytes);
  existed_ = true;
  return true;
}
}  // namespace autolabor_operator_gui
