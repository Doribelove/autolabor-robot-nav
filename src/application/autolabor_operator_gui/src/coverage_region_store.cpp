#include <autolabor_operator_gui/coverage_region_store.h>

#include <QDir>
#include <QCryptographicHash>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLockFile>
#include <QRegularExpression>
#include <QSaveFile>
#include <QStringList>
#include <QUuid>

#include <algorithm>
#include <cmath>
#include <limits>

namespace autolabor_operator_gui
{
namespace
{
constexpr double kPointEpsilon = 1.0e-6;
constexpr double kMinimumAreaM2 = 0.01;
constexpr double kMaximumAreaM2 = 1.0e8;
constexpr double kMaximumCoordinateMagnitude = 1.0e6;
constexpr qint64 kMaximumStoreBytes = 4 * 1024 * 1024;
constexpr int kMaximumPathCharacters = 4096;

bool containsUnsafeControlCharacter(const QString& value)
{
  for (const QChar character : value)
  {
    if (character.isNull() || character.category() == QChar::Other_Control ||
        character.category() == QChar::Separator_Line ||
        character.category() == QChar::Separator_Paragraph)
      return true;
  }
  return false;
}

QString canonicalDirectoryPath(const QString& path)
{
  const QFileInfo info(QDir::cleanPath(path.trimmed()));
  if (!info.exists() || !info.isDir())
    return QString();
  return info.canonicalFilePath();
}

bool prepareStoreDirectory(const QString& root, const QString& directory_path,
                           QString* error)
{
  if (!QDir().mkpath(root))
  {
    if (error)
      *error = QStringLiteral("无法创建区域库根目录：%1").arg(root);
    return false;
  }
  const QString canonical_root = QFileInfo(root).canonicalFilePath();
  if (canonical_root.isEmpty() || canonical_root == QDir::rootPath())
  {
    if (error)
      *error = QStringLiteral("区域库根目录无法安全解析或指向文件系统根目录");
    return false;
  }
  const QString relative_path = QDir(root).relativeFilePath(directory_path);
  if (relative_path == QStringLiteral("..") ||
      relative_path.startsWith(QStringLiteral("../")) ||
      QDir::isAbsolutePath(relative_path))
  {
    if (error)
      *error = QStringLiteral("区域库目录越出了配置根目录");
    return false;
  }
  QString current_path = root;
  const QStringList components =
      relative_path.split(QLatin1Char('/'), QString::SkipEmptyParts);
  for (const QString& component : components)
  {
    if (component == QStringLiteral("."))
      continue;
    const QString child_path = QDir(current_path).filePath(component);
    const QFileInfo child_info(child_path);
    if (child_info.isSymLink())
    {
      if (error)
        *error = QStringLiteral("区域库子目录不能是符号链接：%1").arg(child_path);
      return false;
    }
    if (child_info.exists() && !child_info.isDir())
    {
      if (error)
        *error = QStringLiteral("区域库路径组件不是目录：%1").arg(child_path);
      return false;
    }
    if (!child_info.exists() && !QDir(current_path).mkdir(component))
    {
      if (error)
        *error = QStringLiteral("无法创建区域库目录：%1").arg(child_path);
      return false;
    }
    current_path = child_path;
  }
  const QString canonical_directory =
      QFileInfo(current_path).canonicalFilePath();
  QString root_prefix = canonical_root;
  if (!root_prefix.endsWith(QDir::separator()))
    root_prefix.append(QDir::separator());
  if (!canonical_directory.startsWith(root_prefix))
  {
    if (error)
      *error = QStringLiteral("区域库目录通过符号链接越出了配置根目录");
    return false;
  }
  return true;
}

bool rejectSymlinkFile(const QString& path, QString* error)
{
  if (!QFileInfo(path).isSymLink())
    return true;
  if (error)
    *error = QStringLiteral("区域库文件或锁文件不能是符号链接：%1").arg(path);
  return false;
}

double cross(const QPointF& first, const QPointF& second, const QPointF& third)
{
  return (second.x() - first.x()) * (third.y() - first.y()) -
         (second.y() - first.y()) * (third.x() - first.x());
}

bool almostEqual(double first, double second)
{
  return std::abs(first - second) <= kPointEpsilon;
}

bool samePoint(const QPointF& first, const QPointF& second)
{
  return almostEqual(first.x(), second.x()) &&
         almostEqual(first.y(), second.y());
}

bool onSegment(const QPointF& first, const QPointF& second,
               const QPointF& point)
{
  return std::abs(cross(first, second, point)) <= kPointEpsilon &&
         point.x() >= std::min(first.x(), second.x()) - kPointEpsilon &&
         point.x() <= std::max(first.x(), second.x()) + kPointEpsilon &&
         point.y() >= std::min(first.y(), second.y()) - kPointEpsilon &&
         point.y() <= std::max(first.y(), second.y()) + kPointEpsilon;
}

int orientation(double value)
{
  if (value > kPointEpsilon)
    return 1;
  if (value < -kPointEpsilon)
    return -1;
  return 0;
}

bool segmentsIntersect(const QPointF& first_start, const QPointF& first_end,
                       const QPointF& second_start, const QPointF& second_end)
{
  const int first_orientation =
      orientation(cross(first_start, first_end, second_start));
  const int second_orientation =
      orientation(cross(first_start, first_end, second_end));
  const int third_orientation =
      orientation(cross(second_start, second_end, first_start));
  const int fourth_orientation =
      orientation(cross(second_start, second_end, first_end));
  if (first_orientation != second_orientation &&
      third_orientation != fourth_orientation)
    return true;
  return (first_orientation == 0 && onSegment(first_start, first_end, second_start)) ||
         (second_orientation == 0 && onSegment(first_start, first_end, second_end)) ||
         (third_orientation == 0 && onSegment(second_start, second_end, first_start)) ||
         (fourth_orientation == 0 && onSegment(second_start, second_end, first_end));
}

QString normalizedUuid(const QString& id)
{
  const QUuid uuid(id);
  return uuid.isNull() ? QString() : uuid.toString(QUuid::WithoutBraces);
}

bool validTimestamp(const QDateTime& timestamp)
{
  return timestamp.isValid() && timestamp.timeSpec() == Qt::UTC;
}

QJsonObject recordToJson(const CoverageRegionRecord& record)
{
  QJsonArray polygon;
  for (const QPointF& point : record.polygon)
  {
    QJsonObject vertex;
    vertex.insert(QStringLiteral("x"), point.x());
    vertex.insert(QStringLiteral("y"), point.y());
    polygon.append(vertex);
  }
  QJsonObject object;
  object.insert(QStringLiteral("id"), record.id);
  object.insert(QStringLiteral("name"), record.name);
  object.insert(QStringLiteral("map_digest"), record.map_digest);
  object.insert(QStringLiteral("map_source"), record.map_source);
  object.insert(QStringLiteral("source_mode"), record.source_mode);
  object.insert(QStringLiteral("polygon"), polygon);
  object.insert(QStringLiteral("revision"), QString::number(record.revision));
  object.insert(QStringLiteral("created_at"),
                record.created_at.toUTC().toString(Qt::ISODateWithMs));
  object.insert(QStringLiteral("updated_at"),
                record.updated_at.toUTC().toString(Qt::ISODateWithMs));
  return object;
}

bool parseRecord(const QJsonValue& value, CoverageRegionRecord* record,
                 QString* error)
{
  if (!record || !value.isObject())
  {
    if (error)
      *error = QStringLiteral("区域记录不是 JSON 对象");
    return false;
  }
  const QJsonObject object = value.toObject();
  CoverageRegionRecord parsed;
  parsed.id = object.value(QStringLiteral("id")).toString();
  parsed.name = object.value(QStringLiteral("name")).toString();
  parsed.map_digest = object.value(QStringLiteral("map_digest")).toString();
  parsed.map_source = object.value(QStringLiteral("map_source")).toString();
  parsed.source_mode = object.value(QStringLiteral("source_mode")).toString();
  bool revision_ok = false;
  parsed.revision = object.value(QStringLiteral("revision"))
                        .toString()
                        .toULongLong(&revision_ok);
  parsed.created_at = QDateTime::fromString(
      object.value(QStringLiteral("created_at")).toString(), Qt::ISODateWithMs);
  parsed.updated_at = QDateTime::fromString(
      object.value(QStringLiteral("updated_at")).toString(), Qt::ISODateWithMs);
  if (!revision_ok || parsed.revision == 0)
  {
    if (error)
      *error = QStringLiteral("区域 revision 无效");
    return false;
  }
  const QJsonValue polygon_value = object.value(QStringLiteral("polygon"));
  if (!polygon_value.isArray())
  {
    if (error)
      *error = QStringLiteral("区域 polygon 不是数组");
    return false;
  }
  for (const QJsonValue& vertex_value : polygon_value.toArray())
  {
    if (!vertex_value.isObject())
    {
      if (error)
        *error = QStringLiteral("区域顶点不是 JSON 对象");
      return false;
    }
    const QJsonObject vertex = vertex_value.toObject();
    const QJsonValue x = vertex.value(QStringLiteral("x"));
    const QJsonValue y = vertex.value(QStringLiteral("y"));
    if (!x.isDouble() || !y.isDouble())
    {
      if (error)
        *error = QStringLiteral("区域顶点坐标不是数值");
      return false;
    }
    parsed.polygon.push_back(QPointF(x.toDouble(), y.toDouble()));
  }
  *record = parsed;
  return true;
}

}  // namespace

CoverageRegionStore::CoverageRegionStore(const QString& root)
{
  setRoot(root);
}

void CoverageRegionStore::setRoot(const QString& root)
{
  QString normalized_root = root.trimmed();
  if (!normalized_root.isEmpty())
  {
    normalized_root = QDir::cleanPath(normalized_root);
    const QString canonical_root = canonicalDirectoryPath(normalized_root);
    if (!canonical_root.isEmpty())
      normalized_root = canonical_root;
  }
  if (root_ == normalized_root)
    return;
  root_ = normalized_root;
  regions_.clear();
  loaded_ = false;
  loaded_store_existed_ = false;
  loaded_store_fingerprint_.clear();
}

void CoverageRegionStore::setLegacyRoot(const QString& legacy_root)
{
  QString normalized_root = legacy_root.trimmed();
  if (!normalized_root.isEmpty())
  {
    normalized_root = QDir::cleanPath(normalized_root);
    const QString canonical_root = canonicalDirectoryPath(normalized_root);
    if (!canonical_root.isEmpty())
      normalized_root = canonical_root;
  }
  if (legacy_root_ == normalized_root)
    return;
  legacy_root_ = normalized_root;
  regions_.clear();
  loaded_ = false;
  loaded_store_existed_ = false;
  loaded_store_fingerprint_.clear();
}

bool CoverageRegionStore::setMapContext(const QString& map_digest,
                                        const QString& map_source,
                                        const QString& source_mode,
                                        QString* error)
{
  const QString normalized_digest = map_digest.trimmed().toLower();
  QString normalized_source = map_source.trimmed();
  const QString normalized_mode = source_mode.trimmed();
  const QRegularExpression digest_pattern(QStringLiteral("^[0-9a-f]{64}$"));
  const QRegularExpression mode_pattern(QStringLiteral("^[A-Za-z0-9_.-]{1,64}$"));
  if (!digest_pattern.match(normalized_digest).hasMatch())
  {
    setError(error, QStringLiteral("地图摘要必须是 64 位 SHA-256 十六进制字符串"));
    return false;
  }
  if (normalized_source.isEmpty())
  {
    setError(error, QStringLiteral("静态地图来源为空"));
    return false;
  }
  if (normalized_source.size() > kMaximumPathCharacters ||
      containsUnsafeControlCharacter(normalized_source))
  {
    setError(error, QStringLiteral("静态地图来源过长或包含控制字符"));
    return false;
  }
  normalized_source = canonicalDirectoryPath(normalized_source);
  if (normalized_source.isEmpty())
  {
    setError(error, QStringLiteral("静态地图目录不存在或无法安全解析"));
    return false;
  }
  if (!mode_pattern.match(normalized_mode).hasMatch() ||
      normalized_mode == QStringLiteral(".") ||
      normalized_mode == QStringLiteral(".."))
  {
    setError(error,
             QStringLiteral("地图来源模式只能包含字母、数字、点、下划线或连字符，"
                            "且不能为 . 或 .."));
    return false;
  }
  if (map_digest_ == normalized_digest && map_source_ == normalized_source &&
      source_mode_ == normalized_mode)
    return true;
  map_digest_ = normalized_digest;
  map_source_ = normalized_source;
  source_mode_ = normalized_mode;
  regions_.clear();
  loaded_ = false;
  loaded_store_existed_ = false;
  loaded_store_fingerprint_.clear();
  return true;
}

bool CoverageRegionStore::load(QString* error)
{
  if (!validateContext(error))
    return false;
  const QString path = filePath();
  const QString directory_path = QFileInfo(path).absolutePath();
  if (!prepareStoreDirectory(root_, directory_path, error) ||
      !rejectSymlinkFile(path, error) ||
      !rejectSymlinkFile(path + QStringLiteral(".lock"), error))
    return false;
  {
    QLockFile lock(path + QStringLiteral(".lock"));
    lock.setStaleLockTime(30000);
    if (!lock.tryLock(2000))
    {
      setError(error, QStringLiteral("区域库正被另一个进程使用"));
      return false;
    }
    QFile file(path);
    if (file.exists())
    {
      if (file.size() < 0 || file.size() > kMaximumStoreBytes)
      {
        setError(error, QStringLiteral("区域库文件大小异常"));
        return false;
      }
      if (!file.open(QIODevice::ReadOnly))
      {
        setError(error,
                 QStringLiteral("无法读取区域库：%1").arg(file.errorString()));
        return false;
      }
      const QByteArray payload = file.readAll();
      QVector<CoverageRegionRecord> parsed_regions;
      if (!parseStorePayload(payload, false, &parsed_regions, error))
        return false;
      regions_ = parsed_regions;
      loaded_ = true;
      loaded_store_existed_ = true;
      loaded_store_fingerprint_ =
          QCryptographicHash::hash(payload, QCryptographicHash::Sha256);
      return true;
    }
  }

  QVector<CoverageRegionRecord> migrated_regions;
  const QString legacy_path = legacyFilePath();
  if (!legacy_path.isEmpty() && QFileInfo(legacy_path).exists())
  {
    const QString canonical_legacy_root = canonicalDirectoryPath(legacy_root_);
    if (canonical_legacy_root.isEmpty() ||
        canonical_legacy_root == QDir::rootPath() ||
        !QDir::isAbsolutePath(legacy_root_) ||
        !prepareStoreDirectory(legacy_root_,
                               QFileInfo(legacy_path).absolutePath(), error) ||
        !rejectSymlinkFile(legacy_path, error) ||
        !rejectSymlinkFile(legacy_path + QStringLiteral(".lock"), error))
    {
      if (error && error->isEmpty())
        *error = QStringLiteral("旧区域库根目录无法安全解析");
      return false;
    }
    QLockFile legacy_lock(legacy_path + QStringLiteral(".lock"));
    legacy_lock.setStaleLockTime(30000);
    if (!legacy_lock.tryLock(2000))
    {
      setError(error, QStringLiteral("旧区域库正被另一个进程使用"));
      return false;
    }
    QFile legacy_file(legacy_path);
    if (legacy_file.size() < 0 || legacy_file.size() > kMaximumStoreBytes ||
        !legacy_file.open(QIODevice::ReadOnly))
    {
      setError(error, QStringLiteral("无法安全读取旧区域库"));
      return false;
    }
    const QByteArray payload = legacy_file.readAll();
    QJsonParseError parse_error;
    const QJsonDocument document = QJsonDocument::fromJson(payload, &parse_error);
    if (parse_error.error != QJsonParseError::NoError || !document.isObject())
    {
      setError(error,
               QStringLiteral("旧区域库 JSON 损坏：%1")
                   .arg(parse_error.errorString()));
      return false;
    }
    const QString legacy_source = canonicalDirectoryPath(
        document.object().value(QStringLiteral("map_source")).toString());
    // A digest-only legacy path may be shared by two distinct map-set
    // directories.  Migrate only when its recorded source resolves exactly to
    // the current canonical map-set; otherwise treat this map as having no
    // saved regions.
    if (legacy_source == map_source_ &&
        !parseStorePayload(payload, true, &migrated_regions, error))
      return false;
    if (legacy_source != map_source_)
      migrated_regions.clear();
  }

  regions_.clear();
  loaded_ = true;
  loaded_store_existed_ = false;
  loaded_store_fingerprint_.clear();
  if (!migrated_regions.isEmpty())
  {
    if (!saveRecords(migrated_regions, error))
    {
      loaded_ = false;
      return false;
    }
    regions_ = migrated_regions;
  }
  return true;
}

QString CoverageRegionStore::root() const
{
  return root_;
}

QString CoverageRegionStore::legacyRoot() const
{
  return legacy_root_;
}

QString CoverageRegionStore::mapDigest() const
{
  return map_digest_;
}

QString CoverageRegionStore::mapSource() const
{
  return map_source_;
}

QString CoverageRegionStore::sourceMode() const
{
  return source_mode_;
}

QString CoverageRegionStore::filePath() const
{
  return QDir(root_).filePath(
      QStringLiteral("coverage_regions/%1/regions.json").arg(source_mode_));
}

QString CoverageRegionStore::legacyFilePath() const
{
  if (legacy_root_.isEmpty())
    return QString();
  return QDir(legacy_root_).filePath(
      QStringLiteral("v1/%1/%2/regions.json").arg(map_digest_, source_mode_));
}

bool CoverageRegionStore::isLoaded() const
{
  return loaded_;
}

QVector<CoverageRegionRecord> CoverageRegionStore::regions() const
{
  return regions_;
}

bool CoverageRegionStore::findById(const QString& id,
                                   CoverageRegionRecord* record) const
{
  for (const CoverageRegionRecord& candidate : regions_)
  {
    if (candidate.id == id)
    {
      if (record)
        *record = candidate;
      return true;
    }
  }
  return false;
}

bool CoverageRegionStore::containsName(const QString& name,
                                       const QString& except_id) const
{
  const QString normalized = name.trimmed();
  for (const CoverageRegionRecord& record : regions_)
  {
    if (record.id != except_id &&
        record.name.compare(normalized, Qt::CaseInsensitive) == 0)
      return true;
  }
  return false;
}

bool CoverageRegionStore::addRegion(const QString& name,
                                    const QVector<QPointF>& polygon,
                                    CoverageRegionRecord* created,
                                    QString* error)
{
  if (!loaded_)
  {
    setError(error, QStringLiteral("区域库尚未成功加载；不会覆盖未知或损坏的文件"));
    return false;
  }
  const QString normalized_name = name.trimmed();
  if (!validateName(normalized_name, error) || !validatePolygon(polygon, error))
    return false;
  if (regions_.size() >= kMaximumRegionCount)
  {
    setError(error, QStringLiteral("区域记录数量已达到上限"));
    return false;
  }
  if (containsName(normalized_name))
  {
    setError(error, QStringLiteral("已存在同名清扫区域：%1").arg(normalized_name));
    return false;
  }
  CoverageRegionRecord record;
  record.id = QUuid::createUuid().toString(QUuid::WithoutBraces);
  record.name = normalized_name;
  record.map_digest = map_digest_;
  record.map_source = map_source_;
  record.source_mode = source_mode_;
  record.polygon = polygon;
  record.revision = 1;
  record.created_at = QDateTime::currentDateTimeUtc();
  record.updated_at = record.created_at;
  QVector<CoverageRegionRecord> updated = regions_;
  updated.push_back(record);
  if (!saveRecords(updated, error))
    return false;
  regions_ = updated;
  if (created)
    *created = record;
  return true;
}

bool CoverageRegionStore::removeRegion(const QString& id, QString* error)
{
  if (!loaded_)
  {
    setError(error, QStringLiteral("区域库尚未成功加载；不会覆盖未知或损坏的文件"));
    return false;
  }
  QVector<CoverageRegionRecord> updated = regions_;
  const auto found = std::find_if(updated.begin(), updated.end(),
                                  [&id](const CoverageRegionRecord& record) {
                                    return record.id == id;
                                  });
  if (found == updated.end())
  {
    setError(error, QStringLiteral("要删除的区域记录已不存在"));
    return false;
  }
  updated.erase(found);
  if (!saveRecords(updated, error))
    return false;
  regions_ = updated;
  return true;
}

bool CoverageRegionStore::parseStorePayload(
    const QByteArray& payload, bool require_source_match,
    QVector<CoverageRegionRecord>* records, QString* error) const
{
  if (!records)
  {
    setError(error, QStringLiteral("区域库解析输出为空"));
    return false;
  }
  QJsonParseError parse_error;
  const QJsonDocument document = QJsonDocument::fromJson(payload, &parse_error);
  if (parse_error.error != QJsonParseError::NoError || !document.isObject())
  {
    setError(error,
             QStringLiteral("区域库 JSON 损坏：%1").arg(parse_error.errorString()));
    return false;
  }
  const QJsonObject root = document.object();
  if (root.value(QStringLiteral("schema_version")).toInt(-1) != kSchemaVersion)
  {
    setError(error, QStringLiteral("区域库 schema 版本不受支持"));
    return false;
  }
  if (root.value(QStringLiteral("map_digest")).toString() != map_digest_ ||
      root.value(QStringLiteral("source_mode")).toString() != source_mode_)
  {
    setError(error, QStringLiteral("区域库地图身份与当前静态地图不一致"));
    return false;
  }
  const QString stored_source =
      root.value(QStringLiteral("map_source")).toString();
  if (stored_source.isEmpty() ||
      stored_source.size() > kMaximumPathCharacters ||
      containsUnsafeControlCharacter(stored_source))
  {
    setError(error, QStringLiteral("区域库记录的地图目录无效"));
    return false;
  }
  if (require_source_match &&
      canonicalDirectoryPath(stored_source) != map_source_)
  {
    setError(error, QStringLiteral("旧区域库不属于当前 map-set 目录"));
    return false;
  }
  const QJsonValue regions_value = root.value(QStringLiteral("regions"));
  if (!regions_value.isArray() ||
      regions_value.toArray().size() > kMaximumRegionCount)
  {
    setError(error, QStringLiteral("区域记录数组无效或数量过多"));
    return false;
  }
  QVector<CoverageRegionRecord> parsed_regions;
  for (const QJsonValue& value : regions_value.toArray())
  {
    CoverageRegionRecord record;
    QString record_error;
    if (!parseRecord(value, &record, &record_error) ||
        !validateRecord(record, &record_error))
    {
      setError(error, QStringLiteral("区域库记录无效：%1").arg(record_error));
      return false;
    }
    if (require_source_match &&
        canonicalDirectoryPath(record.map_source) != map_source_)
    {
      setError(error, QStringLiteral("旧区域记录不属于当前 map-set 目录"));
      return false;
    }
    // The containing map-set directory is the persistent identity boundary.
    // Rewrite provenance to its current canonical path so a harmless symlink
    // alias (for example map_sets/latest) does not leak into new writes.
    record.map_source = map_source_;
    for (const CoverageRegionRecord& existing : parsed_regions)
    {
      if (existing.id == record.id)
      {
        setError(error, QStringLiteral("区域库包含重复 UUID"));
        return false;
      }
      if (existing.name.compare(record.name, Qt::CaseInsensitive) == 0)
      {
        setError(error,
                 QStringLiteral("区域库包含同名记录：%1").arg(record.name));
        return false;
      }
    }
    parsed_regions.push_back(record);
  }
  *records = parsed_regions;
  return true;
}

bool CoverageRegionStore::validateName(const QString& name, QString* error)
{
  if (name.isEmpty() || name != name.trimmed())
  {
    setError(error, QStringLiteral("区域名称不能为空或带首尾空格"));
    return false;
  }
  if (name.size() > 80)
  {
    setError(error, QStringLiteral("区域名称不能超过 80 个字符"));
    return false;
  }
  for (const QChar character : name)
  {
    if (character.category() == QChar::Other_Control ||
        character.category() == QChar::Separator_Line ||
        character.category() == QChar::Separator_Paragraph)
    {
      setError(error, QStringLiteral("区域名称不能包含控制符或换行"));
      return false;
    }
  }
  return true;
}

bool CoverageRegionStore::validatePolygon(const QVector<QPointF>& polygon,
                                          QString* error)
{
  if (polygon.size() < 3)
  {
    setError(error, QStringLiteral("覆盖区域至少需要 3 个不同顶点"));
    return false;
  }
  if (polygon.size() > kMaximumVertexCount)
  {
    setError(error, QStringLiteral("覆盖区域顶点数量超过 %1").arg(kMaximumVertexCount));
    return false;
  }
  for (int index = 0; index < polygon.size(); ++index)
  {
    const QPointF& point = polygon[index];
    if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
        std::abs(point.x()) > kMaximumCoordinateMagnitude ||
        std::abs(point.y()) > kMaximumCoordinateMagnitude)
    {
      setError(error, QStringLiteral("覆盖区域包含无效或超范围坐标"));
      return false;
    }
    for (int other = 0; other < index; ++other)
    {
      if (samePoint(point, polygon[other]))
      {
        setError(error, QStringLiteral("覆盖区域包含重复顶点"));
        return false;
      }
    }
  }
  double twice_area = 0.0;
  for (int index = 0; index < polygon.size(); ++index)
  {
    const QPointF& first = polygon[index];
    const QPointF& second = polygon[(index + 1) % polygon.size()];
    twice_area += first.x() * second.y() - second.x() * first.y();
  }
  const double area = std::abs(twice_area) * 0.5;
  if (!std::isfinite(area) || area < kMinimumAreaM2 || area > kMaximumAreaM2)
  {
    setError(error, QStringLiteral("覆盖区域面积必须在 0.01 到 100000000 m² 之间"));
    return false;
  }
  const int count = polygon.size();
  for (int first = 0; first < count; ++first)
  {
    const int first_end = (first + 1) % count;
    for (int second = first + 1; second < count; ++second)
    {
      const int second_end = (second + 1) % count;
      if (first == second || first_end == second || second_end == first)
        continue;
      if (segmentsIntersect(polygon[first], polygon[first_end],
                            polygon[second], polygon[second_end]))
      {
        setError(error, QStringLiteral("覆盖区域边界存在自交"));
        return false;
      }
    }
  }
  return true;
}

void CoverageRegionStore::setError(QString* error, const QString& message)
{
  if (error)
    *error = message;
}

bool CoverageRegionStore::validateContext(QString* error) const
{
  if (root_.trimmed().isEmpty())
  {
    setError(error, QStringLiteral("coverage_region_root 未配置"));
    return false;
  }
  if (root_.size() > kMaximumPathCharacters ||
      containsUnsafeControlCharacter(root_) ||
      !QDir::isAbsolutePath(root_) ||
      QDir::cleanPath(root_) == QDir::rootPath())
  {
    setError(error,
             QStringLiteral("coverage_region_root 必须是非根目录的绝对安全路径"));
    return false;
  }
  const QString canonical_root = canonicalDirectoryPath(root_);
  const QString canonical_source = canonicalDirectoryPath(map_source_);
  if (canonical_root.isEmpty() || canonical_source.isEmpty() ||
      canonical_root != canonical_source)
  {
    setError(error,
             QStringLiteral("区域库必须绑定到当前静态地图的 map-set 目录"));
    return false;
  }
  const QRegularExpression digest_pattern(QStringLiteral("^[0-9a-f]{64}$"));
  const QRegularExpression mode_pattern(QStringLiteral("^[A-Za-z0-9_.-]{1,64}$"));
  if (!digest_pattern.match(map_digest_).hasMatch() || map_source_.isEmpty() ||
      !mode_pattern.match(source_mode_).hasMatch() ||
      source_mode_ == QStringLiteral(".") ||
      source_mode_ == QStringLiteral(".."))
  {
    setError(error, QStringLiteral("当前静态地图身份不完整"));
    return false;
  }
  return true;
}

bool CoverageRegionStore::validateRecord(const CoverageRegionRecord& record,
                                         QString* error) const
{
  if (normalizedUuid(record.id) != record.id)
  {
    setError(error, QStringLiteral("区域 UUID 无效或不是规范格式"));
    return false;
  }
  if (!validateName(record.name, error) || !validatePolygon(record.polygon, error))
    return false;
  if (record.map_digest != map_digest_ || record.source_mode != source_mode_)
  {
    setError(error, QStringLiteral("区域记录地图身份与当前区域库不一致"));
    return false;
  }
  if (record.map_source.isEmpty() ||
      record.map_source.size() > kMaximumPathCharacters ||
      containsUnsafeControlCharacter(record.map_source))
  {
    setError(error, QStringLiteral("区域记录的地图来源无效"));
    return false;
  }
  if (record.revision == 0 || !validTimestamp(record.created_at) ||
      !validTimestamp(record.updated_at) || record.updated_at < record.created_at)
  {
    setError(error, QStringLiteral("区域记录版本或时间戳无效"));
    return false;
  }
  return true;
}

bool CoverageRegionStore::saveRecords(
    const QVector<CoverageRegionRecord>& records, QString* error)
{
  if (!validateContext(error))
    return false;
  if (records.size() > kMaximumRegionCount)
  {
    setError(error, QStringLiteral("区域记录数量超过上限"));
    return false;
  }
  for (int index = 0; index < records.size(); ++index)
  {
    if (!validateRecord(records[index], error))
      return false;
    for (int other = 0; other < index; ++other)
    {
      if (records[index].id == records[other].id ||
          records[index].name.compare(records[other].name,
                                      Qt::CaseInsensitive) == 0)
      {
        setError(error, QStringLiteral("区域库包含重复 UUID 或同名记录"));
        return false;
      }
    }
  }
  const QString path = filePath();
  const QString directory_path = QFileInfo(path).absolutePath();
  if (!prepareStoreDirectory(root_, directory_path, error) ||
      !rejectSymlinkFile(path, error) ||
      !rejectSymlinkFile(path + QStringLiteral(".lock"), error))
    return false;
  QLockFile lock(path + QStringLiteral(".lock"));
  lock.setStaleLockTime(30000);
  if (!lock.tryLock(2000))
  {
    setError(error, QStringLiteral("区域库正被另一个进程使用"));
    return false;
  }
  // QLockFile serializes writers, but by itself cannot prevent a process that
  // loaded an older snapshot from overwriting a newer committed snapshot.
  // Compare the exact bytes observed by load() while holding the write lock;
  // stale callers must reload instead of silently losing another writer's
  // region.
  QFile current_file(path);
  const bool current_exists = current_file.exists();
  if (current_exists != loaded_store_existed_)
  {
    loaded_ = false;
    setError(error, QStringLiteral("区域库已被其他进程修改；请等待重新加载后再试"));
    return false;
  }
  if (current_exists)
  {
    if (current_file.size() < 0 || current_file.size() > kMaximumStoreBytes ||
        !current_file.open(QIODevice::ReadOnly))
    {
      setError(error, QStringLiteral("无法复核区域库当前版本；不会覆盖现有文件"));
      return false;
    }
    const QByteArray current_payload = current_file.readAll();
    const QByteArray current_fingerprint =
        QCryptographicHash::hash(current_payload, QCryptographicHash::Sha256);
    if (current_fingerprint != loaded_store_fingerprint_)
    {
      loaded_ = false;
      setError(error, QStringLiteral("区域库已被其他进程修改；请等待重新加载后再试"));
      return false;
    }
  }
  QJsonArray serialized_regions;
  for (const CoverageRegionRecord& record : records)
    serialized_regions.append(recordToJson(record));
  QJsonObject root;
  root.insert(QStringLiteral("schema_version"), kSchemaVersion);
  root.insert(QStringLiteral("map_digest"), map_digest_);
  root.insert(QStringLiteral("map_source"), map_source_);
  root.insert(QStringLiteral("source_mode"), source_mode_);
  root.insert(QStringLiteral("regions"), serialized_regions);
  const QByteArray payload = QJsonDocument(root).toJson(QJsonDocument::Indented);
  if (payload.size() > kMaximumStoreBytes)
  {
    setError(error, QStringLiteral("区域库序列化后超过 4 MiB 上限"));
    return false;
  }
  QSaveFile file(path);
  file.setDirectWriteFallback(false);
  if (!file.open(QIODevice::WriteOnly))
  {
    setError(error, QStringLiteral("无法写入区域库：%1").arg(file.errorString()));
    return false;
  }
  if (file.write(payload) != payload.size())
  {
    file.cancelWriting();
    setError(error, QStringLiteral("区域库写入不完整：%1").arg(file.errorString()));
    return false;
  }
  if (!file.commit())
  {
    setError(error, QStringLiteral("区域库原子提交失败：%1").arg(file.errorString()));
    return false;
  }
  loaded_store_existed_ = true;
  loaded_store_fingerprint_ =
      QCryptographicHash::hash(payload, QCryptographicHash::Sha256);
  return true;
}

}  // namespace autolabor_operator_gui
