#ifndef AUTOLABOR_OPERATOR_GUI_COVERAGE_REGION_STORE_H
#define AUTOLABOR_OPERATOR_GUI_COVERAGE_REGION_STORE_H

#include <QDateTime>
#include <QByteArray>
#include <QPointF>
#include <QString>
#include <QVector>

namespace autolabor_operator_gui
{

struct CoverageRegionRecord
{
  QString id;
  QString name;
  QString map_digest;
  QString map_source;
  QString source_mode;
  QVector<QPointF> polygon;
  quint64 revision = 1;
  QDateTime created_at;
  QDateTime updated_at;
};

class CoverageRegionStore
{
public:
  static constexpr int kSchemaVersion = 1;
  static constexpr int kMaximumRegionCount = 1000;
  static constexpr int kMaximumVertexCount = 4096;

  explicit CoverageRegionStore(const QString& root = QString());

  void setRoot(const QString& root);
  void setLegacyRoot(const QString& legacy_root);
  bool setMapContext(const QString& map_digest, const QString& map_source,
                     const QString& source_mode, QString* error = nullptr);
  bool load(QString* error = nullptr);

  QString root() const;
  QString legacyRoot() const;
  QString mapDigest() const;
  QString mapSource() const;
  QString sourceMode() const;
  QString filePath() const;
  QString legacyFilePath() const;
  bool isLoaded() const;
  QVector<CoverageRegionRecord> regions() const;
  bool findById(const QString& id, CoverageRegionRecord* record) const;
  bool containsName(const QString& name, const QString& except_id = QString()) const;

  bool addRegion(const QString& name, const QVector<QPointF>& polygon,
                 CoverageRegionRecord* created = nullptr,
                 QString* error = nullptr);
  bool removeRegion(const QString& id, QString* error = nullptr);

  static bool validateName(const QString& name, QString* error = nullptr);
  static bool validatePolygon(const QVector<QPointF>& polygon,
                              QString* error = nullptr);

private:
  static void setError(QString* error, const QString& message);
  bool validateContext(QString* error) const;
  bool validateRecord(const CoverageRegionRecord& record,
                      QString* error) const;
  bool parseStorePayload(const QByteArray& payload, bool require_source_match,
                         QVector<CoverageRegionRecord>* records,
                         QString* error) const;
  bool saveRecords(const QVector<CoverageRegionRecord>& records,
                   QString* error);

  QString root_;
  QString legacy_root_;
  QString map_digest_;
  QString map_source_;
  QString source_mode_;
  QVector<CoverageRegionRecord> regions_;
  bool loaded_ = false;
  bool loaded_store_existed_ = false;
  QByteArray loaded_store_fingerprint_;
};

}  // namespace autolabor_operator_gui

#endif  // AUTOLABOR_OPERATOR_GUI_COVERAGE_REGION_STORE_H
