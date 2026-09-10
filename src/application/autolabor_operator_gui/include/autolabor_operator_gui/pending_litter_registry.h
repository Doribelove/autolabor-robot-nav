#ifndef AUTOLABOR_OPERATOR_GUI_PENDING_LITTER_REGISTRY_H
#define AUTOLABOR_OPERATOR_GUI_PENDING_LITTER_REGISTRY_H

#include <QByteArray>
#include <QPointF>
#include <QString>
#include <QVector>
#include <array>

namespace autolabor_operator_gui
{
struct LitterObservation
{
  QPointF position;
  QString category;
  std::array<double, 3> odom{{0, 0, 0}};
  quint64 object_id = 0;
};

struct PendingLitterPoint
{
  QString label;
  QString category;
  QPointF position;
  QString state = "CONFIRMED";
  QString session;
  std::array<double, 3> odom{{0, 0, 0}};
  std::array<double, 3> anchor{{0, 0, 0}};
  quint64 object_id = 0;
  double first_seen = 0, last_seen = 0;
  quint64 observations = 0;
  double spread = 0;
  QVector<LitterObservation> samples;
};

// Session-local association, stable map projection, no task/control interface.
class PendingLitterRegistry
{
public:
  static constexpr int kConfirmationFrames = 5;
  static constexpr double kMergeDistance = 0.15;
  static constexpr double kMaximumFrameGap = 0.5;
  bool observe(double stamp, const QVector<LitterObservation>& observations,
               const QString& session, const std::array<double, 16>& map_from_odom,
               bool stable, quint64 generation);
  void breakContinuity();
  bool suspend();
  bool removePoint(int index);
  void reset(const QVector<PendingLitterPoint>& points = {});
  const QVector<PendingLitterPoint>& points() const { return points_; }
  static bool nearby(const QPointF& first, const QPointF& second);
  static QString labelForIndex(int index);

private:
  struct Candidate
  {
    std::array<double, 3> anchor;
    QVector<LitterObservation> samples;
    double first_seen = 0;
  };
  QVector<Candidate> candidates_;
  QVector<PendingLitterPoint> points_;
  int next_label_ = 0;
  double last_stamp_ = 0.0;
  QString session_;
  quint64 generation_ = 0;
};

// Separate file per static map identity; atomic writes and conflict detection.
class PendingLitterStore
{
public:
  bool open(const QString& root, const QString& map_key,
            QVector<PendingLitterPoint>* points, QString* error);
  bool save(const QVector<PendingLitterPoint>& points, QString* error);
  QString filePath() const { return path_; }
private:
  QString path_;
  QString map_key_;
  QByteArray fingerprint_;
  bool existed_ = false;
  bool loaded_ = false;
};
}  // namespace autolabor_operator_gui
#endif
