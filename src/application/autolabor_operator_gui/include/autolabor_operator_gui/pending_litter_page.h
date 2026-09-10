#ifndef AUTOLABOR_OPERATOR_GUI_PENDING_LITTER_PAGE_H
#define AUTOLABOR_OPERATOR_GUI_PENDING_LITTER_PAGE_H

#include <autolabor_operator_gui/pending_litter_registry.h>
#include <autolabor_fod_msgs/FodMapPositionArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <QGraphicsView>
#include <QFutureWatcher>
#include <QWidget>
#include <QSet>

class QGraphicsPixmapItem;
class QLabel;
class QListWidget;
class QPushButton;

namespace autolabor_operator_gui
{
struct PendingLitterInput
{
  nav_msgs::OccupancyGrid::ConstPtr map;
  autolabor_fod_msgs::FodMapPositionArray::ConstPtr frame;
  ros::WallTime received_at;
  ros::Time received_stamp;
  bool reset = false;
};

struct PendingLitterCleaningUiResult
{
  bool transport_ok = false;
  bool accepted = false;
  bool cancel_request = false;
  QString message;
};

class PendingLitterMapView : public QGraphicsView
{
public:
  explicit PendingLitterMapView(QWidget* parent = nullptr);
  void setMap(const nav_msgs::OccupancyGrid& map);
  void setPoints(const QVector<PendingLitterPoint>& points);
  QPointF worldToScene(const QPointF& point) const;
  void fitMap();
protected:
  void wheelEvent(QWheelEvent* event) override;
  void showEvent(QShowEvent* event) override;
private:
  QGraphicsPixmapItem* map_item_ = nullptr;
  QVector<QGraphicsItem*> markers_;
  nav_msgs::MapMetaData geometry_;
  double yaw_ = 0.0;
  bool needs_fit_ = false;
};

class PendingLitterPage : public QWidget
{
public:
  explicit PendingLitterPage(QWidget* parent = nullptr);
  void configure(const QString& root, const QString& map_source, bool persist);
  void setAvailable(bool available);
  void consume(const PendingLitterInput& input, const ros::Time& now,
               const ros::WallTime& wall_now);
  const QVector<PendingLitterPoint>& points() const { return registry_.points(); }
  void markCleaned(quint64 object_id);
  void updateCleaningStatus(const QString& json_status);
  static QString mapKey(const nav_msgs::OccupancyGrid& map, const QString& source);
private:
  void redrawPoints();
  void updateStatus();
  int selectedPointIndex(QListWidget* list) const;
  void updateButtons();
  void startCleaningRequest();
  void cancelCleaningRequest();
  PendingLitterRegistry registry_;
  QSet<QString> cleaning_labels_;
  QSet<quint64> cleaned_object_ids_;
  QString source_, map_key_, stream_key_;
  QString stability_status_;
  bool configured_ = false;
  bool available_ = false;
  bool map_ready_ = false;
  bool current_position_valid_ = false;
  bool cleaning_request_busy_ = false;
  bool cleaning_mission_requested_ = false;
  ros::Time map_changed_at_;
  ros::WallTime last_frame_received_;
  PendingLitterMapView* map_view_ = nullptr;
  QListWidget* list_ = nullptr;
  QListWidget* cleaning_list_ = nullptr;
  QLabel* cleaning_notice_ = nullptr;
  QLabel* status_ = nullptr;
  QPushButton* fit_button_ = nullptr;
  QPushButton* delete_button_ = nullptr;
  QPushButton* add_button_ = nullptr;
  QPushButton* return_button_ = nullptr;
  QPushButton* clean_button_ = nullptr;
  QPushButton* cancel_clean_button_ = nullptr;
  QFutureWatcher<PendingLitterCleaningUiResult>* cleaning_watcher_ = nullptr;
};
}  // namespace autolabor_operator_gui
#endif
