#ifndef AUTOLABOR_OPERATOR_GUI_GPS_MAP_PANEL_H
#define AUTOLABOR_OPERATOR_GUI_GPS_MAP_PANEL_H
#include <QWidget>
#include <QJsonArray>
#include <QJsonObject>
#include <ros/ros.h>
#include <memory>
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QGraphicsView;
class QLabel;
class QPlainTextEdit;
class QTableWidget;
namespace autolabor_operator_gui
{
class KeepoutPanel : public QWidget
{
  Q_OBJECT
public:
  explicit KeepoutPanel(QWidget* parent=nullptr);
  void setOnline(bool online);
  QJsonObject configuration() const;
  void loadConfiguration(const QJsonObject& config);
Q_SIGNALS:
  void operationEvent(const QString& text,bool error);
private:
  void selectZone(int row);
  void showPoints();
  void changePoints(const QJsonArray& points);
  void changed();
  void run(const QString& command,const QJsonObject& payload);
  QJsonArray zones_;
  QJsonObject airport_;
  int current_=-1;
  bool loaded_=false, online_=false, dirty_=false, busy_=false, have_fix_=false;
  double fix_lat_=0.,fix_lon_=0.;
  std::unique_ptr<ros::NodeHandle> node_;
  ros::Subscriber state_sub_,fix_sub_;
  QComboBox* zones_box_;
  QCheckBox* enabled_;
  QDoubleSpinBox *clearance_,*latitude_,*longitude_,*resolution_;
  QTableWidget* points_;
  QGraphicsView* preview_;
  QLabel *status_, *airport_status_;
};

class FodTargetsPanel : public QWidget
{
  Q_OBJECT
public:
  explicit FodTargetsPanel(QWidget* parent=nullptr);
  void setOnline(bool online);
  void loadPoints(const QJsonArray& points);
Q_SIGNALS:
  void operationEvent(const QString& text,bool error);
  void navigationRequested(const QString& name,double latitude,double longitude);
  void showNavigationMap();
private:
  void run(const QString& command,const QJsonValue& payload,bool apply_after=false);
  void refreshDistances();
  QJsonArray targets_;
  QJsonObject airport_;
  bool online_=false,busy_=false,loaded_=false,have_fix_=false;
  double fix_lat_=0.,fix_lon_=0.;
  std::unique_ptr<ros::NodeHandle> node_;
  ros::Subscriber state_sub_,fix_sub_,keepout_sub_;
  ros::Publisher selection_pub_;
  QTableWidget* table_;
  QPlainTextEdit* input_;
  QGraphicsView* preview_;
  QLabel* status_;
};
}
#endif
