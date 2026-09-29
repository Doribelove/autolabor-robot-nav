#ifndef AUTOLABOR_OPERATOR_GUI_ZED_CAMERA_PANEL_H
#define AUTOLABOR_OPERATOR_GUI_ZED_CAMERA_PANEL_H

#include <dynamic_reconfigure/Config.h>
#include <QWidget>
#include <map>
#include <string>

class QCheckBox;
class QLabel;
class QPushButton;
class QSpinBox;

namespace autolabor_operator_gui
{

class ZedCameraPanel : public QWidget
{
  Q_OBJECT
public:
  explicit ZedCameraPanel(const std::string& camera_node, QWidget* parent = nullptr);
  void setOnline(bool online);

Q_SIGNALS:
  void operationEvent(const QString& message, bool error);

private:
  void request(bool apply, bool restore_auto = false);
  void updateEnabled();
  bool displayConfig(const dynamic_reconfigure::Config& config);

  std::string camera_node_;
  bool online_ = false;
  bool busy_ = false;
  bool loaded_ = false;
  std::map<std::string, QSpinBox*> integers_;
  QCheckBox* auto_exposure_ = nullptr;
  QCheckBox* auto_whitebalance_ = nullptr;
  QPushButton* query_ = nullptr;
  QPushButton* apply_ = nullptr;
  QPushButton* automatic_ = nullptr;
  QLabel* status_ = nullptr;
  QLabel* acquisition_ = nullptr;
};

}  // namespace autolabor_operator_gui
#endif
