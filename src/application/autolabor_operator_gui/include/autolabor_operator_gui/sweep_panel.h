#ifndef AUTOLABOR_OPERATOR_GUI_SWEEP_PANEL_H
#define AUTOLABOR_OPERATOR_GUI_SWEEP_PANEL_H

#include <QProcess>
#include <QTimer>
#include <QWidget>

class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QSpinBox;

namespace autolabor_operator_gui
{

class SweepPanel : public QWidget
{
  Q_OBJECT

public:
  explicit SweepPanel(const QString& script, QWidget* parent = nullptr);

public Q_SLOTS:
  void queryStatus();

Q_SIGNALS:
  void stateChanged(int state, const QString& detail, bool error);
  void operationEvent(const QString& message, bool error);

protected:
  void showEvent(QShowEvent* event) override;
  void hideEvent(QHideEvent* event) override;

private:
  void request(const QString& action);
  void finished(int exit_code, QProcess::ExitStatus exit_status);
  void complete(int state, const QString& message, bool error);
  void setBusy(bool busy);
  void appendLog(const QString& message);

  QString script_;
  QString action_;
  QByteArray output_;
  QByteArray diagnostics_;
  QProcess process_;
  QTimer refresh_timer_;
  bool busy_ = false;
  bool stop_pending_ = false;
  QLineEdit* address_ = nullptr;
  QSpinBox* port_ = nullptr;
  QLabel* state_ = nullptr;
  QLabel* result_ = nullptr;
  QLabel* updated_ = nullptr;
  QPushButton* start_ = nullptr;
  QPushButton* stop_ = nullptr;
  QPushButton* query_ = nullptr;
  QPlainTextEdit* log_ = nullptr;
};

}  // namespace autolabor_operator_gui

#endif
