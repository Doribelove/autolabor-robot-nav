#include <autolabor_operator_gui/sweep_panel.h>

#include <QDateTime>
#include <QFormLayout>
#include <QGroupBox>
#include <QHideEvent>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QShowEvent>
#include <QSpinBox>
#include <QVBoxLayout>

namespace autolabor_operator_gui
{

SweepPanel::SweepPanel(const QString& script, QWidget* parent)
  : QWidget(parent), script_(script)
{
  setObjectName(QStringLiteral("sweepPanel"));
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(32, 24, 32, 24);
  root->setSpacing(18);
  auto* heading = new QLabel(QStringLiteral("清扫装置控制"), this);
  heading->setStyleSheet(QStringLiteral("font-size:24pt;font-weight:700;"));
  root->addWidget(heading);

  auto* connection = new QGroupBox(QStringLiteral("控制器连接"), this);
  auto* form = new QFormLayout(connection);
  address_ = new QLineEdit(QStringLiteral("192.168.0.197"), connection);
  address_->setObjectName(QStringLiteral("sweepAddress"));
  port_ = new QSpinBox(connection);
  port_->setObjectName(QStringLiteral("sweepPort"));
  port_->setRange(1, 65535);
  port_->setValue(50003);
  form->addRow(QStringLiteral("设备 IP"), address_);
  form->addRow(QStringLiteral("端口"), port_);
  root->addWidget(connection);

  auto* status = new QGroupBox(QStringLiteral("设备回读状态"), this);
  auto* status_layout = new QVBoxLayout(status);
  state_ = new QLabel(QStringLiteral("未查询"), status);
  state_->setObjectName(QStringLiteral("sweepState"));
  state_->setStyleSheet(QStringLiteral("font-size:28pt;font-weight:700;color:#8fa0b5;"));
  updated_ = new QLabel(QStringLiteral("尚未收到设备状态"), status);
  updated_->setObjectName(QStringLiteral("sweepUpdated"));
  result_ = new QLabel(QStringLiteral("进入本页后自动查询，也可点击刷新状态。"), status);
  result_->setObjectName(QStringLiteral("sweepResult"));
  result_->setWordWrap(true);
  status_layout->addWidget(state_);
  status_layout->addWidget(updated_);
  status_layout->addWidget(result_);
  root->addWidget(status);

  auto* buttons = new QHBoxLayout();
  start_ = new QPushButton(QStringLiteral("启动清扫"), this);
  start_->setObjectName(QStringLiteral("sweepStartButton"));
  stop_ = new QPushButton(QStringLiteral("停止清扫"), this);
  stop_->setObjectName(QStringLiteral("sweepStopButton"));
  query_ = new QPushButton(QStringLiteral("刷新状态"), this);
  query_->setObjectName(QStringLiteral("sweepQueryButton"));
  start_->setStyleSheet(QStringLiteral("QPushButton:enabled {background:#17805b;border:1px solid #20b47a;}"));
  stop_->setStyleSheet(QStringLiteral("QPushButton:enabled {background:#813a42;border:1px solid #b34d57;}"));
  for (auto* button : {start_, stop_, query_})
  {
    button->setMinimumHeight(56);
    buttons->addWidget(button, 1);
  }
  root->addLayout(buttons);
  auto* note = new QLabel(QStringLiteral(
      "本页显示时每 5 秒刷新状态。操作完成后以设备回读为准。\n"
      "使用本页前请退出命令行清扫测试。关闭窗口不会关闭清扫装置，停机请点击“停止清扫”。"), this);
  note->setWordWrap(true);
  note->setStyleSheet(QStringLiteral("color:#9cafc4;font-size:11pt;"));
  root->addWidget(note);
  log_ = new QPlainTextEdit(this);
  log_->setObjectName(QStringLiteral("sweepLog"));
  log_->setReadOnly(true);
  log_->setMaximumBlockCount(300);
  root->addWidget(log_, 1);

  connect(start_, &QPushButton::clicked, this, [this]() { request(QStringLiteral("on")); });
  connect(stop_, &QPushButton::clicked, this, [this]() { request(QStringLiteral("off")); });
  connect(query_, &QPushButton::clicked, this, &SweepPanel::queryStatus);
  connect(&refresh_timer_, &QTimer::timeout, this, &SweepPanel::queryStatus);
  refresh_timer_.setInterval(5000);
  connect(&process_, &QProcess::readyReadStandardOutput, this,
          [this]() { output_ += process_.readAllStandardOutput(); });
  connect(&process_, &QProcess::readyReadStandardError, this, [this]() {
    const auto data = process_.readAllStandardError();
    diagnostics_ += data;
    appendLog(QString::fromUtf8(data).trimmed());
  });
  connect(&process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this, &SweepPanel::finished);
  connect(&process_, &QProcess::errorOccurred, this, [this](QProcess::ProcessError error) {
    if (error == QProcess::FailedToStart)
      complete(-1, QStringLiteral("清扫控制程序无法启动：%1").arg(process_.errorString()), true);
  });
}

void SweepPanel::showEvent(QShowEvent* event)
{
  QWidget::showEvent(event);
  refresh_timer_.start();
  QTimer::singleShot(0, this, &SweepPanel::queryStatus);
}

void SweepPanel::hideEvent(QHideEvent* event)
{
  refresh_timer_.stop();
  QWidget::hideEvent(event);
}

void SweepPanel::queryStatus()
{
  request(QStringLiteral("status"));
}

void SweepPanel::request(const QString& action)
{
  if (busy_)
  {
    if (action == QStringLiteral("off") && action_ != QStringLiteral("off"))
    {
      stop_pending_ = true;
      stop_->setEnabled(false);
      result_->setText(QStringLiteral("已接收停止请求，当前通信结束后立即停止清扫。"));
    }
    return;
  }
  if (address_->text().trimmed().isEmpty())
  {
    result_->setText(QStringLiteral("请填写设备 IP。"));
    return;
  }
  action_ = action;
  output_.clear();
  diagnostics_.clear();
  setBusy(true);
  const QString verb = action == QStringLiteral("on") ? QStringLiteral("启动清扫") :
                       action == QStringLiteral("off") ? QStringLiteral("停止清扫") : QStringLiteral("查询状态");
  result_->setText(QStringLiteral("正在%1，等待设备回读……").arg(verb));
  if (action != QStringLiteral("status"))
    Q_EMIT operationEvent(verb, false);
  process_.setProgram(QStringLiteral("/usr/bin/python3"));
  process_.setArguments({QStringLiteral("-u"), script_, action, QStringLiteral("--json"),
                        QStringLiteral("--ip"), address_->text().trimmed(),
                        QStringLiteral("--port"), QString::number(port_->value())});
  process_.start();
}

void SweepPanel::setBusy(bool busy)
{
  busy_ = busy;
  start_->setEnabled(!busy);
  query_->setEnabled(!busy);
  address_->setEnabled(!busy);
  port_->setEnabled(!busy);
  stop_->setEnabled(!stop_pending_ && (!busy || action_ != QStringLiteral("off")));
}

void SweepPanel::finished(int exit_code, QProcess::ExitStatus exit_status)
{
  output_ += process_.readAllStandardOutput();
  diagnostics_ += process_.readAllStandardError();
  QJsonParseError parse_error;
  const auto document = QJsonDocument::fromJson(output_.trimmed(), &parse_error);
  const auto object = document.object();
  const bool valid = parse_error.error == QJsonParseError::NoError && document.isObject() &&
                     object.value(QStringLiteral("action")).toString() == action_ &&
                     object.value(QStringLiteral("success")).isBool();
  const bool success = valid && exit_status == QProcess::NormalExit && exit_code == 0 &&
                       object.value(QStringLiteral("success")).toBool();
  int state = valid ? object.value(QStringLiteral("state")).toInt(-1) : -1;
  if (state != 0 && state != 1)
    state = -1;
  QString message = valid ? object.value(QStringLiteral("message")).toString() :
                            QStringLiteral("控制程序未返回有效结果；请重新查询状态。%1")
                                .arg(QString::fromUtf8(diagnostics_).trimmed());
  complete(state, message, !success || state < 0);
}

void SweepPanel::complete(int state, const QString& message, bool error)
{
  const QString now = QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss"));
  state_->setText(state == 1 ? QStringLiteral("已开启") :
                  state == 0 ? QStringLiteral("已关闭") : QStringLiteral("状态未确认"));
  const QString color = error ? QStringLiteral("#e45b61") : QStringLiteral("#20b47a");
  state_->setStyleSheet(QStringLiteral("font-size:28pt;font-weight:700;color:%1;").arg(color));
  updated_->setText(state >= 0 ? QStringLiteral("最近回读：%1").arg(now) :
                                QStringLiteral("%1  查询失败，请检查控制器连接").arg(now));
  result_->setText(message);
  Q_EMIT stateChanged(state, updated_->text(), error);
  if (action_ != QStringLiteral("status") || error)
    Q_EMIT operationEvent(message, error);
  appendLog(message);
  const bool stop_next = stop_pending_;
  stop_pending_ = false;
  setBusy(false);
  if (stop_next)
    request(QStringLiteral("off"));
}

void SweepPanel::appendLog(const QString& message)
{
  if (!message.isEmpty())
    log_->appendPlainText(QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss  ")) + message);
}

}  // namespace autolabor_operator_gui
