#include <autolabor_operator_gui/main_window.h>

#include <sweeper_mcp/CancelAiTask.h>
#include <sweeper_mcp/SetAiAuthorization.h>
#include <sweeper_mcp/SetAsrModel.h>
#include <sweeper_mcp/SetAsrRecording.h>
#include <sweeper_mcp/SetSmartVoice.h>
#include <sweeper_mcp/SubmitAiText.h>

#include <QDateTime>
#include <QComboBox>
#include <QFutureWatcher>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QtConcurrent/QtConcurrentRun>

#include <deque>

namespace autolabor_operator_gui
{

void MainWindow::toggleAiVoiceAuthorization()
{
  const TelemetrySnapshot data = snapshot();
  const bool enabled = data.ai_status_received && data.ai_status.voice_authorized;
  if (!enabled)
  {
    const auto answer = QMessageBox::question(
        this, QStringLiteral("确认语音输入授权"),
        QStringLiteral("本次运行允许访问麦克风并使用本地 ASR。授权后麦克风仍保持"
                       "关闭，需要另行点击“开始录音”或确认“启用智能语音”；"
                       "未授权 AI 语义解析时，识别文字只在本机界面显示，不会发送"
                       "云端。是否确认？"),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (answer != QMessageBox::Yes)
      return;
  }
  requestAiAuthorization("voice", !enabled);
}

void MainWindow::toggleAiAsrRecording()
{
  if (ai_asr_request_pending_)
    return;
  const TelemetrySnapshot data = snapshot();
  const bool fresh = data.ai_status_received &&
                     wallAge(data.ai_status_received_at) <= 2.5;
  if (!fresh || !data.ai_status.ui_session_alive ||
      ai_session_token_.isEmpty())
  {
    QMessageBox::warning(this, QStringLiteral("ASR 后端未连接"),
                         QStringLiteral("Qt 会话心跳或 ASR 控制服务尚未就绪。"));
    return;
  }
  if (data.ai_status.smart_voice_enabled ||
      data.ai_status.smart_voice_listening ||
      ai_smart_voice_request_pending_)
  {
    QMessageBox::information(
        this, QStringLiteral("智能语音正在使用麦克风"),
        QStringLiteral("请先关闭智能语音并等待监听停止，再使用手动录音。"));
    return;
  }

  const QString phase = QString::fromStdString(data.ai_status.asr_phase).toUpper();
  const bool has_recorded_audio = phase == QStringLiteral("RECORDED");
  const bool recording_or_recorded =
      data.ai_status.asr_recording || has_recorded_audio;
  const bool asr_busy = phase == QStringLiteral("LOADING") ||
                        phase == QStringLiteral("TRANSCRIBING") ||
                        phase == QStringLiteral("RECOGNIZING") ||
                        phase == QStringLiteral("CANCELLING") ||
                        phase == QStringLiteral("STARTING") ||
                        phase == QStringLiteral("STOPPING");
  if (asr_busy)
  {
    QMessageBox::information(this, QStringLiteral("ASR 正在处理"),
                             QStringLiteral("请等待当前 ASR 操作完成。"));
    return;
  }
  if (!recording_or_recorded)
  {
    if (!data.ai_status.voice_authorized)
    {
      QMessageBox::information(this, QStringLiteral("语音输入授权关闭"),
                               QStringLiteral("请先确认语音输入授权。"));
      return;
    }
    if (!data.ai_status.asr_available)
    {
      QMessageBox::warning(this, QStringLiteral("ASR 不可用"),
                           QStringLiteral("请检查本地 ASR 模型、推理环境和麦克风。"));
      return;
    }
  }

  ai_asr_request_pending_ = true;
  const bool requested_recording = !recording_or_recorded;
  const std::string token = ai_session_token_.toStdString();
  auto* watcher = new QFutureWatcher<AsrRecordingUiResult>(this);
  connect(watcher, &QFutureWatcher<AsrRecordingUiResult>::finished, this,
          [this, watcher, requested_recording]() {
            const AsrRecordingUiResult result = watcher->result();
            ai_asr_request_pending_ = false;
            const QString action = requested_recording
                                       ? QStringLiteral("开始录音")
                                       : QStringLiteral("停止并识别");
            if (result.transport_ok && result.accepted)
            {
              if (requested_recording && ai_transcript_)
                ai_transcript_->clear();
              if (ai_events_)
              {
                QString event = QDateTime::currentDateTime().toString(
                                    QStringLiteral("HH:mm:ss  ")) +
                                action + QStringLiteral(" · ") + result.message;
                if (!result.capture_id.isEmpty())
                  event += QStringLiteral(" · capture=") +
                           result.capture_id.left(12);
                ai_events_->appendPlainText(event);
              }
            }
            else
            {
              const QString error = action + QStringLiteral("失败：") + result.message;
              if (ai_events_)
                ai_events_->appendPlainText(
                    QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss  ")) +
                    error);
              QMessageBox::warning(this, QStringLiteral("ASR 操作失败"), error);
            }
            watcher->deleteLater();
          });
  watcher->setFuture(QtConcurrent::run([token, requested_recording]() {
    AsrRecordingUiResult result;
    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<sweeper_mcp::SetAsrRecording>(
        "/sweeper_ai/set_asr_recording", false);
    if (!client.waitForExistence(ros::Duration(1.0)))
    {
      result.message = QStringLiteral("/sweeper_ai/set_asr_recording 不在线");
      return result;
    }
    sweeper_mcp::SetAsrRecording call;
    call.request.session_token = token;
    call.request.recording = requested_recording;
    if (!client.call(call))
    {
      result.message = QStringLiteral("ASR 录音服务调用失败");
      return result;
    }
    result.transport_ok = true;
    result.accepted = call.response.accepted;
    result.capture_id = QString::fromStdString(call.response.capture_id);
    result.message = QString::fromStdString(call.response.message);
    return result;
  }));
}

void MainWindow::selectAiAsrModel(int index)
{
  if (!ai_asr_model_combo_ || ai_asr_model_request_pending_ || index < 0)
    return;
  const QString requested_model =
      ai_asr_model_combo_->itemData(index).toString().trimmed().toLower();
  if (requested_model != QStringLiteral("small") &&
      requested_model != QStringLiteral("medium") &&
      requested_model != QStringLiteral("large"))
    return;

  const TelemetrySnapshot data = snapshot();
  const bool fresh = data.ai_status_received &&
                     wallAge(data.ai_status_received_at) <= 2.5;
  QString active_model = QString::fromStdString(data.ai_status.asr_model)
                             .trimmed().toLower();
  if (active_model == QStringLiteral("large-v3"))
    active_model = QStringLiteral("large");
  auto restore_combo = [this](const QString& model) {
    if (!ai_asr_model_combo_)
      return;
    QString canonical = model.trimmed().toLower();
    if (canonical == QStringLiteral("large-v3"))
      canonical = QStringLiteral("large");
    const int active_index = ai_asr_model_combo_->findData(canonical);
    if (active_index >= 0)
    {
      const QSignalBlocker blocker(ai_asr_model_combo_);
      ai_asr_model_combo_->setCurrentIndex(active_index);
    }
  };
  if (!fresh || !data.ai_status.ui_session_alive || ai_session_token_.isEmpty())
  {
    restore_combo(active_model.isEmpty() ? QStringLiteral("medium")
                                         : active_model);
    QMessageBox::warning(this, QStringLiteral("ASR 后端未连接"),
                         QStringLiteral("Qt 会话心跳或 ASR 模型服务尚未就绪。"));
    return;
  }
  if (requested_model == active_model)
    return;

  const QString phase =
      QString::fromStdString(data.ai_status.asr_phase).toUpper();
  const bool busy = data.ai_status.asr_recording ||
                    data.ai_status.smart_voice_enabled ||
                    data.ai_status.smart_voice_listening ||
                    phase == QStringLiteral("STARTING") ||
                    phase == QStringLiteral("RECORDING") ||
                    phase == QStringLiteral("RECORDED") ||
                    phase == QStringLiteral("STOPPING") ||
                    phase == QStringLiteral("TRANSCRIBING") ||
                    phase == QStringLiteral("RECOGNIZING") ||
                    phase == QStringLiteral("CANCELLING") ||
                    phase == QStringLiteral("SMART_STARTING") ||
                    phase == QStringLiteral("SMART_LISTENING") ||
                    phase == QStringLiteral("SMART_STOPPING") ||
                    phase == QStringLiteral("SWITCHING");
  if (busy)
  {
    restore_combo(active_model);
    QMessageBox::information(
        this, QStringLiteral("ASR 正在使用中"),
        QStringLiteral("请先停止录音或智能语音，并等待当前识别完成后再切换模型。"));
    return;
  }

  ai_asr_model_request_pending_ = true;
  ai_asr_model_combo_->setEnabled(false);
  const std::string token = ai_session_token_.toStdString();
  const std::string model = requested_model.toStdString();
  auto* watcher = new QFutureWatcher<AsrModelUiResult>(this);
  connect(watcher, &QFutureWatcher<AsrModelUiResult>::finished, this,
          [this, watcher, requested_model]() {
            const AsrModelUiResult result = watcher->result();
            ai_asr_model_request_pending_ = false;
            const bool success = result.transport_ok && result.accepted;
            if (!success && ai_asr_model_combo_)
            {
              QString active = result.active_model.trimmed().toLower();
              if (active == QStringLiteral("large-v3"))
                active = QStringLiteral("large");
              const int active_index = ai_asr_model_combo_->findData(active);
              if (active_index >= 0)
              {
                const QSignalBlocker blocker(ai_asr_model_combo_);
                ai_asr_model_combo_->setCurrentIndex(active_index);
              }
            }
            if (ai_events_)
            {
              ai_events_->appendPlainText(
                  QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss  ")) +
                  QStringLiteral("ASR 模型 ") + requested_model +
                  (success ? QStringLiteral(" · ") : QStringLiteral(" 切换失败 · ")) +
                  result.message);
            }
            if (!success)
              QMessageBox::warning(this, QStringLiteral("ASR 模型未切换"),
                                   result.message);
            watcher->deleteLater();
          });
  watcher->setFuture(QtConcurrent::run([token, model]() {
    AsrModelUiResult result;
    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<sweeper_mcp::SetAsrModel>(
        "/sweeper_ai/set_asr_model", false);
    if (!client.waitForExistence(ros::Duration(1.0)))
    {
      result.message = QStringLiteral("/sweeper_ai/set_asr_model 不在线");
      return result;
    }
    sweeper_mcp::SetAsrModel call;
    call.request.session_token = token;
    call.request.model = model;
    if (!client.call(call))
    {
      result.message = QStringLiteral("ASR 模型服务调用失败");
      return result;
    }
    result.transport_ok = true;
    result.accepted = call.response.accepted;
    result.active_model = QString::fromStdString(call.response.active_model);
    result.message = QString::fromStdString(call.response.message);
    return result;
  }));
}

void MainWindow::toggleAiSmartVoice()
{
  if (ai_smart_voice_request_pending_)
    return;
  const TelemetrySnapshot data = snapshot();
  const bool fresh = data.ai_status_received &&
                     wallAge(data.ai_status_received_at) <= 2.5;
  if (!fresh || !data.ai_status.ui_session_alive ||
      ai_session_token_.isEmpty())
  {
    QMessageBox::warning(this, QStringLiteral("智能语音后端未连接"),
                         QStringLiteral("Qt 会话心跳或智能语音服务尚未就绪。"));
    return;
  }

  const bool enabled = data.ai_status.smart_voice_enabled ||
                       data.ai_status.smart_voice_listening;
  if (!enabled)
  {
    if (!data.ai_status.voice_authorized)
    {
      QMessageBox::information(this, QStringLiteral("语音输入授权关闭"),
                               QStringLiteral("请先确认语音输入授权。"));
      return;
    }
    if (!data.ai_status.asr_available)
    {
      QMessageBox::warning(this, QStringLiteral("ASR 不可用"),
                           QStringLiteral("请检查本地 ASR 模型、推理环境和麦克风。"));
      return;
    }
    const QString phase = QString::fromStdString(
        data.ai_status.asr_phase).toUpper();
    const bool asr_busy = data.ai_status.asr_recording ||
                          phase == QStringLiteral("RECORDED") ||
                          phase == QStringLiteral("LOADING") ||
                          phase == QStringLiteral("TRANSCRIBING") ||
                          phase == QStringLiteral("RECOGNIZING") ||
                          phase == QStringLiteral("CANCELLING") ||
                          phase == QStringLiteral("STARTING") ||
                          phase == QStringLiteral("STOPPING") ||
                          ai_asr_request_pending_;
    if (asr_busy)
    {
      QMessageBox::information(
          this, QStringLiteral("手动 ASR 正在处理"),
          QStringLiteral("请先停止手动录音或等待本次识别结束。"));
      return;
    }
    const auto answer = QMessageBox::question(
        this, QStringLiteral("确认启用智能语音"),
        QStringLiteral("智能语音会持续访问本机麦克风并自动断句、逐句识别。"
                       "若 AI 语义解析已授权，每句识别文字会自动发送到云端；"
                       "若 AI 控制也已授权，云端生成的计划可能通过 MCP 工具"
                       "控制无人车。关闭相应授权仍会立即阻止对应能力。"
                       "是否确认启用？"),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (answer != QMessageBox::Yes)
      return;
  }

  ai_smart_voice_request_pending_ = true;
  const bool requested_enabled = !enabled;
  const std::string token = ai_session_token_.toStdString();
  auto* watcher = new QFutureWatcher<SmartVoiceUiResult>(this);
  connect(watcher, &QFutureWatcher<SmartVoiceUiResult>::finished, this,
          [this, watcher, requested_enabled]() {
            const SmartVoiceUiResult result = watcher->result();
            ai_smart_voice_request_pending_ = false;
            const QString action = requested_enabled
                                       ? QStringLiteral("启用智能语音")
                                       : QStringLiteral("关闭智能语音");
            const bool success = result.transport_ok && result.accepted;
            QString event = QDateTime::currentDateTime().toString(
                                QStringLiteral("HH:mm:ss  ")) +
                            action + (success ? QStringLiteral(" · ")
                                              : QStringLiteral("失败 · ")) +
                            result.message;
            if (!result.session_id.isEmpty())
              event += QStringLiteral(" · session=") +
                       result.session_id.left(12);
            if (ai_events_)
              ai_events_->appendPlainText(event);
            if (!success)
              QMessageBox::warning(this, QStringLiteral("智能语音状态未变更"),
                                   action + QStringLiteral("失败：") +
                                       result.message);
            watcher->deleteLater();
          });
  watcher->setFuture(QtConcurrent::run([token, requested_enabled]() {
    SmartVoiceUiResult result;
    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<sweeper_mcp::SetSmartVoice>(
        "/sweeper_ai/set_smart_voice", false);
    if (!client.waitForExistence(ros::Duration(1.0)))
    {
      result.message = QStringLiteral("/sweeper_ai/set_smart_voice 不在线");
      return result;
    }
    sweeper_mcp::SetSmartVoice call;
    call.request.session_token = token;
    call.request.enabled = requested_enabled;
    if (!client.call(call))
    {
      result.message = QStringLiteral("智能语音服务调用失败");
      return result;
    }
    result.transport_ok = true;
    result.accepted = call.response.accepted;
    result.session_id = QString::fromStdString(call.response.session_id);
    result.message = QString::fromStdString(call.response.message);
    return result;
  }));
}

void MainWindow::toggleAiParseAuthorization()
{
  const TelemetrySnapshot data = snapshot();
  const bool enabled = data.ai_status_received && data.ai_status.parse_authorized;
  if (!enabled)
  {
    const auto answer = QMessageBox::question(
        this, QStringLiteral("确认云端语义解析授权"),
        QStringLiteral("手工或 ASR 文本将发送给配置的 DeepSeek 云端模型。"
                       "执行过程中必要的成功/失败、任务状态和位姿结果会脱敏回传，"
                       "密钥、日志路径和设备标识不会回传。是否确认？"),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (answer != QMessageBox::Yes)
      return;
  }
  requestAiAuthorization("parse", !enabled);
}

void MainWindow::toggleAiControlAuthorization()
{
  const TelemetrySnapshot data = snapshot();
  const bool enabled = data.ai_status_received && data.ai_status.control_authorized;
  if (!enabled)
  {
    if (!data.ai_status_received || !data.ai_status.parse_authorized)
    {
      QMessageBox::information(this, QStringLiteral("尚不能授权控制"),
                               QStringLiteral("请先确认 AI 语义解析授权。"));
      return;
    }
    const auto answer = QMessageBox::question(
        this, QStringLiteral("确认 AI 控制授权"),
        QStringLiteral("校验通过的计划将不再逐步弹窗，而是自动按顺序调用 MCP 工具。"
                       "能力包括车体相对导航、map 绝对导航、视觉定点清扫和已保存区域"
                       "覆盖清扫；仍受全部现有安全门约束。关闭本授权或退出 Qt 会取消"
                       "AI 所有的活动任务和剩余步骤。是否确认？"),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (answer != QMessageBox::Yes)
      return;
  }
  requestAiAuthorization("control", !enabled);
}

void MainWindow::requestAiAuthorization(const std::string& gate, bool enabled,
                                        bool heartbeat)
{
  if (!master_online_ || !ros_interfaces_ready_ || ai_session_token_.isEmpty())
  {
    if (!heartbeat)
      QMessageBox::warning(this, QStringLiteral("AI 后端未连接"),
                           QStringLiteral("ROS 或 NVIDIA AI 会话令牌未就绪。"));
    return;
  }
  if (heartbeat ? ai_heartbeat_pending_ : ai_authorization_request_pending_)
    return;
  if (heartbeat)
    ai_heartbeat_pending_ = true;
  else
    ai_authorization_request_pending_ = true;
  const std::string token = ai_session_token_.toStdString();
  auto* watcher = new QFutureWatcher<AiAuthorizationUiResult>(this);
  connect(watcher, &QFutureWatcher<AiAuthorizationUiResult>::finished, this,
          [this, watcher, heartbeat, gate]() {
            const AiAuthorizationUiResult result = watcher->result();
            if (heartbeat)
              ai_heartbeat_pending_ = false;
            else
              ai_authorization_request_pending_ = false;
            if (!heartbeat)
            {
              const QString label = QString::fromStdString(gate);
              const QString text = (result.transport_ok && result.accepted)
                                       ? result.message
                                       : QStringLiteral("授权请求失败：") + result.message;
              if (ai_events_)
                ai_events_->appendPlainText(
                    QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss  ")) +
                    label + QStringLiteral(" · ") + text);
              if (!result.transport_ok || !result.accepted)
                QMessageBox::warning(this, QStringLiteral("AI 授权未变更"), text);
            }
            watcher->deleteLater();
          });
  watcher->setFuture(QtConcurrent::run([token, gate, enabled]() {
    AiAuthorizationUiResult result;
    ros::NodeHandle node;
    ros::ServiceClient client =
        node.serviceClient<sweeper_mcp::SetAiAuthorization>(
            "/sweeper_ai/set_authorization", false);
    if (!client.waitForExistence(ros::Duration(1.0)))
    {
      result.message = QStringLiteral("/sweeper_ai/set_authorization 不在线");
      return result;
    }
    sweeper_mcp::SetAiAuthorization call;
    call.request.session_token = token;
    call.request.gate = gate;
    call.request.enabled = enabled;
    if (!client.call(call))
    {
      result.message = QStringLiteral("AI 授权服务调用失败");
      return result;
    }
    result.transport_ok = true;
    result.accepted = call.response.accepted;
    result.voice_authorized = call.response.voice_authorized;
    result.parse_authorized = call.response.parse_authorized;
    result.control_authorized = call.response.control_authorized;
    result.message = QString::fromStdString(call.response.message);
    return result;
  }));
}

void MainWindow::sendAiHeartbeat()
{
  if (!master_online_ || !ros_interfaces_ready_ || ai_session_token_.isEmpty())
    return;
  requestAiAuthorization("heartbeat", true, true);
}

void MainWindow::submitAiManualText()
{
  if (ai_submit_pending_ || !ai_manual_input_)
    return;
  const QString text = ai_manual_input_->toPlainText().trimmed();
  if (text.isEmpty())
  {
    QMessageBox::information(this, QStringLiteral("没有输入"),
                             QStringLiteral("请先输入需要 AI 分解的文本。"));
    return;
  }
  const TelemetrySnapshot data = snapshot();
  if (!data.ai_status_received || !data.ai_status.parse_authorized)
  {
    QMessageBox::information(this, QStringLiteral("解析授权关闭"),
                             QStringLiteral("请先确认 AI 语义解析授权。"));
    return;
  }
  if (data.ai_status.task_active)
  {
    QMessageBox::information(this, QStringLiteral("AI 正在处理"),
                             QStringLiteral("请等待当前请求结束或点击停止 AI 任务。"));
    return;
  }
  ai_submit_pending_ = true;
  ai_plan_table_->setRowCount(0);
  ai_final_output_->clear();
  ai_transcript_->setPlainText(text);
  const std::string token = ai_session_token_.toStdString();
  const std::string request_text = text.toStdString();
  auto* watcher = new QFutureWatcher<AiSubmitUiResult>(this);
  connect(watcher, &QFutureWatcher<AiSubmitUiResult>::finished, this,
          [this, watcher]() {
            const AiSubmitUiResult result = watcher->result();
            ai_submit_pending_ = false;
            if (!result.transport_ok || !result.accepted)
            {
              const QString error = QStringLiteral("AI 请求未接收：") + result.message;
              ai_final_output_->setPlainText(error);
              QMessageBox::warning(this, QStringLiteral("AI 请求失败"), error);
            }
            else if (ai_events_)
            {
              ai_events_->appendPlainText(
                  QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss  ")) +
                  QStringLiteral("请求已接收 · ") + result.request_id);
            }
            watcher->deleteLater();
          });
  watcher->setFuture(QtConcurrent::run([token, request_text]() {
    AiSubmitUiResult result;
    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<sweeper_mcp::SubmitAiText>(
        "/sweeper_ai/submit_text", false);
    if (!client.waitForExistence(ros::Duration(1.0)))
    {
      result.message = QStringLiteral("/sweeper_ai/submit_text 不在线");
      return result;
    }
    sweeper_mcp::SubmitAiText call;
    call.request.session_token = token;
    call.request.source = "MANUAL";
    call.request.text = request_text;
    if (!client.call(call))
    {
      result.message = QStringLiteral("AI 文本服务调用失败");
      return result;
    }
    result.transport_ok = true;
    result.accepted = call.response.accepted;
    result.request_id = QString::fromStdString(call.response.request_id);
    result.message = QString::fromStdString(call.response.message);
    return result;
  }));
}

void MainWindow::cancelAiTask()
{
  if (ai_cancel_pending_ || ai_session_token_.isEmpty())
    return;
  ai_cancel_pending_ = true;
  const TelemetrySnapshot data = snapshot();
  const std::string token = ai_session_token_.toStdString();
  const std::string request_id = data.ai_status.request_id;
  auto* watcher = new QFutureWatcher<AiCancelUiResult>(this);
  connect(watcher, &QFutureWatcher<AiCancelUiResult>::finished, this,
          [this, watcher]() {
            const AiCancelUiResult result = watcher->result();
            ai_cancel_pending_ = false;
            if (ai_events_)
              ai_events_->appendPlainText(
                  QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss  ")) +
                  (result.accepted ? QStringLiteral("停止请求 · ")
                                   : QStringLiteral("停止失败 · ")) +
                  result.message);
            if (!result.transport_ok || !result.accepted)
              QMessageBox::warning(this, QStringLiteral("AI 任务未停止"),
                                   result.message);
            watcher->deleteLater();
          });
  watcher->setFuture(QtConcurrent::run([token, request_id]() {
    AiCancelUiResult result;
    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<sweeper_mcp::CancelAiTask>(
        "/sweeper_ai/cancel_task", false);
    if (!client.waitForExistence(ros::Duration(1.0)))
    {
      result.message = QStringLiteral("/sweeper_ai/cancel_task 不在线");
      return result;
    }
    sweeper_mcp::CancelAiTask call;
    call.request.session_token = token;
    call.request.request_id = request_id;
    if (!client.call(call))
    {
      result.message = QStringLiteral("AI 停止服务调用失败");
      return result;
    }
    result.transport_ok = true;
    result.accepted = call.response.accepted;
    result.message = QString::fromStdString(call.response.message);
    return result;
  }));
}

void MainWindow::clearAiDisplay()
{
  if (ai_transcript_)
    ai_transcript_->clear();
  if (ai_final_output_)
    ai_final_output_->clear();
  if (ai_events_)
    ai_events_->clear();
  if (ai_plan_table_)
    ai_plan_table_->setRowCount(0);
}

void MainWindow::applyAiEvent(const sweeper_mcp::AiEvent& event)
{
  const QString time_text = event.header.stamp.isZero()
                                ? QDateTime::currentDateTime().toString(
                                      QStringLiteral("HH:mm:ss"))
                                : QDateTime::fromMSecsSinceEpoch(
                                      static_cast<qint64>(
                                          event.header.stamp.toSec() * 1000.0))
                                      .toString(QStringLiteral("HH:mm:ss"));
  const QString kind = QString::fromStdString(event.kind);
  const QString state = QString::fromStdString(event.state);
  const QString description = QString::fromStdString(event.description);
  if (kind == QStringLiteral("TRANSCRIPT") && ai_transcript_)
    ai_transcript_->setPlainText(description);
  if (kind == QStringLiteral("PLAN") && ai_plan_table_)
  {
    QJsonParseError error;
    const QJsonDocument document = QJsonDocument::fromJson(
        QByteArray::fromStdString(event.result_json), &error);
    if (error.error == QJsonParseError::NoError && document.isObject())
    {
      const QJsonArray steps = document.object().value(
          QStringLiteral("steps")).toArray();
      ai_plan_table_->setRowCount(steps.size());
      for (int row = 0; row < steps.size(); ++row)
      {
        const QJsonObject step = steps.at(row).toObject();
        const QString arguments = QString::fromUtf8(
            QJsonDocument(step.value(QStringLiteral("arguments")).toObject())
                .toJson(QJsonDocument::Compact));
        const QStringList columns = {
          QString::number(row + 1),
          step.value(QStringLiteral("description")).toString(),
          step.value(QStringLiteral("tool")).toString(),
          arguments,
          QStringLiteral("已校验"),
          QStringLiteral("--"),
        };
        for (int column = 0; column < columns.size(); ++column)
          ai_plan_table_->setItem(row, column,
                                  new QTableWidgetItem(columns.at(column)));
      }
    }
  }
  if (kind == QStringLiteral("STEP") && ai_plan_table_ &&
      event.step_index >= 0 && event.step_index < ai_plan_table_->rowCount())
  {
    const int row = event.step_index;
    if (!ai_plan_table_->item(row, 4))
      ai_plan_table_->setItem(row, 4, new QTableWidgetItem());
    ai_plan_table_->item(row, 4)->setText(
        state == QStringLiteral("RUNNING") ? QStringLiteral("执行中")
        : (state == QStringLiteral("SUCCEEDED") ? QStringLiteral("完成")
                                                 : QStringLiteral("失败")));
    ai_plan_table_->item(row, 4)->setToolTip(
        QString::fromStdString(event.result_json));
    if (!ai_plan_table_->item(row, 5))
      ai_plan_table_->setItem(row, 5, new QTableWidgetItem());
    ai_plan_table_->item(row, 5)->setText(
        event.duration_ms > 0.0
            ? QStringLiteral("%1 ms").arg(event.duration_ms, 0, 'f', 0)
            : QStringLiteral("--"));
  }
  if (kind == QStringLiteral("FINAL") && ai_final_output_)
    ai_final_output_->setPlainText(description);
  if (ai_events_)
  {
    QString summary = time_text + QStringLiteral("  ") + kind;
    if (!state.isEmpty())
      summary += QStringLiteral(" · ") + state;
    if (!description.isEmpty())
      summary += QStringLiteral(" · ") + description.left(220);
    ai_events_->appendPlainText(summary);
  }
}

void MainWindow::refreshAiUi(const TelemetrySnapshot& data)
{
  std::deque<sweeper_mcp::AiEvent> pending;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    pending.swap(ai_event_queue_);
  }
  for (const sweeper_mcp::AiEvent& event : pending)
    applyAiEvent(event);

  const bool fresh = data.ai_status_received &&
                     wallAge(data.ai_status_received_at) <= 2.5;
  const sweeper_mcp::AiControlStatus& status = data.ai_status;
  values_["ai_backend"]->setText(
      fresh ? QString::fromStdString(status.backend) +
                  (status.ui_session_alive ? QStringLiteral(" · 在线 · 会话正常")
                                           : QStringLiteral(" · 在线 · 等待心跳"))
            : QStringLiteral("离线"));
  values_["ai_model"]->setText(
      fresh ? QString::fromStdString(status.model) +
                  (status.cloud_configured ? QStringLiteral(" · 已配置")
                                           : QStringLiteral(" · 未配置"))
            : QStringLiteral("--"));
  values_["ai_phase"]->setText(
      fresh ? QString::fromStdString(status.phase) : QStringLiteral("--"));
  values_["ai_asr_model"]->setText(
      fresh && !status.asr_model.empty()
          ? (QString::fromStdString(status.asr_model) == QStringLiteral("large")
                 ? QStringLiteral("large（large-v3）")
                 : QString::fromStdString(status.asr_model)) +
                (status.asr_model_loaded ? QStringLiteral(" · 已加载")
                                         : QStringLiteral(" · 未加载"))
          : QStringLiteral("--"));
  values_["ai_asr_device"]->setText(
      fresh && !status.asr_device.empty()
          ? QString::fromStdString(status.asr_device)
          : QStringLiteral("--"));
  values_["ai_asr_phase"]->setText(
      fresh ? (status.asr_available ? QString::fromStdString(status.asr_phase)
                                    : QStringLiteral("不可用 · ") +
                                          QString::fromStdString(status.asr_phase))
            : QStringLiteral("--"));
  values_["ai_asr_audio_duration"]->setText(
      fresh && status.asr_audio_duration_s > 0.0
          ? QString::number(status.asr_audio_duration_s, 'f', 1)
          : QStringLiteral("--"));
  values_["ai_asr_latency"]->setText(
      fresh && status.asr_latency_ms > 0.0
          ? QString::number(status.asr_latency_ms, 'f', 0)
          : QStringLiteral("--"));
  values_["ai_asr_error"]->setText(
      fresh && !status.asr_last_error.empty()
          ? QString::fromStdString(status.asr_last_error)
          : QStringLiteral("无"));
  values_["ai_smart_voice_mode"]->setText(
      fresh ? (status.smart_voice_enabled
                   ? QStringLiteral("智能连续监听 · 自动断句")
                   : QStringLiteral("关闭 · 手动按键录音"))
            : QStringLiteral("--"));
  values_["ai_smart_voice_listening"]->setText(
      fresh ? (status.smart_voice_listening
                   ? QStringLiteral("正在监听麦克风")
                   : (status.smart_voice_enabled
                          ? QStringLiteral("等待语音 / 处理中")
                          : QStringLiteral("未监听")))
            : QStringLiteral("--"));
  values_["ai_smart_voice_utterances"]->setText(
      fresh ? QString::number(status.smart_voice_utterance_count)
            : QStringLiteral("--"));
  values_["ai_smart_voice_pending"]->setText(
      fresh ? QString::number(status.smart_voice_pending_count)
            : QStringLiteral("--"));
  values_["ai_cloud_rtt"]->setText(
      fresh && status.last_cloud_rtt_ms > 0.0
          ? QString::number(status.last_cloud_rtt_ms, 'f', 0)
          : QStringLiteral("--"));
  values_["ai_total_latency"]->setText(
      fresh && status.last_total_latency_ms > 0.0
          ? QString::number(status.last_total_latency_ms, 'f', 0)
          : QStringLiteral("--"));
  values_["ai_http"]->setText(
      fresh && status.last_http_status > 0
          ? QString::number(status.last_http_status)
          : QStringLiteral("--"));
  values_["ai_progress"]->setText(
      fresh ? QStringLiteral("%1 / %2").arg(status.current_step).arg(status.total_steps)
            : QStringLiteral("--"));
  values_["ai_request_id"]->setText(
      fresh && !status.request_id.empty()
          ? QString::fromStdString(status.request_id).left(12)
          : QStringLiteral("--"));
  values_["ai_error"]->setText(
      fresh && !status.last_error.empty()
          ? QString::fromStdString(status.last_error)
          : QStringLiteral("无"));
  if (fresh && !status.final_text.empty() && ai_final_output_ &&
      ai_final_output_->toPlainText() != QString::fromStdString(status.final_text))
    ai_final_output_->setPlainText(QString::fromStdString(status.final_text));

  const bool service_ready = fresh && status.ui_session_alive &&
                             !ai_session_token_.isEmpty();
  ai_voice_auth_button_->setText(
      fresh && status.voice_authorized
          ? QStringLiteral("关闭语音输入授权")
          : QStringLiteral("授权语音输入"));
  ai_parse_auth_button_->setText(
      fresh && status.parse_authorized
          ? QStringLiteral("关闭 AI 语义解析")
          : QStringLiteral("授权 AI 语义解析"));
  ai_control_auth_button_->setText(
      fresh && status.control_authorized
          ? QStringLiteral("撤销 AI 控制并停止任务")
          : QStringLiteral("授权 AI 控制"));
  ai_smart_voice_button_->setText(
      fresh && (status.smart_voice_enabled || status.smart_voice_listening)
          ? QStringLiteral("关闭智能语音")
          : QStringLiteral("启用智能语音"));
  const QString asr_phase = QString::fromStdString(status.asr_phase).toUpper();
  const bool asr_busy = asr_phase == QStringLiteral("LOADING") ||
                        asr_phase == QStringLiteral("TRANSCRIBING") ||
                        asr_phase == QStringLiteral("RECOGNIZING") ||
                        asr_phase == QStringLiteral("CANCELLING") ||
                        asr_phase == QStringLiteral("STARTING") ||
                        asr_phase == QStringLiteral("STOPPING") ||
                        asr_phase == QStringLiteral("SWITCHING");
  ai_asr_record_button_->setText(
      fresh && (status.asr_recording ||
                asr_phase == QStringLiteral("RECORDED"))
          ? QStringLiteral("停止并识别")
          : QStringLiteral("开始录音"));
  const bool asr_can_start =
      service_ready && status.voice_authorized && status.asr_available &&
      !status.asr_recording && asr_phase != QStringLiteral("RECORDED") &&
      !asr_busy;
  const bool asr_can_stop =
      service_ready && status.voice_authorized && status.asr_available &&
      !asr_busy &&
      (status.asr_recording || asr_phase == QStringLiteral("RECORDED"));
  const bool smart_voice_active = status.smart_voice_enabled ||
                                  status.smart_voice_listening;
  ai_asr_record_button_->setEnabled(
      !ai_asr_request_pending_ && !ai_smart_voice_request_pending_ &&
      !smart_voice_active && (asr_can_start || asr_can_stop));
  const bool manual_asr_active = status.asr_recording ||
                                 asr_phase == QStringLiteral("RECORDED") ||
                                 asr_busy || ai_asr_request_pending_;
  if (ai_asr_model_combo_)
  {
    QString active_model = QString::fromStdString(status.asr_model)
                               .trimmed().toLower();
    if (active_model == QStringLiteral("large-v3"))
      active_model = QStringLiteral("large");
    if (fresh && !ai_asr_model_request_pending_)
    {
      const int active_index = ai_asr_model_combo_->findData(active_model);
      if (active_index >= 0 &&
          active_index != ai_asr_model_combo_->currentIndex())
      {
        const QSignalBlocker blocker(ai_asr_model_combo_);
        ai_asr_model_combo_->setCurrentIndex(active_index);
      }
    }
    ai_asr_model_combo_->setEnabled(
        service_ready && !ai_asr_model_request_pending_ &&
        !ai_asr_request_pending_ && !ai_smart_voice_request_pending_ &&
        !manual_asr_active && !smart_voice_active);
  }
  const bool smart_voice_can_enable =
      service_ready && status.voice_authorized && status.asr_available &&
      !status.smart_voice_enabled && !status.smart_voice_listening &&
      !manual_asr_active;
  const bool smart_voice_can_disable =
      service_ready && smart_voice_active;
  ai_smart_voice_button_->setEnabled(
      !ai_smart_voice_request_pending_ &&
      (smart_voice_can_enable || smart_voice_can_disable));
  ai_voice_auth_button_->setEnabled(service_ready &&
                                     !ai_authorization_request_pending_);
  ai_parse_auth_button_->setEnabled(service_ready && status.cloud_configured &&
                                     !ai_authorization_request_pending_);
  ai_control_auth_button_->setEnabled(
      service_ready && status.parse_authorized &&
      !ai_authorization_request_pending_);
  ai_submit_button_->setEnabled(
      service_ready && status.parse_authorized && !status.task_active &&
      !ai_submit_pending_ && !ai_manual_input_->toPlainText().trimmed().isEmpty());
  ai_cancel_button_->setEnabled(
      fresh && status.task_active && !ai_cancel_pending_);
  ai_clear_button_->setEnabled(!ai_submit_pending_);
}

}  // namespace autolabor_operator_gui
