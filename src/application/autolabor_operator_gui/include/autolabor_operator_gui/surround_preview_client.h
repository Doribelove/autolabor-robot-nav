#ifndef AUTOLABOR_OPERATOR_GUI_SURROUND_PREVIEW_CLIENT_H
#define AUTOLABOR_OPERATOR_GUI_SURROUND_PREVIEW_CLIENT_H

#include <QElapsedTimer>
#include <QImage>
#include <QJsonObject>
#include <QNetworkAccessManager>
#include <QPointer>
#include <QSet>
#include <QTimer>
#include <QUrl>
#include <array>

class QNetworkReply;

namespace autolabor_operator_gui
{
struct SurroundPreviewFrame
{
  QImage raw;
  QImage annotated;
  QJsonObject metadata;
};

// One atomic response contains the inference result and its exact source image.
bool decodeSurroundPreview(const QByteArray& packet, int expected_port,
                          SurroundPreviewFrame* frame, QString* error);

class SurroundPreviewClient : public QObject
{
  Q_OBJECT
public:
  explicit SurroundPreviewClient(const QUrl& url, QObject* parent = nullptr);
  void selectPort(int port);
  int port() const { return port_; }
  const SurroundPreviewFrame& frame() const { return frame_; }
  QString status() const { return status_; }
  bool live() const { return !frame_.annotated.isNull(); }

signals:
  void changed();

private:
  void poll();
  void clear(const QString& message);
  bool acceptSource(const QJsonObject& source);
  QUrl url_;
  QNetworkAccessManager manager_;
  QPointer<QNetworkReply> pending_;
  QTimer timer_;
  QElapsedTimer received_;
  double read_age_ms_ = 0;
  int port_ = 0;
  unsigned generation_ = 0;
  SurroundPreviewFrame frame_;
  QString status_;
  std::array<QJsonObject, 3> sources_;
  std::array<QSet<QString>, 3> retired_;
};
}  // namespace autolabor_operator_gui
#endif
