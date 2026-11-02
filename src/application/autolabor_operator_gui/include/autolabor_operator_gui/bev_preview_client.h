#ifndef AUTOLABOR_OPERATOR_GUI_BEV_PREVIEW_CLIENT_H
#define AUTOLABOR_OPERATOR_GUI_BEV_PREVIEW_CLIENT_H
#include <QObject>
#include <QImage>
#include <QJsonObject>
#include <QMap>
#include <QNetworkAccessManager>
#include <QPointer>
#include <QTimer>
#include <QUrl>
class QNetworkReply;
namespace autolabor_operator_gui
{
struct BevPreviewFrame
{
  QImage raw;
  QImage annotated;
  QMap<QString, QImage> camera_images;
  QJsonObject metadata;
};
bool decodeBevPreview(const QByteArray& packet, BevPreviewFrame* frame, QString* error);
class BevPreviewClient : public QObject
{
  Q_OBJECT
public:
  explicit BevPreviewClient(const QUrl& url, QObject* parent = nullptr);
  const BevPreviewFrame& frame() const { return frame_; }
  QString status() const { return status_; }
signals:
  void changed();
private:
  void poll();
  void clear(const QString& message);
  QUrl url_;
  QNetworkAccessManager manager_;
  QPointer<QNetworkReply> pending_;
  QTimer timer_;
  BevPreviewFrame frame_;
  QString status_;
};
}
#endif
