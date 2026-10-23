#include <autolabor_operator_gui/surround_preview_client.h>
#include <gtest/gtest.h>
#include <QApplication>
#include <QBuffer>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTcpServer>
#include <QTcpSocket>
#include <QThread>
#include <QtEndian>
#include <functional>

using namespace autolabor_operator_gui;
namespace
{
QJsonObject metadata(int port=0, int sequence=1)
{
  QJsonObject source{{"port", port}, {"session", "camera-session"}, {"sequence", sequence},
      {"raw_stamp", QString::number(100000+sequence)}, {"hal_received_ns", "200000"},
      {"frame_id", QStringLiteral("surround_rx1_port%1_rectified_optical").arg(port)}};
  return QJsonObject{{"schema", "j6m.surround.operator.frame.v1"}, {"source", source},
      {"image_source", source}, {"inference_device", "J6M_BPU"},
      {"detector_sha256", "f2f1f7e9753b53b0494495be44af92a0603b3ee6c71e69e0f58a0d50544a8cb8"},
      {"classifier_sha256", "d4cb80f5f7364d1156ea2275eba9e522991dec309f2c397e74649254fe36a89f"},
      {"width",960}, {"height",540}, {"encoding","jpeg_display_copy"},
      {"gdc_applied",true}, {"lut_sha256",QString(64,'a')}, {"production_ready",false},
      {"motion_eligible",false}, {"source_clock_verified",false}, {"accuracy_verified",false},
      {"read_age_ms",20}, {"read_to_result_ms",10}, {"inference_fps",5},
      {"timing",QJsonObject{{"processing_ms",10}}}, {"objects",QJsonArray()}};
}
QByteArray payload(QJsonObject meta)
{
  QImage image(960,540,QImage::Format_RGB32);
  image.fill(QColor(20+60*meta["source"].toObject()["port"].toInt(),50,80));
  QByteArray jpeg;
  QBuffer buffer(&jpeg); buffer.open(QIODevice::WriteOnly); image.save(&buffer,"JPEG");
  QByteArray json = QJsonDocument(meta).toJson(QJsonDocument::Compact);
  QByteArray result(12,0); result.replace(0,4,"SG01");
  qToBigEndian<quint32>(json.size(), reinterpret_cast<uchar*>(result.data()+4));
  qToBigEndian<quint32>(jpeg.size(), reinterpret_cast<uchar*>(result.data()+8));
  return result+json+jpeg;
}
bool spin(const std::function<bool()>& predicate, int timeout=1500)
{
  QElapsedTimer time; time.start();
  while (!predicate() && time.elapsed()<timeout)
  {
    QApplication::processEvents(); QThread::msleep(2);
  }
  return predicate();
}
class Server : public QTcpServer
{
public:
  bool available = true;
  int delay0 = 0;
  int sequence = 1;
  QList<int> ports;
  Server()
  {
    listen(QHostAddress::LocalHost,0);
    connect(this,&QTcpServer::newConnection,this,[this]() {
      auto* socket=nextPendingConnection();
      connect(socket,&QTcpSocket::disconnected,socket,&QObject::deleteLater);
      connect(socket,&QTcpSocket::readyRead,this,[this,socket]() {
        const auto request=socket->readAll();
        if (!request.contains("\r\n\r\n")) return;
        int port=request.mid(request.indexOf("/frame/")+7,1).toInt();
        ports.append(port);
        const auto body=payload(metadata(port,sequence++));
        QTimer::singleShot(port==0 ? delay0 : 0,socket,[this,socket,body]() {
          if (socket->state()!=QAbstractSocket::ConnectedState) return;
          const QByteArray data=available ? body : QByteArray("unavailable");
          socket->write(QByteArray(available ? "HTTP/1.1 200 OK\r\n" : "HTTP/1.1 503 Unavailable\r\n")+
              "Content-Length: "+QByteArray::number(data.size())+"\r\nConnection: close\r\n\r\n"+data);
          socket->disconnectFromHost();
        });
      });
    });
  }
};
}

TEST(SurroundPreview, DecodesPairedGdcResult)
{
  SurroundPreviewFrame frame; QString error;
  ASSERT_TRUE(decodeSurroundPreview(payload(metadata()),0,&frame,&error)) << error.toStdString();
  EXPECT_EQ(frame.annotated.size(), QSize(960,540));
  EXPECT_TRUE(frame.metadata["objects"].toArray().isEmpty());
}
TEST(SurroundPreview, RejectsWrongPortAndMixedFrame)
{
  SurroundPreviewFrame frame; QString error;
  EXPECT_FALSE(decodeSurroundPreview(payload(metadata(0)),1,&frame,&error));
  auto meta=metadata(); meta["image_source"]=metadata(1)["source"];
  EXPECT_FALSE(decodeSurroundPreview(payload(meta),0,&frame,&error));
}
TEST(SurroundPreview, RejectsStaleUnknownModelsAndFakeReadiness)
{
  SurroundPreviewFrame frame; QString error;
  for (const QString& key : {QString("read_age_ms"),QString("detector_sha256"),QString("gdc_applied"),QString("motion_eligible")})
  {
    auto meta=metadata();
    if (key=="read_age_ms") meta[key]=501;
    else if (key=="detector_sha256") meta[key]="wrong";
    else meta[key]=key=="motion_eligible";
    EXPECT_FALSE(decodeSurroundPreview(payload(meta),0,&frame,&error)) << key.toStdString();
  }
}
TEST(SurroundPreview, RejectsMalformedBoxAndPacket)
{
  SurroundPreviewFrame frame; QString error;
  auto meta=metadata();
  meta["objects"]=QJsonArray{QJsonObject{{"bbox",QJsonArray{0,0,1000,100}},
      {"class_id",0},{"detect_confidence",.8},{"probabilities",QJsonValue()},{"material","unknown"}}};
  EXPECT_FALSE(decodeSurroundPreview(payload(meta),0,&frame,&error));
  EXPECT_FALSE(decodeSurroundPreview(payload(metadata()).chopped(1),0,&frame,&error));
}
TEST(SurroundPreview, SwitchDiscardsLateReplyAndLossClearsImage)
{
  Server server; server.delay0=200;
  SurroundPreviewClient client(QUrl(QStringLiteral("http://127.0.0.1:%1").arg(server.serverPort())));
  ASSERT_TRUE(spin([&](){ return server.ports.contains(0); }));
  client.selectPort(1);
  EXPECT_FALSE(client.live());
  bool wrong=false;
  QObject::connect(&client,&SurroundPreviewClient::changed,[&]() {
    if (client.live() && client.frame().metadata["source"].toObject()["port"].toInt()!=client.port()) wrong=true;
  });
  ASSERT_TRUE(spin([&](){ return client.live(); }));
  EXPECT_EQ(client.frame().metadata["source"].toObject()["port"].toInt(),1);
  server.available=false;
  ASSERT_TRUE(spin([&](){ return !client.live(); }));
  server.available=true;
  client.selectPort(2);
  ASSERT_TRUE(spin([&](){ return client.live(); }));
  EXPECT_EQ(client.frame().metadata["source"].toObject()["port"].toInt(),2);
  EXPECT_FALSE(wrong);
  EXPECT_TRUE(server.ports.contains(2));
}

int main(int argc,char** argv)
{
  QApplication app(argc,argv);
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
