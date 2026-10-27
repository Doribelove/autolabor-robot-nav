#include <autolabor_operator_gui/bev_preview_client.h>
#include <gtest/gtest.h>
#include <QApplication>
#include <QBuffer>
#include <QJsonArray>
#include <QJsonDocument>
#include <QtEndian>
using namespace autolabor_operator_gui;
namespace
{
QJsonObject metadata()
{
  const QJsonObject id{{"session", "bev-session"}, {"sequence", 7}};
  return QJsonObject{{"schema", "conbine.bev.yolo.frame.v1"}, {"source_id", id},
    {"image_source_id", id}, {"result_source_id", id}, {"width", 600}, {"height", 600},
    {"backend", "CUDA"}, {"model_name", "YOLO11-GAM / best6.pt"},
    {"model_sha256", QString(64, 'a')}, {"produced_epoch_s", 1792900000.},
    {"inference_ms", 42.}, {"inference_fps", 5.}, {"objects", QJsonArray()}};
}
QByteArray packet(const QJsonObject& meta)
{
  QImage raw(600, 600, QImage::Format_RGB32); raw.fill(Qt::darkBlue);
  QImage result = raw; result.setPixelColor(250, 250, Qt::green);
  QByteArray r, a;
  QBuffer rb(&r), ab(&a); rb.open(QIODevice::WriteOnly); ab.open(QIODevice::WriteOnly);
  raw.save(&rb, "JPEG"); result.save(&ab, "JPEG");
  auto json = QJsonDocument(meta).toJson(QJsonDocument::Compact);
  QByteArray body(16, 0); body.replace(0, 4, "BV01");
  qToBigEndian<quint32>(json.size(), reinterpret_cast<uchar*>(body.data()+4));
  qToBigEndian<quint32>(r.size(), reinterpret_cast<uchar*>(body.data()+8));
  qToBigEndian<quint32>(a.size(), reinterpret_cast<uchar*>(body.data()+12));
  return body+json+r+a;
}
}
TEST(BevPreview, DecodesActualRawAndAnnotatedPairEvenWithNoDetections)
{
  BevPreviewFrame frame; QString error;
  ASSERT_TRUE(decodeBevPreview(packet(metadata()), &frame, &error)) << error.toStdString();
  EXPECT_EQ(frame.raw.size(), QSize(600, 600));
  EXPECT_EQ(frame.annotated.size(), QSize(600, 600));
  EXPECT_TRUE(frame.metadata["objects"].toArray().isEmpty());
}
TEST(BevPreview, RejectsOverlayFromDifferentFusionFrame)
{
  auto meta = metadata();
  meta["result_source_id"] = QJsonObject{{"session", "bev-session"}, {"sequence", 8}};
  BevPreviewFrame frame; QString error;
  EXPECT_FALSE(decodeBevPreview(packet(meta), &frame, &error));
}
TEST(BevPreview, AcceptsBoundedDetectionOnPairedImage)
{
  auto meta = metadata();
  meta["objects"] = QJsonArray{QJsonObject{{"class_name", "paper"}, {"confidence", .8},
      {"bbox", QJsonArray{10, 20, 60, 50}}}};
  BevPreviewFrame frame; QString error;
  ASSERT_TRUE(decodeBevPreview(packet(meta), &frame, &error)) << error.toStdString();
  EXPECT_EQ(frame.metadata["objects"].toArray().size(), 1);
}
TEST(BevPreview, RejectsTruncatedImagePacket)
{
  auto p = packet(metadata()); p.chop(3);
  BevPreviewFrame frame; QString error;
  EXPECT_FALSE(decodeBevPreview(p, &frame, &error));
}
TEST(BevPreview, RejectsNonfiniteConfidenceAndOffImageBoxes)
{
  auto meta = metadata();
  meta["objects"] = QJsonArray{QJsonObject{{"class_name", "paper"}, {"confidence", .8},
      {"bbox", QJsonArray{10, 20, 601, 50}}}};
  BevPreviewFrame frame; QString error;
  EXPECT_FALSE(decodeBevPreview(packet(meta), &frame, &error));
  meta["objects"] = QJsonArray{QJsonObject{{"class_name", "paper"}, {"confidence", QJsonValue()},
      {"bbox", QJsonArray{10, 20, 60, 50}}}};
  EXPECT_FALSE(decodeBevPreview(packet(meta), &frame, &error));
}

int main(int argc, char** argv)
{
  QApplication app(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
