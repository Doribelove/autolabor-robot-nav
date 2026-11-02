#include <autolabor_operator_gui/bev_preview_client.h>
#include <gtest/gtest.h>
#include <QApplication>
#include <QBuffer>
#include <QCryptographicHash>
#include <QJsonArray>
#include <QJsonDocument>
#include <QFile>
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
QByteArray viewsPacket(bool mismatch=false, bool duplicate=false)
{
  auto meta=metadata();
  QJsonArray rows;
  QJsonObject sources,sizes;
  QByteArray images;
  int index=0;
  for (const QString name : {"front","left","right","rear"})
  {
    QImage image(960,540,QImage::Format_RGB32);image.fill(index++%2 ? Qt::blue : Qt::yellow);
    QByteArray data;QBuffer buffer(&data);buffer.open(QIODevice::WriteOnly);image.save(&buffer,"JPEG");
    auto id=meta["source_id"].toObject();
    if (mismatch && name=="rear") id["sequence"]=8;
    rows.append(QJsonObject{{"camera",duplicate && name=="rear" ? QString("front") : name},
      {"width",960},{"height",540},{"bytes",data.size()},{"source_id",id},
      {"sha256",QString::fromLatin1(QCryptographicHash::hash(data,QCryptographicHash::Sha256).toHex())}});
    images+=data;
    sources[name]=QJsonObject{{"sequence",2}};sizes[name]=QJsonArray{960,540};
  }
  meta["camera_previews"]=rows;
  meta["source_metadata"]=QJsonObject{{"sources",sources},{"camera_image_sizes",sizes}};
  auto result=packet(meta);result.replace(0,4,"BV02");return result+images;
}
}
TEST(BevPreview, DecodesFourOriginalViewsPairedWithTheBev)
{
  BevPreviewFrame frame;QString error;
  ASSERT_TRUE(decodeBevPreview(viewsPacket(),&frame,&error)) << error.toStdString();
  EXPECT_EQ(frame.camera_images.size(),4);
  EXPECT_EQ(frame.camera_images["rear"].size(),QSize(960,540));
  EXPECT_NE(frame.camera_images["front"].pixelColor(20,20),frame.camera_images["left"].pixelColor(20,20));
  ASSERT_TRUE(decodeBevPreview(packet(metadata()),&frame,&error));
  EXPECT_TRUE(frame.camera_images.isEmpty());
}
TEST(BevPreview, RejectsMixedOrRepeatedCameraFrames)
{
  BevPreviewFrame frame;QString error;
  EXPECT_FALSE(decodeBevPreview(viewsPacket(true),&frame,&error));
  EXPECT_FALSE(decodeBevPreview(viewsPacket(false,true),&frame,&error));
}
TEST(BevPreview, RejectsTruncatedOrCorruptOriginalViewPayload)
{
  BevPreviewFrame frame;QString error;
  auto truncated=viewsPacket();truncated.chop(1);
  EXPECT_FALSE(decodeBevPreview(truncated,&frame,&error));
  auto corrupt=viewsPacket();corrupt[corrupt.size()-10]=char(corrupt[corrupt.size()-10]^1);
  EXPECT_FALSE(decodeBevPreview(corrupt,&frame,&error));
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
TEST(BevPreview, PreservesRearEvidenceFromFourCameraFusion)
{
  auto meta = metadata();
  QJsonObject sources;
  for (const auto& name : {"front", "left", "right", "rear"})
    sources[name] = QJsonObject{{"sequence", 20}};
  meta["source_metadata"] = QJsonObject{{"sources", sources},
      {"configured_cameras", QJsonArray{"front", "left", "right", "rear"}}};
  BevPreviewFrame frame; QString error;
  ASSERT_TRUE(decodeBevPreview(packet(meta), &frame, &error)) << error.toStdString();
  EXPECT_EQ(frame.metadata["source_metadata"].toObject()["sources"].toObject().size(), 4);
  EXPECT_TRUE(frame.metadata["source_metadata"].toObject()["sources"].toObject().contains("rear"));
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
TEST(BevPreview, AcceptsActualBpuBackendAndRejectsUnknownBackend)
{
  auto meta = metadata();
  meta["backend"] = "BPU";
  meta["inference_image_space"] = "original_upright_cameras";
  BevPreviewFrame frame; QString error;
  ASSERT_TRUE(decodeBevPreview(packet(meta), &frame, &error)) << error.toStdString();
  EXPECT_EQ(frame.metadata["backend"].toString(), "BPU");
  meta["backend"] = "unknown";
  EXPECT_FALSE(decodeBevPreview(packet(meta), &frame, &error));
}
TEST(BevPreview, DecodesRecordedServerPacketWhenSupplied)
{
  const auto path = qgetenv("BEV_PREVIEW_TEST_PACKET");
  if (path.isEmpty()) return;
  QFile file(QString::fromUtf8(path));
  ASSERT_TRUE(file.open(QIODevice::ReadOnly));
  BevPreviewFrame frame; QString error;
  ASSERT_TRUE(decodeBevPreview(file.readAll(), &frame, &error)) << error.toStdString();
  const auto backend = qgetenv("BEV_PREVIEW_TEST_BACKEND");
  EXPECT_EQ(frame.metadata["backend"].toString(),
            backend.isEmpty() ? QStringLiteral("BPU") : QString::fromUtf8(backend));
  EXPECT_EQ(frame.metadata["inference_image_space"].toString(), "original_upright_cameras");
  EXPECT_EQ(frame.metadata["camera_detections"].toObject().size(), 4);
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
