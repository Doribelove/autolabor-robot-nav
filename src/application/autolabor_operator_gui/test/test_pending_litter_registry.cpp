#include <autolabor_operator_gui/pending_litter_registry.h>
#include <gtest/gtest.h>
#include <QFile>
#include <QTemporaryDir>
#include <cmath>
#include <limits>

namespace autolabor_operator_gui
{
namespace
{
const std::array<double,16> identity{{1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1}};
class TestRegistry : public PendingLitterRegistry
{
public:
  using PendingLitterRegistry::observe;
  bool observe(double stamp, const QVector<LitterObservation>& points)
  { return PendingLitterRegistry::observe(stamp, points, "test-session", identity, true, 0); }
};
LitterObservation at(double x = 1.0, double y = 2.0, const QString& category = "plastic")
{ return {{x, y}, category, {{x, y, 0}}, 20}; }
void confirm(TestRegistry& registry, double start, const LitterObservation& point)
{
  for (int i = 0; i < 5; ++i) registry.observe(start + i * 0.05, {point});
}

TEST(PendingLitterRegistry, ExactlyFiveDistinctFrames)
{
  TestRegistry registry;
  for (int i = 0; i < 4; ++i) EXPECT_FALSE(registry.observe(1 + i * 0.05, {at()}));
  EXPECT_TRUE(registry.points().empty());
  EXPECT_TRUE(registry.observe(1.20, {at()}));
  ASSERT_EQ(1, registry.points().size());
  EXPECT_EQ(QString("a"), registry.points()[0].label);
}
TEST(PendingLitterRegistry, DuplicateFramesAndBoxesAreOneVote)
{
  TestRegistry registry;
  for (int i = 0; i < 12; ++i)
    registry.observe(1, {at(), at(1.02), at(1.01), at(), at()});
  EXPECT_TRUE(registry.points().empty());
  for (int i = 1; i < 5; ++i) registry.observe(1 + i * 0.05, {at(), at(1.02)});
  EXPECT_EQ(1, registry.points().size());
}
TEST(PendingLitterRegistry, MissingFrameResetsOnlyMissingCandidate)
{
  TestRegistry registry;
  for (int i = 0; i < 4; ++i) registry.observe(1 + i * 0.05, {at(), at(3)});
  registry.observe(1.20, {at(3)});
  ASSERT_EQ(1, registry.points().size());
  EXPECT_DOUBLE_EQ(3.0, registry.points()[0].position.x());
  for (int i = 0; i < 4; ++i) registry.observe(1.25 + i * 0.05, {at()});
  EXPECT_EQ(1, registry.points().size());
  registry.observe(1.45, {at()});
  EXPECT_EQ(2, registry.points().size());
}
TEST(PendingLitterRegistry, InvalidAndLongGapBreakStreak)
{
  TestRegistry registry;
  for (int i = 0; i < 4; ++i) registry.observe(1 + i * 0.05, {at()});
  registry.observe(2, {at()});
  EXPECT_TRUE(registry.points().empty());
  registry.observe(2.05, {at(std::numeric_limits<double>::quiet_NaN())});
  for (int i = 0; i < 4; ++i) registry.observe(2.1 + i * 0.05, {at()});
  EXPECT_TRUE(registry.points().empty());
  registry.observe(2.3, {at()});
  EXPECT_EQ(1, registry.points().size());
}
TEST(PendingLitterRegistry, EuclideanRadiusIndependentOfCategory)
{
  TestRegistry registry;
  confirm(registry, 1, at());
  confirm(registry, 2, at(1.10, 2.10, "metal"));
  ASSERT_EQ(1, registry.points().size());
  confirm(registry, 3, at(1.1501, 2, "metal"));
  ASSERT_EQ(2, registry.points().size());
  confirm(registry, 4, at(1, 2.1501));
  ASSERT_EQ(3, registry.points().size());
}
TEST(PendingLitterRegistry, MedianCoordinatesAndCategoryVote)
{
  TestRegistry registry;
  const double xs[] = {1.00, 1.08, 0.99, 1.01, 1.02};
  const double ys[] = {2.00, 2.04, 2.02, 1.99, 2.01};
  for (int i = 0; i < 5; ++i)
    registry.observe(1 + i * 0.05, {at(xs[i], ys[i], i < 2 ? "metal" : "plastic")});
  ASSERT_EQ(1, registry.points().size());
  EXPECT_EQ(QPointF(1.01, 2.01), registry.points()[0].position);
  EXPECT_EQ(QString("plastic"), registry.points()[0].category);
}
TEST(PendingLitterRegistry, FixedAnchorPreventsChainedMergingAndDrift)
{
  TestRegistry registry;
  confirm(registry, 1, at());
  confirm(registry, 2, at(1.09));
  confirm(registry, 3, at(1.18));
  ASSERT_EQ(2, registry.points().size());
  EXPECT_DOUBLE_EQ(1, registry.points()[0].anchor[0]);
  EXPECT_EQ(QPointF(1.18, 2), registry.points()[1].position);
}
TEST(PendingLitterRegistry, PersistentMissesNeverDeleteConfirmedPoints)
{
  TestRegistry registry;
  confirm(registry, 1, at());
  for (int i = 0; i < 30; ++i) registry.observe(2 + i * 0.1, {});
  registry.breakContinuity();
  ASSERT_EQ(1, registry.points().size());
  confirm(registry, 9, at());
  EXPECT_EQ(1, registry.points().size());
}
TEST(PendingLitterRegistry, LabelsContinueAfterZAndResetByMap)
{
  TestRegistry registry;
  QVector<LitterObservation> observations;
  for (int i = 0; i < 28; ++i) observations.push_back(at(i));
  for (int i = 0; i < 5; ++i) registry.observe(1 + i * 0.05, observations);
  ASSERT_EQ(28, registry.points().size());
  EXPECT_EQ(QString("z"), registry.points()[25].label);
  EXPECT_EQ(QString("aa"), registry.points()[26].label);
  EXPECT_EQ(QString("ab"), registry.points()[27].label);
  registry.reset();
  confirm(registry, 1, at());
  EXPECT_EQ(QString("a"), registry.points()[0].label);
}
TEST(PendingLitterRegistry, ReorderedFramesDoNotConfirm)
{
  TestRegistry registry;
  registry.observe(2, {at()});
  for (int i = 0; i < 10; ++i) registry.observe(1 + i * 0.05, {at()});
  EXPECT_TRUE(registry.points().empty());
  for (int i = 1; i < 5; ++i) registry.observe(2 + i * 0.05, {at()});
  EXPECT_EQ(1, registry.points().size());
}
TEST(PendingLitterStore, RoundTripAndSeparateMaps)
{
  QTemporaryDir root;
  QVector<PendingLitterPoint> points;
  PendingLitterStore store;
  QString error;
  ASSERT_TRUE(store.open(root.path(), QString(64, 'a'), &points, &error));
  points.push_back({"a", QStringLiteral("塑料"), {-1.25, 4.56}});
  ASSERT_TRUE(store.save(points, &error)) << error.toStdString();
  PendingLitterStore restored;
  ASSERT_TRUE(restored.open(root.path(), QString(64, 'a'), &points, &error));
  ASSERT_EQ(1, points.size());
  EXPECT_EQ(QPointF(-1.25, 4.56), points[0].position);
  EXPECT_EQ(QStringLiteral("塑料"), points[0].category);
  TestRegistry registry;
  registry.reset(points);
  confirm(registry, 1, at());
  EXPECT_EQ(QString("b"), registry.points()[1].label);
  ASSERT_TRUE(restored.open(root.path(), QString(64, 'b'), &points, &error));
  EXPECT_TRUE(points.empty());
}
TEST(PendingLitterRegistry, GlobalJumpUpdatesSameLocalLandmarkWithoutDuplicate)
{
  TestRegistry registry;
  confirm(registry, 1, at());
  auto transform = identity;
  transform[7] = 1.8;
  for (int i = 0; i < 60; ++i)
  {
    auto observation = at(1 + .04*std::sin(i), 2 + .04*std::cos(i));
    observation.position += QPointF(0, 1.8);  // raw map jumps, odom does not.
    observation.object_id = i % 3 + 30;  // detector IDs are not permanent.
    registry.observe(2+i*.125, {observation}, "test-session", transform, i >= 48, 1);
    ASSERT_EQ(1, registry.points().size());
    if (i < 48) EXPECT_EQ(QString("UNCERTAIN"), registry.points()[0].state);
  }
  EXPECT_EQ(QString("a"), registry.points()[0].label);
  EXPECT_EQ(QString("CONFIRMED"), registry.points()[0].state);
  EXPECT_NEAR(3.8, registry.points()[0].position.y(), .05);
}
TEST(PendingLitterRegistry, UnstableMapCannotConfirmEvenWithManyFrames)
{
  TestRegistry registry;
  for (int i = 0; i < 60; ++i)
    registry.observe(1+i*.125, {at()}, "test-session", identity, false, 1);
  EXPECT_TRUE(registry.points().empty());
  registry.observe(8.5, {at()}, "test-session", identity, true, 1);
  EXPECT_EQ(1, registry.points().size());
}
TEST(PendingLitterRegistry, NewSessionReobservesOldMapPointWithNewLocalOrigin)
{
  TestRegistry registry;
  confirm(registry, 1, at());
  auto transform = identity; transform[3] = -10;
  for (int i = 0; i < 4; ++i)
    registry.observe(2+i*.125, {at(11)}, "new-session", transform, true, 0);
  ASSERT_EQ(1, registry.points().size());
  EXPECT_EQ(QString("UNCERTAIN"), registry.points()[0].state);
  registry.observe(2.5, {at(11)}, "new-session", transform, true, 0);
  ASSERT_EQ(1, registry.points().size());
  EXPECT_EQ(QString("CONFIRMED"), registry.points()[0].state);
  EXPECT_DOUBLE_EQ(11, registry.points()[0].odom[0]);
  EXPECT_EQ(QPointF(1, 2), registry.points()[0].position);
}
TEST(PendingLitterRegistry, TwoNearbyDistinctObjectsAndReusedIdRemainSeparate)
{
  TestRegistry registry;
  for (int i = 0; i < 8; ++i) registry.observe(1+i*.125, {at(), at(1.35)});
  ASSERT_EQ(2, registry.points().size());
}
TEST(PendingLitterRegistry, InvalidTransformCannotConfirm)
{
  TestRegistry registry;
  auto bad = identity; bad[0] = -1;
  for (int i = 0; i < 8; ++i)
    registry.observe(1+i*.125, {at()}, "test", bad, true, 0);
  EXPECT_TRUE(registry.points().empty());
}
TEST(PendingLitterRegistry, DeletionKeepsOtherLabelsAndAllowsFreshConfirmation)
{
  TestRegistry registry;
  confirm(registry, 1, at()); confirm(registry, 2, at(3)); confirm(registry, 3, at(5));
  ASSERT_EQ(3, registry.points().size());
  EXPECT_FALSE(registry.removePoint(-1)); EXPECT_FALSE(registry.removePoint(3));
  ASSERT_TRUE(registry.removePoint(1));
  EXPECT_EQ(QString("c"), registry.points()[1].label);
  for (int i = 0; i < 4; ++i) registry.observe(4+i*.05, {at(3)});
  ASSERT_EQ(2, registry.points().size());
  registry.observe(4.2, {at(3)});
  ASSERT_EQ(3, registry.points().size());
  EXPECT_EQ(QString("d"), registry.points()[2].label);
  confirm(registry, 5, at(7));
  ASSERT_EQ(4, registry.points().size());
  EXPECT_EQ(QString("e"), registry.points()[3].label);
}
TEST(PendingLitterStore, SchemaOneBackupAndUncertainMigration)
{
  QTemporaryDir root;
  const QString path = root.path()+"/"+QString(64, 'a')+".json";
  const QByteArray original = QByteArray("{\"schema\":1,\"map_key\":\"") + QByteArray(64,'a') +
      "\",\"points\":[{\"label\":\"a\",\"category\":\"plastic\",\"x\":1,\"y\":2}]}";
  QFile file(path); ASSERT_TRUE(file.open(QIODevice::WriteOnly)); file.write(original); file.close();
  PendingLitterStore store; QVector<PendingLitterPoint> points; QString error;
  ASSERT_TRUE(store.open(root.path(), QString(64,'a'), &points, &error));
  ASSERT_EQ(1, points.size()); EXPECT_EQ(QString("UNCERTAIN"), points[0].state);
  ASSERT_TRUE(store.save(points, &error));
  QFile backup(path+".schema1.bak"); ASSERT_TRUE(backup.open(QIODevice::ReadOnly));
  EXPECT_EQ(original, backup.readAll());
  PendingLitterStore restored;
  ASSERT_TRUE(restored.open(root.path(), QString(64,'a'), &points, &error));
  EXPECT_EQ(QString("UNCERTAIN"), points[0].state);
}
TEST(PendingLitterStore, CorruptionIsPreserved)
{
  QTemporaryDir root;
  QVector<PendingLitterPoint> points;
  PendingLitterStore store;
  QString error;
  ASSERT_TRUE(store.open(root.path(), QString(64, 'a'), &points, &error));
  QFile file(store.filePath());
  ASSERT_TRUE(file.open(QIODevice::WriteOnly));
  file.write("corrupted"); file.close();
  EXPECT_FALSE(store.open(root.path(), QString(64, 'a'), &points, &error));
  EXPECT_FALSE(store.save(points, &error));
  ASSERT_TRUE(file.open(QIODevice::ReadOnly));
  EXPECT_EQ(QByteArray("corrupted"), file.readAll());
}
TEST(PendingLitterStore, ConcurrentWriterCannotOverwrite)
{
  QTemporaryDir root;
  QVector<PendingLitterPoint> points;
  PendingLitterStore first, second;
  QString error;
  ASSERT_TRUE(first.open(root.path(), QString(64, 'a'), &points, &error));
  ASSERT_TRUE(second.open(root.path(), QString(64, 'a'), &points, &error));
  ASSERT_TRUE(first.save({{"a", "plastic", {1, 2}}}, &error));
  EXPECT_FALSE(second.save({}, &error));
}
}  // namespace
}  // namespace autolabor_operator_gui

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
