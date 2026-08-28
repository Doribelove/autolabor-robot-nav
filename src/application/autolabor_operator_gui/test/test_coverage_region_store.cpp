#include <autolabor_operator_gui/coverage_region_store.h>

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLockFile>
#include <QTemporaryDir>

#include <gtest/gtest.h>

#include <limits>

namespace autolabor_operator_gui
{
namespace
{
const QString kDigest(64, QLatin1Char('a'));

QVector<QPointF> rectangle()
{
  return { QPointF(0.0, 0.0), QPointF(4.0, 0.0),
           QPointF(4.0, 3.0), QPointF(0.0, 3.0) };
}

CoverageRegionStore configuredStore(
    const QString& root,
    const QString& source = QString())
{
  CoverageRegionStore store(root);
  const QString effective_source = source.isEmpty() ? root : source;
  EXPECT_TRUE(store.setMapContext(kDigest, effective_source,
                                  QStringLiteral("fused")));
  return store;
}

TEST(CoverageRegionStoreTest, RoundTripsUtf8WithoutRuntimePlanState)
{
  QTemporaryDir directory;
  ASSERT_TRUE(directory.isValid());
  CoverageRegionStore store = configuredStore(directory.path());
  QString error;
  ASSERT_TRUE(store.load(&error)) << error.toStdString();
  CoverageRegionRecord created;
  ASSERT_TRUE(store.addRegion(QStringLiteral("大厅 A区"), rectangle(), &created,
                              &error))
      << error.toStdString();
  EXPECT_FALSE(created.id.isEmpty());
  EXPECT_EQ(1U, created.revision);

  QFile file(store.filePath());
  ASSERT_TRUE(file.open(QIODevice::ReadOnly));
  const QByteArray payload = file.readAll();
  EXPECT_TRUE(payload.contains(QStringLiteral("大厅 A区").toUtf8()));
  EXPECT_FALSE(payload.contains("plan_id"));
  EXPECT_FALSE(payload.contains("planned_path"));
  EXPECT_FALSE(payload.contains("progress"));

  CoverageRegionStore reloaded = configuredStore(directory.path());
  ASSERT_TRUE(reloaded.load(&error)) << error.toStdString();
  ASSERT_EQ(1, reloaded.regions().size());
  EXPECT_EQ(created.id, reloaded.regions().front().id);
  EXPECT_EQ(created.polygon, reloaded.regions().front().polygon);
}

TEST(CoverageRegionStoreTest, RejectsInvalidGeometryAndDuplicateNames)
{
  QTemporaryDir directory;
  CoverageRegionStore store = configuredStore(directory.path());
  QString error;
  ASSERT_TRUE(store.load(&error));
  ASSERT_TRUE(store.addRegion(QStringLiteral("Region One"), rectangle(), nullptr,
                              &error));
  EXPECT_FALSE(store.addRegion(QStringLiteral("region one"), rectangle(), nullptr,
                               &error));

  QVector<QPointF> duplicate = rectangle();
  duplicate[3] = duplicate[0];
  EXPECT_FALSE(CoverageRegionStore::validatePolygon(duplicate, &error));
  QVector<QPointF> self_intersecting = {
    QPointF(0.0, 0.0), QPointF(3.0, 3.0),
    QPointF(0.0, 3.0), QPointF(3.0, 0.0)
  };
  EXPECT_FALSE(CoverageRegionStore::validatePolygon(self_intersecting, &error));
  QVector<QPointF> non_finite = rectangle();
  non_finite[0].setX(std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(CoverageRegionStore::validatePolygon(non_finite, &error));
}

TEST(CoverageRegionStoreTest, CorruptFileIsNotOverwritten)
{
  QTemporaryDir directory;
  CoverageRegionStore store = configuredStore(directory.path());
  ASSERT_TRUE(QDir().mkpath(QFileInfo(store.filePath()).absolutePath()));
  QFile corrupt(store.filePath());
  ASSERT_TRUE(corrupt.open(QIODevice::WriteOnly));
  const QByteArray original("{ definitely-not-json");
  ASSERT_EQ(original.size(), corrupt.write(original));
  corrupt.close();

  QString error;
  EXPECT_FALSE(store.load(&error));
  EXPECT_FALSE(store.addRegion(QStringLiteral("不会覆盖"), rectangle(), nullptr,
                               &error));
  ASSERT_TRUE(corrupt.open(QIODevice::ReadOnly));
  EXPECT_EQ(original, corrupt.readAll());
}

TEST(CoverageRegionStoreTest, LockContentionKeepsInMemoryStateUnchanged)
{
  QTemporaryDir directory;
  CoverageRegionStore store = configuredStore(directory.path());
  QString error;
  ASSERT_TRUE(store.load(&error));
  QLockFile lock(store.filePath() + QStringLiteral(".lock"));
  ASSERT_TRUE(lock.tryLock());
  EXPECT_FALSE(store.addRegion(QStringLiteral("锁冲突"), rectangle(), nullptr,
                               &error));
  EXPECT_TRUE(store.regions().isEmpty());
}

TEST(CoverageRegionStoreTest, SameGridLoadsAcrossMapSourceAliases)
{
  QTemporaryDir map_set;
  QTemporaryDir alias_parent;
  ASSERT_TRUE(map_set.isValid());
  ASSERT_TRUE(alias_parent.isValid());
  const QString alias =
      QDir(alias_parent.path()).filePath(QStringLiteral("latest"));
  ASSERT_TRUE(QFile::link(map_set.path(), alias));

  CoverageRegionStore store = configuredStore(alias, alias);
  QString error;
  ASSERT_TRUE(store.load(&error));
  ASSERT_TRUE(store.addRegion(QStringLiteral("地图绑定"), rectangle(), nullptr,
                              &error));

  CoverageRegionStore reloaded = configuredStore(map_set.path());
  ASSERT_TRUE(reloaded.load(&error)) << error.toStdString();
  ASSERT_EQ(1, reloaded.regions().size());
  EXPECT_EQ(QFileInfo(map_set.path()).canonicalFilePath(),
            reloaded.regions().front().map_source);
  EXPECT_TRUE(reloaded.addRegion(QStringLiteral("第二块"), rectangle(), nullptr,
                                 &error))
      << error.toStdString();
}

TEST(CoverageRegionStoreTest, SameDigestDifferentMapSetsDoNotShareRegions)
{
  QTemporaryDir first_map;
  QTemporaryDir second_map;
  ASSERT_TRUE(first_map.isValid());
  ASSERT_TRUE(second_map.isValid());
  QString error;
  CoverageRegionStore first = configuredStore(first_map.path());
  ASSERT_TRUE(first.load(&error)) << error.toStdString();
  ASSERT_TRUE(first.addRegion(QStringLiteral("只属于地图 A"), rectangle(),
                              nullptr, &error))
      << error.toStdString();

  CoverageRegionStore second = configuredStore(second_map.path());
  ASSERT_TRUE(second.load(&error)) << error.toStdString();
  EXPECT_TRUE(second.regions().isEmpty());
  EXPECT_NE(first.filePath(), second.filePath());
  EXPECT_TRUE(first.filePath().startsWith(first_map.path()));
  EXPECT_TRUE(second.filePath().startsWith(second_map.path()));
}

TEST(CoverageRegionStoreTest, MigratesMatchingLegacyStoreWithoutDeletingIt)
{
  QTemporaryDir map_set;
  QTemporaryDir legacy_root;
  ASSERT_TRUE(map_set.isValid());
  ASSERT_TRUE(legacy_root.isValid());
  QString error;
  CoverageRegionStore original = configuredStore(map_set.path());
  ASSERT_TRUE(original.load(&error));
  ASSERT_TRUE(original.addRegion(QStringLiteral("旧区域"), rectangle(), nullptr,
                                 &error));
  QFile new_file(original.filePath());
  ASSERT_TRUE(new_file.open(QIODevice::ReadOnly));
  const QByteArray payload = new_file.readAll();
  new_file.close();
  ASSERT_TRUE(QFile::remove(original.filePath()));

  CoverageRegionStore migrated = configuredStore(map_set.path());
  migrated.setLegacyRoot(legacy_root.path());
  const QString legacy_path = migrated.legacyFilePath();
  ASSERT_TRUE(QDir().mkpath(QFileInfo(legacy_path).absolutePath()));
  QFile legacy_file(legacy_path);
  ASSERT_TRUE(legacy_file.open(QIODevice::WriteOnly));
  ASSERT_EQ(payload.size(), legacy_file.write(payload));
  legacy_file.close();

  ASSERT_TRUE(migrated.load(&error)) << error.toStdString();
  ASSERT_EQ(1, migrated.regions().size());
  EXPECT_EQ(QStringLiteral("旧区域"), migrated.regions().front().name);
  EXPECT_TRUE(QFileInfo(migrated.filePath()).isFile());
  EXPECT_TRUE(QFileInfo(legacy_path).isFile());
}

TEST(CoverageRegionStoreTest, DoesNotMigrateLegacyStoreFromDifferentMapSet)
{
  QTemporaryDir first_map;
  QTemporaryDir second_map;
  QTemporaryDir legacy_root;
  ASSERT_TRUE(first_map.isValid());
  ASSERT_TRUE(second_map.isValid());
  ASSERT_TRUE(legacy_root.isValid());
  QString error;
  CoverageRegionStore original = configuredStore(first_map.path());
  ASSERT_TRUE(original.load(&error));
  ASSERT_TRUE(original.addRegion(QStringLiteral("地图 A 区域"), rectangle(),
                                 nullptr, &error));
  QFile source_file(original.filePath());
  ASSERT_TRUE(source_file.open(QIODevice::ReadOnly));
  const QByteArray payload = source_file.readAll();

  CoverageRegionStore second = configuredStore(second_map.path());
  second.setLegacyRoot(legacy_root.path());
  const QString legacy_path = second.legacyFilePath();
  ASSERT_TRUE(QDir().mkpath(QFileInfo(legacy_path).absolutePath()));
  QFile legacy_file(legacy_path);
  ASSERT_TRUE(legacy_file.open(QIODevice::WriteOnly));
  ASSERT_EQ(payload.size(), legacy_file.write(payload));
  legacy_file.close();

  ASSERT_TRUE(second.load(&error)) << error.toStdString();
  EXPECT_TRUE(second.regions().isEmpty());
  EXPECT_FALSE(QFileInfo(second.filePath()).exists());
}

TEST(CoverageRegionStoreTest, RejectsStaleWriterWithoutLosingCommittedRegion)
{
  QTemporaryDir directory;
  CoverageRegionStore first = configuredStore(directory.path());
  CoverageRegionStore stale = configuredStore(directory.path());
  QString error;
  ASSERT_TRUE(first.load(&error));
  ASSERT_TRUE(stale.load(&error));
  ASSERT_TRUE(first.addRegion(QStringLiteral("先提交"), rectangle(), nullptr,
                              &error))
      << error.toStdString();
  EXPECT_FALSE(stale.addRegion(QStringLiteral("过期写入"), rectangle(), nullptr,
                               &error));
  EXPECT_TRUE(error.contains(QStringLiteral("其他进程修改")));
  EXPECT_FALSE(stale.isLoaded());
  ASSERT_TRUE(stale.load(&error)) << error.toStdString();
  ASSERT_TRUE(stale.addRegion(QStringLiteral("重新加载后提交"), rectangle(),
                              nullptr, &error))
      << error.toStdString();

  CoverageRegionStore verify = configuredStore(directory.path());
  ASSERT_TRUE(verify.load(&error)) << error.toStdString();
  ASSERT_EQ(2, verify.regions().size());
  EXPECT_EQ(QStringLiteral("先提交"), verify.regions().front().name);
  EXPECT_EQ(QStringLiteral("重新加载后提交"), verify.regions().back().name);
}

TEST(CoverageRegionStoreTest, RejectsUnsafeRootAndMapContextComponents)
{
  QTemporaryDir valid_map;
  ASSERT_TRUE(valid_map.isValid());
  QString error;
  CoverageRegionStore relative(QStringLiteral("relative/coverage_regions"));
  ASSERT_TRUE(relative.setMapContext(kDigest, valid_map.path(),
                                     QStringLiteral("fused")));
  EXPECT_FALSE(relative.load(&error));

  CoverageRegionStore filesystem_root(QDir::rootPath());
  ASSERT_TRUE(filesystem_root.setMapContext(
      kDigest, QDir::rootPath(), QStringLiteral("fused")));
  EXPECT_FALSE(filesystem_root.load(&error));

  CoverageRegionStore context;
  EXPECT_FALSE(context.setMapContext(
      kDigest, valid_map.path() + QStringLiteral("\nforged"),
      QStringLiteral("fused"), &error));
  EXPECT_FALSE(context.setMapContext(kDigest, valid_map.path(),
                                     QStringLiteral("../fused"), &error));
  EXPECT_FALSE(context.setMapContext(kDigest, valid_map.path(),
                                     QStringLiteral(".."), &error));
}

TEST(CoverageRegionStoreTest, RejectsSymlinkedChildWithoutWritingOutsideRoot)
{
  QTemporaryDir root;
  QTemporaryDir outside;
  ASSERT_TRUE(root.isValid());
  ASSERT_TRUE(outside.isValid());
  const QString linked_regions =
      QDir(root.path()).filePath(QStringLiteral("coverage_regions"));
  ASSERT_TRUE(QFile::link(outside.path(), linked_regions));

  CoverageRegionStore store = configuredStore(root.path());
  QString error;
  EXPECT_FALSE(store.load(&error));
  EXPECT_FALSE(QFileInfo(QDir(outside.path()).filePath(
      QStringLiteral("fused"))).exists());
}

}  // namespace
}  // namespace autolabor_operator_gui

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
