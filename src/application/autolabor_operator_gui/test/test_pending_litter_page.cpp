#include <autolabor_operator_gui/main_window.h>
#include <autolabor_operator_gui/pending_litter_page.h>
#include <gtest/gtest.h>
#include <boost/make_shared.hpp>
#include <QApplication>
#include <QGraphicsEllipseItem>
#include <QGraphicsScene>
#include <QListWidget>
#include <QLabel>
#include <QPushButton>
#include <QTabWidget>
#include <QTemporaryDir>
#include <QWheelEvent>
#include <cmath>
#include <limits>

namespace autolabor_operator_gui
{
namespace
{
nav_msgs::OccupancyGrid::Ptr makeMap()
{
  auto map = boost::make_shared<nav_msgs::OccupancyGrid>();
  map->header.frame_id = "map";
  map->info.width = 160;
  map->info.height = 100;
  map->info.resolution = 0.1;
  map->info.origin.position.x = -5;
  map->info.origin.position.y = -4;
  map->info.origin.orientation.w = 1;
  map->data.resize(16000, 0);
  for (int y = 8; y < 92; ++y)
    for (int x = 8; x < 152; ++x)
      if (x < 10 || x > 149 || y < 10 || y > 89 ||
          (x > 75 && x < 78 && (y < 43 || y > 62)))
        map->data[y * 160 + x] = 100;
  return map;
}
autolabor_fod_msgs::FodMapPositionArray::Ptr makeFrame(double stamp, double x = -2, double y = 2)
{
  auto frame = boost::make_shared<autolabor_fod_msgs::FodMapPositionArray>();
  frame->header.stamp.fromSec(stamp);
  frame->header.frame_id = "map";
  frame->source_header = frame->header;
  frame->source_header.frame_id = "zed_left_camera_optical_frame";
  frame->backend_id = "detect_and_classify";
  frame->calibration_sha256 = std::string(64, 'a');
  frame->status = "OK";
  frame->odom_session_id = "synthetic-session";
  frame->odom_frame = "camera_init";
  frame->map_stable = true;
  frame->stability_status = "STABLE";
  for (int i = 0; i < 16; ++i) frame->stable_map_from_odom[i] = i % 5 == 0 ? 1 : 0;
  autolabor_fod_msgs::FodMapPosition position;
  position.object_id = static_cast<std::uint64_t>(
      1000 + std::llround((x + 20.0) * 10.0) * 100
      + std::llround((y + 20.0) * 10.0));
  position.position_valid = true;
  position.status = "OK";
  position.material_class = "plastic";
  position.position.x = x;
  position.position.y = y;
  position.position.z = std::numeric_limits<double>::quiet_NaN();
  position.odom_position_valid = true;
  position.odom_position.x = x;
  position.odom_position.y = y;
  position.odom_position.z = 0;
  position.depth_m = 1.2;
  position.depth_sample_count = 35;
  frame->positions.push_back(position);
  return frame;
}
void consume(PendingLitterPage& page,
             const autolabor_fod_msgs::FodMapPositionArray::ConstPtr& frame)
{
  page.consume({{}, frame, ros::WallTime::now(), ros::Time::now(), false},
               ros::Time::now(), ros::WallTime::now());
}
void supplyMap(PendingLitterPage& page, nav_msgs::OccupancyGrid::ConstPtr map = makeMap())
{
  page.consume({map, {}, ros::WallTime::now(), ros::Time::now(), false},
               ros::Time::now(), ros::WallTime::now());
}
TEST(PendingLitterPage, RequiresMapAndFiveFramesWithValidGlobalDepth)
{
  PendingLitterPage page;
  page.configure("", "test_map", false);
  page.setAvailable(true);
  double start = ros::Time::now().toSec() - 0.3;
  for (int i = 0; i < 5; ++i) consume(page, makeFrame(start + i * .01));
  EXPECT_TRUE(page.points().empty());
  supplyMap(page);
  for (int i = 5; i < 9; ++i) consume(page, makeFrame(start + i * .01));
  EXPECT_TRUE(page.points().empty());
  auto invalid = makeFrame(start + .09);
  invalid->positions[0].odom_position_valid = false;
  consume(page, invalid);
  for (int i = 10; i < 14; ++i) consume(page, makeFrame(start + i * .01));
  EXPECT_TRUE(page.points().empty());
  consume(page, makeFrame(start + .14));
  ASSERT_EQ(1, page.points().size());
  auto* list = page.findChild<QListWidget*>("pendingLitterList");
  ASSERT_NE(nullptr, list);
  EXPECT_EQ(QStringLiteral("待清扫垃圾a  类别:plastic  position:map (-2.000, 2.000)m · 已确认"),
            list->item(0)->text());
}
TEST(PendingLitterPage, StaleFutureWrongFrameAndCalibrationChangeBreakStreak)
{
  for (int failure = 0; failure < 6; ++failure)
  {
    PendingLitterPage page;
    page.configure("", "test", false);
    page.setAvailable(true);
    supplyMap(page);
    const double start = ros::Time::now().toSec() - .3;
    for (int i = 0; i < 4; ++i) consume(page, makeFrame(start + i*.01));
    auto invalid = makeFrame(start + .04);
    if (failure == 0) invalid->header.frame_id = "odom";
    if (failure == 1)
    {
      invalid->source_header.stamp = ros::Time::now() - ros::Duration(2);
      invalid->header.stamp = invalid->source_header.stamp;
    }
    if (failure == 2)
    {
      invalid->source_header.stamp = ros::Time::now() + ros::Duration(2);
      invalid->header.stamp = invalid->source_header.stamp;
    }
    if (failure == 3) invalid->calibration_sha256 = std::string(64, 'b');
    if (failure == 4) invalid->positions[0].depth_m = std::numeric_limits<float>::quiet_NaN();
    if (failure == 5) invalid->positions.clear();
    consume(page, invalid);
    EXPECT_TRUE(page.points().empty()) << failure;
    for (int i = 5; i < 9; ++i) consume(page, makeFrame(start + i*.01));
    EXPECT_TRUE(page.points().empty()) << failure;
    consume(page, makeFrame(start + .09));
    EXPECT_EQ(1, page.points().size()) << failure;
  }
}
TEST(PendingLitterPage, SameMapKeepsSessionButRestartAndMapSwitchStartEmpty)
{
  QTemporaryDir root;
  PendingLitterPage page;
  page.configure(root.path(), "test_map", true);
  page.setAvailable(true);
  auto map = makeMap();
  supplyMap(page, map);
  const auto first_key = PendingLitterPage::mapKey(*map, "test_map");
  const double start = ros::Time::now().toSec() - .2;
  for (int i = 0; i < 5; ++i) consume(page, makeFrame(start + i*.01));
  ASSERT_EQ(1, page.points().size());
  map->header.stamp = ros::Time::now();
  map->info.map_load_time = ros::Time::now();
  EXPECT_EQ(first_key, PendingLitterPage::mapKey(*map, "test_map"));
  supplyMap(page, map);
  EXPECT_EQ(1, page.points().size());
  auto other = boost::make_shared<nav_msgs::OccupancyGrid>(*map);
  other->info.origin.position.x += 10;
  supplyMap(page, other);
  EXPECT_TRUE(page.points().empty());
  // Old in-flight frames cannot plant old-map coordinates after a map change.
  for (int i = 5; i < 10; ++i) consume(page, makeFrame(start + i*.01));
  EXPECT_TRUE(page.points().empty());
  supplyMap(page, map);
  EXPECT_TRUE(page.points().empty());
  PendingLitterPage restarted;
  restarted.configure(root.path(), "test_map", true);
  restarted.setAvailable(true);
  supplyMap(restarted, map);
  EXPECT_TRUE(restarted.points().empty());
}
TEST(PendingLitterPage, OldSavedRecordsCannotReturnEvenWithLegacyPersistEnabled)
{
  QTemporaryDir root;
  auto map = makeMap();
  PendingLitterStore old_store;
  QVector<PendingLitterPoint> points;
  QString error;
  ASSERT_TRUE(old_store.open(root.path(), PendingLitterPage::mapKey(*map, "test"), &points, &error));
  ASSERT_TRUE(old_store.save({{"a", "plastic", {1,2}}}, &error));
  PendingLitterPage page;
  page.configure(root.path(), "test", true);
  page.setAvailable(true); supplyMap(page, map);
  EXPECT_TRUE(page.points().empty());
}
TEST(PendingLitterPage, DeleteButtonRemovesListAndMapAndAllowsFreshRediscovery)
{
  PendingLitterPage page;
  page.configure("", "test", false); page.setAvailable(true); supplyMap(page);
  auto* button = page.findChild<QPushButton*>("deletePendingLitterButton");
  auto* list = page.findChild<QListWidget*>("pendingLitterList");
  auto* view = dynamic_cast<PendingLitterMapView*>(page.findChild<QGraphicsView*>("pendingLitterMap"));
  ASSERT_NE(nullptr, button); ASSERT_NE(nullptr, list); ASSERT_NE(nullptr, view);
  EXPECT_FALSE(button->isEnabled());
  const double start = ros::Time::now().toSec()-.3;
  for (int i = 0; i < 5; ++i) consume(page, makeFrame(start+i*.01));
  ASSERT_EQ(1, list->count());
  list->setCurrentRow(0);
  ASSERT_TRUE(button->isEnabled());
  button->click();
  EXPECT_EQ(0, list->count()); EXPECT_TRUE(page.points().empty());
  EXPECT_FALSE(button->isEnabled());
  for (auto* item : view->scene()->items())
    EXPECT_EQ(nullptr, dynamic_cast<QGraphicsEllipseItem*>(item));
  for (int i = 5; i < 9; ++i) consume(page, makeFrame(start+i*.01));
  EXPECT_TRUE(page.points().empty());
  consume(page, makeFrame(start+.09));
  ASSERT_EQ(1, page.points().size());
  EXPECT_EQ(QString("b"), page.points()[0].label);
  page.consume({{}, {}, ros::WallTime::now(), ros::Time::now(), true}, ros::Time::now(), ros::WallTime::now());
  supplyMap(page);
  for (int i = 15; i < 20; ++i) consume(page, makeFrame(start+i*.01));
  EXPECT_EQ(1, page.points().size());
  for (int i = 20; i < 25; ++i) consume(page, makeFrame(start+i*.01, 3, 2));
  ASSERT_EQ(2, page.points().size());
  EXPECT_EQ(QString("c"), page.points()[1].label);
  PendingLitterPage restarted;
  restarted.configure("", "test", false); restarted.setAvailable(true); supplyMap(restarted);
  EXPECT_TRUE(restarted.points().empty());
  for (int i = 25; i < 30; ++i) consume(restarted, makeFrame(start+i*.01));
  ASSERT_EQ(1, restarted.points().size());
  EXPECT_EQ(QString("a"), restarted.points()[0].label);
}
TEST(PendingLitterPage, CleaningColumnKeepsIdentityAndCompletionRemovesExactObject)
{
  PendingLitterPage page;
  page.configure("", "test", false); page.setAvailable(true); supplyMap(page);
  auto* pending = page.findChild<QListWidget*>("pendingLitterList");
  auto* cleaning = page.findChild<QListWidget*>("cleaningLitterList");
  auto* add = page.findChild<QPushButton*>("addPendingLitterToCleaningButton");
  auto* remove = page.findChild<QPushButton*>("removeCleaningLitterButton");
  auto* erase = page.findChild<QPushButton*>("deletePendingLitterButton");
  auto* clean = page.findChild<QPushButton*>("cleanLitterButton");
  ASSERT_NE(nullptr, cleaning); ASSERT_NE(nullptr, add); ASSERT_NE(nullptr, remove);
  ASSERT_NE(nullptr, clean);
  EXPECT_FALSE(add->isEnabled()); EXPECT_FALSE(remove->isEnabled()); EXPECT_FALSE(clean->isEnabled());
  const double start = ros::Time::now().toSec()-.3;
  const auto frameAt = [&](int i) {
    auto frame = makeFrame(start+i*.01);
    frame->positions.push_back(makeFrame(start, 3, 2)->positions[0]);
    frame->positions.push_back(makeFrame(start, 5, 2)->positions[0]);
    return frame;
  };
  for (int i = 0; i < 5; ++i) consume(page, frameAt(i));
  ASSERT_EQ(3, pending->count());
  pending->setCurrentRow(1); add->click();
  ASSERT_EQ(2, pending->count()); ASSERT_EQ(1, cleaning->count());
  EXPECT_EQ(QString("b"), cleaning->item(0)->data(Qt::UserRole).toString());
  EXPECT_EQ(3, page.points().size());
  for (int i = 5; i < 10; ++i) consume(page, frameAt(i));
  EXPECT_EQ(2, pending->count()); EXPECT_EQ(1, cleaning->count());
  EXPECT_EQ(0, cleaning->currentRow());
  EXPECT_TRUE(clean->toolTip().contains("1.5m"));
  EXPECT_FALSE(clean->toolTip().contains("10cm"));
  const quint64 cleaning_object_id = page.points()[1].object_id;
  ASSERT_NE(quint64(0), cleaning_object_id);
  page.markCleaned(cleaning_object_id);
  EXPECT_EQ(0, cleaning->count()); EXPECT_EQ(2, page.points().size());
  EXPECT_TRUE(page.findChild<QLabel*>("cleaningLitterNotice")->text().contains("清扫完成"));
  // The exact completed identity is removed; other queued points remain.
  ASSERT_EQ(2, pending->count());
  pending->setCurrentRow(1); erase->click();
  ASSERT_EQ(1, page.points().size());
  EXPECT_EQ(0, cleaning->count());
  pending->setCurrentRow(0); add->click();
  page.consume({{}, {}, ros::WallTime::now(), ros::Time::now(), true}, ros::Time::now(), ros::WallTime::now());
  supplyMap(page);
  EXPECT_EQ(1, cleaning->count()); EXPECT_EQ(0, pending->count());
  EXPECT_TRUE(cleaning->item(0)->text().contains("待复核"));
  // Reprojection updates the queued identity too; it must not move back left.
  auto shifted = frameAt(11);
  shifted->stable_map_from_odom[3] = 1.8;
  shifted->correction_generation = 1;
  consume(page, shifted);
  EXPECT_EQ(1, cleaning->count()); EXPECT_EQ(0, pending->count());
  EXPECT_TRUE(cleaning->item(0)->text().contains("-0.200"));
  auto map = makeMap(); map->data[0] = 100; supplyMap(page, map);
  EXPECT_EQ(0, cleaning->count()); EXPECT_EQ(0, pending->count());
  EXPECT_FALSE(clean->isEnabled()); EXPECT_FALSE(remove->isEnabled());
}
TEST(PendingLitterPage, AbortPreservesTargetAndLatchedCompletionRemovesIt)
{
  PendingLitterPage page;
  page.configure("", "test", false); page.setAvailable(true); supplyMap(page);
  const double start = ros::Time::now().toSec() - .2;
  for (int i = 0; i < 5; ++i) consume(page, makeFrame(start + i*.01));
  ASSERT_EQ(1, page.points().size());
  auto* list = page.findChild<QListWidget*>("pendingLitterList");
  list->setCurrentRow(0);
  page.findChild<QPushButton*>("addPendingLitterToCleaningButton")->click();
  auto* clean = page.findChild<QPushButton*>("cleanLitterButton");
  auto* cancel = page.findChild<QPushButton*>("cancelLitterCleaningButton");
  ASSERT_TRUE(clean->isEnabled());
  page.updateCleaningStatus(R"({"state":"NAVIGATING","active":true,"reason":"moving"})");
  EXPECT_FALSE(clean->isEnabled()); EXPECT_TRUE(cancel->isEnabled());
  page.updateCleaningStatus(R"({"state":"ABORT","active":false,"reason":"operator exit"})");
  EXPECT_TRUE(clean->isEnabled()); EXPECT_FALSE(cancel->isEnabled());
  EXPECT_EQ(1, page.points().size());
  EXPECT_TRUE(page.findChild<QLabel*>("cleaningLitterNotice")->text().contains("中止"));
  const auto id = page.points()[0].object_id;
  const auto completed = QStringLiteral(
      "{\"state\":\"COMPLETE\",\"active\":false,\"last_completed_object_id_text\":\"%1\"}").arg(id);
  page.updateCleaningStatus(completed);
  EXPECT_EQ(1, page.points().size());  // An old latched result is not a new mission.
  page.updateCleaningStatus(R"({"state":"VISUAL_SERVO","active":true})");
  page.updateCleaningStatus(completed);
  EXPECT_TRUE(page.points().empty());
  EXPECT_FALSE(cancel->isEnabled());
}
TEST(PendingLitterPage, UnstableAndStaleContextCannotRemainConfirmed)
{
  PendingLitterPage page;
  page.configure("", "test", false); page.setAvailable(true); supplyMap(page);
  const double start = ros::Time::now().toSec()-.3;
  for (int i = 0; i < 5; ++i) consume(page, makeFrame(start+i*.01));
  ASSERT_EQ(1, page.points().size());
  auto unstable = makeFrame(start+.06);
  unstable->map_stable = false;
  unstable->stability_status = "MAP_CORRECTION_JUMP";
  unstable->correction_generation = 1;
  consume(page, unstable);
  EXPECT_EQ(QString("UNCERTAIN"), page.points()[0].state);
  auto recovered = makeFrame(start+.07); recovered->correction_generation = 1;
  consume(page, recovered);
  EXPECT_EQ(QString("CONFIRMED"), page.points()[0].state);
  page.consume({}, ros::Time::now(), ros::WallTime::now()+ros::WallDuration(1));
  EXPECT_EQ(QString("UNCERTAIN"), page.points()[0].state);
  EXPECT_EQ(1, page.points().size());
}
TEST(PendingLitterMapView, RotatedMapCoordinatesPanAndZoomShareOneScene)
{
  PendingLitterMapView view;
  auto map = makeMap();
  const double pi = std::acos(-1.0);
  map->info.origin.orientation.z = std::sin(pi/4);
  map->info.origin.orientation.w = std::cos(pi/4);
  view.resize(900, 600);
  view.setMap(*map);
  const QPointF world(-7, -1);  // grid local (3, 2) metres at +90 deg.
  const QPointF scene = view.worldToScene(world);
  EXPECT_NEAR(30, scene.x(), 1e-5);
  EXPECT_NEAR(80, scene.y(), 1e-5);
  view.setPoints({{"a", "plastic", world}});
  QGraphicsEllipseItem* marker = nullptr;
  for (auto* item : view.scene()->items())
    if (item->data(0).toString() == "a") marker = dynamic_cast<QGraphicsEllipseItem*>(item);
  ASSERT_NE(nullptr, marker);
  EXPECT_EQ(scene, marker->scenePos());
  EXPECT_EQ(QColor("#e02020"), marker->brush().color());
  EXPECT_EQ(Qt::SolidPattern, marker->brush().style());
  view.show(); QApplication::processEvents();
  view.scale(2, 2);
  view.centerOn(scene + QPointF(10, 5));
  EXPECT_EQ(view.mapFromScene(scene), view.mapFromScene(marker->scenePos()));
  EXPECT_EQ(scene, marker->scenePos());
  QApplication::processEvents();
  const QImage rendered = view.viewport()->grab().toImage();
  const QPoint screen_point = view.mapFromScene(scene);
  ASSERT_TRUE(rendered.rect().contains(screen_point));
  const QColor center = rendered.pixelColor(screen_point);
  EXPECT_GT(center.red(), 190);
  EXPECT_LT(center.green(), 80);
  EXPECT_LT(center.blue(), 80);
  const auto transform = view.transform();
  view.setPoints({{"a", "plastic", world}, {"b", "metal", {-6, -1}}});
  EXPECT_EQ(transform, view.transform());
}
TEST(PendingLitterPage, SyntheticScreenshot)
{
  PendingLitterPage page;
  page.configure("", "synthetic", false);
  page.setAvailable(true);
  supplyMap(page);
  const double start = ros::Time::now().toSec() - .2;
  for (int i = 0; i < 5; ++i)
  {
    auto frame = makeFrame(start + i*.01);
    frame->positions.push_back(makeFrame(start, 5, 3)->positions[0]);
    frame->positions.back().material_class = "metal";
    frame->positions.push_back(makeFrame(start, 4, -1)->positions[0]);
    frame->positions.back().material_class = "paper";
    consume(page, frame);
  }
  ASSERT_EQ(3, page.points().size());
  page.findChild<QListWidget*>("pendingLitterList")->setCurrentRow(1);
  page.findChild<QPushButton*>("addPendingLitterToCleaningButton")->click();
  page.resize(1200, 800);
  page.show(); QApplication::processEvents();
  auto* pending = page.findChild<QListWidget*>("pendingLitterList");
  auto* cleaning = page.findChild<QListWidget*>("cleaningLitterList");
  auto* map = page.findChild<QGraphicsView*>("pendingLitterMap");
  const QPoint map_origin = map->mapTo(&page, QPoint());
  const QPoint pending_origin = pending->mapTo(&page, QPoint());
  const QPoint cleaning_origin = cleaning->mapTo(&page, QPoint());
  EXPECT_EQ(map_origin.x(), pending_origin.x());
  EXPECT_GT(pending_origin.y(), map_origin.y() + map->height());
  EXPECT_GT(cleaning_origin.x(), map_origin.x() + map->width());
  EXPECT_GT(cleaning_origin.x(), pending_origin.x() + pending->width());
  EXPECT_GT(cleaning->height(), pending->height());
  if (qEnvironmentVariableIsSet("PENDING_LITTER_SCREENSHOT"))
    EXPECT_TRUE(page.grab().save(qEnvironmentVariable("PENDING_LITTER_SCREENSHOT")));
}
}  // namespace

class PendingLitterIntegrationTest : public testing::Test
{
protected:
  void SetUp() override
  {
    ros::NodeHandle parameters("~");
    parameters.setParam("enable_rviz", false);
    parameters.setParam("static_map_mode", true);
    parameters.setParam("pending_litter_persist", false);
    parameters.setParam("configured_vision_backend", "detect_and_classify");
    window_.reset(new MainWindow);
    window_->master_probe_timer_.stop();
    window_->ui_refresh_timer_.stop();
    window_->master_online_ = true;
    window_->previous_probe_online_ = true;
    window_->setupRosInterfaces();
  }
  void TearDown() override { window_.reset(); }
  void verifyOrderedFramesAndUnchangedTabs()
  {
    QStringList tabs;
    for (int i = 0; i < window_->tabs_->count(); ++i) tabs << window_->tabs_->tabText(i);
    EXPECT_EQ(QStringList({"综合", "FAST-LIO", "测试", "视觉", "待清扫垃圾", "清扫",
                           "AI语音控制", "BPU 预览", "日志"}), tabs);
    window_->mapCallback(makeMap());
    const double start = ros::Time::now().toSec() - .2;
    for (int i = 0; i < 4; ++i) window_->visionPositionsCallback(makeFrame(start + i*.01));
    window_->refreshPendingLitter();
    EXPECT_TRUE(window_->pending_litter_page_->points().empty());
    for (int i = 0; i < 15; ++i) window_->refreshPendingLitter();
    EXPECT_TRUE(window_->pending_litter_page_->points().empty());
    window_->visionPositionsCallback(makeFrame(start + .04));
    window_->refreshPendingLitter();
    EXPECT_EQ(1, window_->pending_litter_page_->points().size());
    EXPECT_EQ(window_->overview_tab_index_, window_->tabs_->currentIndex());
    EXPECT_FALSE(window_->rviz_initialized_);
    if (qEnvironmentVariableIsSet("PENDING_LITTER_WINDOW_SCREENSHOT"))
    {
      window_->tabs_->setCurrentWidget(window_->pending_litter_page_);
      window_->resize(1440, 960);
      window_->show(); QApplication::processEvents();
      EXPECT_TRUE(window_->grab().save(qEnvironmentVariable("PENDING_LITTER_WINDOW_SCREENSHOT")));
    }
  }
  void verifyBacklogAndReconnect()
  {
    window_->mapCallback(makeMap());
    double start = ros::Time::now().toSec() - .2;
    for (int i = 0; i < 4; ++i) window_->visionPositionsCallback(makeFrame(start + i*.001));
    window_->refreshPendingLitter();
    // Overflow consists of duplicate frames; it must break the earlier streak.
    for (int i = 0; i < 130; ++i) window_->visionPositionsCallback(makeFrame(start + .01));
    window_->refreshPendingLitter();
    EXPECT_TRUE(window_->pending_litter_page_->points().empty());
    for (int i = 0; i < 3; ++i) window_->visionPositionsCallback(makeFrame(start + .02 + i*.001));
    window_->refreshPendingLitter();
    EXPECT_TRUE(window_->pending_litter_page_->points().empty());
    window_->visionPositionsCallback(makeFrame(start + .03));
    window_->refreshPendingLitter();
    ASSERT_EQ(1, window_->pending_litter_page_->points().size());
    window_->setupRosInterfaces();
    window_->mapCallback(makeMap());
    window_->refreshPendingLitter();
    EXPECT_EQ(1, window_->pending_litter_page_->points().size());
  }
  std::unique_ptr<MainWindow> window_;
};
TEST_F(PendingLitterIntegrationTest, OrderedCallbacksPreserveAllFramesAndTabLayout)
{ verifyOrderedFramesAndUnchangedTabs(); }
TEST_F(PendingLitterIntegrationTest, BacklogAndReconnectResetOnlyUnconfirmedStreaks)
{ verifyBacklogAndReconnect(); }
}  // namespace autolabor_operator_gui

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "pending_litter_page_test");
  ros::NodeHandle test_node;
  QApplication application(argc, argv);
  QTemporaryDir settings;
  qputenv("XDG_CONFIG_HOME", settings.path().toUtf8());
  const int result = RUN_ALL_TESTS();
  ros::shutdown();
  return result;
}
