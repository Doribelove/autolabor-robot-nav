#include <autolabor_operator_gui/main_window.h>
#include <autolabor_operator_gui/ros_graph_reconnect.h>

#include <gtest/gtest.h>
#include <boost/make_shared.hpp>
#include <ros/callback_queue.h>
#include <ros/master.h>
#include <ros/xmlrpc_manager.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <rviz/visualization_frame.h>
#include <rviz/visualization_manager.h>
#include <QApplication>
#include <QTabWidget>
#include <QTemporaryDir>
#include <QThread>
#include <QPlainTextEdit>
#include <QTableWidget>
#include <QLabel>
#include <QGroupBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QPixmap>
#include <limits>

#include <atomic>
#include <chrono>
#include <functional>
#include <cmath>
#include <set>
#include <thread>

namespace autolabor_operator_gui
{
namespace
{
class ProbeCallback : public ros::CallbackInterface
{
public:
  explicit ProbeCallback(std::function<void()> callback) : callback_(std::move(callback)) {}
  CallResult call() override
  {
    callback_();
    return Success;
  }
private:
  std::function<void()> callback_;
};
}

class RvizCallbackIsolationTest : public testing::Test
{
protected:
  void SetUp() override
  {
    ros::NodeHandle parameters("~");
    const bool rendering = qEnvironmentVariableIsSet("AUTOLABOR_RVIZ_STRESS");
    parameters.setParam("enable_rviz", rendering);
    parameters.setParam("static_map_mode", rendering);
    parameters.setParam("cloud_point_count_topic", "/test/cloud_count");
    window_.reset(new MainWindow);
    window_->resize(1440, 960);
    if (rendering)
    {
      window_->show();
      QApplication::processEvents();
    }
    // Use the real application setup, without rendering or any robot nodes.
    window_->master_probe_timer_.stop();
    window_->ui_refresh_timer_.stop();
    window_->master_online_ = true;
    window_->previous_probe_online_ = true;
    window_->setupRosInterfaces();
    ASSERT_TRUE(window_->ros_interfaces_ready_);
  }

  void TearDown() override
  {
    window_.reset();
    ros::getGlobalCallbackQueue()->clear();
  }

  ros::CallbackQueueInterface* telemetryQueue() const
  {
    return window_->node_->getCallbackQueue();
  }

  void reconnect()
  {
    window_->setupRosInterfaces();
  }

  void verifyParameterEditingWithoutMapRetainsTaskLocks()
  {
    window_->static_map_mode_ = true;
    window_->navigation_profile_dirty_ = false;
    window_->navigation_profile_synced_ = true;
    window_->navigation_profile_apply_timer_.stop();
    window_->telemetry_.coverage_status_received = true;
    window_->telemetry_.coverage_status_received_at = ros::WallTime::now();
    window_->telemetry_.coverage_status.state = "IDLE";
    window_->telemetry_.coverage_status.active = false;
    window_->refreshUi();
    EXPECT_TRUE(window_->coverage_width_input_->isEnabled());
    EXPECT_TRUE(window_->coverage_speed_input_->isEnabled());
    EXPECT_FALSE(window_->coverage_start_button_->isEnabled());
    window_->telemetry_.coverage_status.active = true;
    window_->refreshUi();
    EXPECT_FALSE(window_->coverage_width_input_->isEnabled());
    window_->telemetry_.coverage_status.active = false;
    window_->coverage_plan_id_ = "existing-preview";
    window_->refreshUi();
    EXPECT_FALSE(window_->coverage_width_input_->isEnabled());
    window_->coverage_plan_id_.clear();
    window_->telemetry_.coverage_status_received_at = ros::WallTime::now()-ros::WallDuration(10.0);
    window_->refreshUi();
    EXPECT_FALSE(window_->coverage_width_input_->isEnabled());
  }

  void verifyTelemetryDelivery()
  {
    ros::NodeHandle publisher_node;
    auto publisher = publisher_node.advertise<std_msgs::UInt64>("/test/cloud_count", 1, true);
    std_msgs::UInt64 message;
    message.data = 6471;
    publisher.publish(message);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!window_->snapshot().cloud_received &&
           std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    EXPECT_TRUE(window_->snapshot().cloud_received);
    EXPECT_EQ(6471U, window_->snapshot().cloud_point_count);
  }

  void verifyBevCleaningControls()
  {
    for (auto* tabs : window_->findChildren<QTabWidget*>())
      for (int i = 0; i < tabs->count(); ++i)
        EXPECT_NE(QStringLiteral("待清扫垃圾"), tabs->tabText(i));
    EXPECT_NE(nullptr, window_->findChild<QWidget*>("bevPreviewPage"));
    EXPECT_NE(nullptr, window_->findChild<QPushButton*>("startBevCleaningButton"));
    window_->refreshBevCleaning();
    EXPECT_FALSE(window_->bev_cleaning_start_->isEnabled());
    window_->bev_cleaner_status_json_ = QStringLiteral(
        R"({"state":"IDLE","reason":"等待开始","perception":"三路 BEV 在线","active":false,"visible_count":2,"passed_count":1,"skipped_count":0})");
    window_->bev_cleaner_received_ = ros::WallTime::now();
    window_->refreshBevCleaning();
    EXPECT_TRUE(window_->bev_cleaning_start_->isEnabled());
    EXPECT_FALSE(window_->bev_cleaning_stop_->isEnabled());
    EXPECT_TRUE(window_->bev_cleaning_status_->text().contains(QStringLiteral("视野目标 2")));
    window_->bev_cleaner_status_json_ = QStringLiteral(
        R"({"state":"SWEEPING","reason":"前进清扫","active":true,"visible_count":2,"selected":{"label":"paper"}})");
    window_->refreshBevCleaning();
    EXPECT_FALSE(window_->bev_cleaning_start_->isEnabled());
    EXPECT_TRUE(window_->bev_cleaning_stop_->isEnabled());
    EXPECT_TRUE(window_->bev_cleaning_status_->text().contains(QStringLiteral("paper")));
  }

  void verifyVoiceDraftAndClarification()
  {
    sweeper_mcp::AiEvent event;
    event.kind = "SMART_VOICE";
    event.state = "WAITING";
    event.result_json = u8R"({"draft":"先清扫A区，然后"})";
    window_->applyAiEvent(event);
    ASSERT_NE(nullptr, window_->ai_transcript_);
    EXPECT_EQ(QStringLiteral("[等待说完] 先清扫A区，然后"),
              window_->ai_transcript_->toPlainText());
    event.kind = "PLAN";
    event.state = "VALIDATED";
    event.result_json = u8R"({"steps":[],"needs_clarification":true,"understood_sequence":["清扫A区","回到未指定的位置"]})";
    window_->applyAiEvent(event);
    ASSERT_NE(nullptr, window_->ai_plan_table_);
    ASSERT_EQ(2, window_->ai_plan_table_->rowCount());
    EXPECT_EQ(QStringLiteral("--"), window_->ai_plan_table_->item(0, 2)->text());
    EXPECT_EQ(QStringLiteral("理解草稿·待补充，未执行"),
              window_->ai_plan_table_->item(1, 4)->text());
    event.kind = "TRANSCRIPT";
    event.state = "SMART_TURN";
    event.description = u8"先清扫A区，然后清扫B区";
    window_->applyAiEvent(event);
    EXPECT_EQ(QStringLiteral("先清扫A区，然后清扫B区"), window_->ai_transcript_->toPlainText());
  }

  void pump(int milliseconds)
  {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(milliseconds);
    do
    {
      QApplication::processEvents();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (std::chrono::steady_clock::now() < deadline);
  }

  void runRenderStress()
  {
    ASSERT_TRUE(window_->rviz_initialized_);
    ASSERT_NE(nullptr, window_->rviz_frame_);
    EXPECT_EQ(ros::getGlobalCallbackQueue(),
              window_->rviz_frame_->getManager()->getUpdateQueue());
    ros::NodeHandle node;
    auto map_pub = node.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    auto global_pub = node.advertise<nav_msgs::OccupancyGrid>(
        "/move_base/global_costmap/costmap", 1, true);
    auto local_pub = node.advertise<nav_msgs::OccupancyGrid>(
        "/move_base/local_costmap/costmap", 1, true);
    auto path_pub = node.advertise<nav_msgs::Path>("/coverage/planned_path", 1);
    auto footprint_pub = node.advertise<geometry_msgs::PolygonStamped>(
        "/move_base/local_costmap/footprint", 1);
    tf2_ros::StaticTransformBroadcaster transforms;
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.transform.rotation.w = 1.0;
    transforms.sendTransform(transform);

    nav_msgs::OccupancyGrid map;
    map.header.frame_id = "map";
    map.info.resolution = 0.1;
    map.info.origin.orientation.w = 1.0;
    map.info.width = 240;
    map.info.height = 160;
    map.data.resize(map.info.width * map.info.height, 0);
    for (unsigned int x = 0; x < map.info.width; ++x)
      map.data[x] = 100;
    map_pub.publish(map);
    pump(1500);
    const auto map_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!window_->ensureStaticMapDisplayReady(window_->snapshot()) &&
           std::chrono::steady_clock::now() < map_deadline)
      pump(100);
    ASSERT_TRUE(window_->ensureStaticMapDisplayReady(window_->snapshot()));

    // Probe the real manager's update queue while publisher and render threads
    // are busy. Capture shared counters so queued callbacks remain valid even
    // if an assertion ends the test early.
    auto wrong_thread = std::make_shared<std::atomic<int>>(0);
    auto callbacks = std::make_shared<std::atomic<int>>(0);
    auto* gui_thread = QThread::currentThread();
    auto* initial_frame = window_->rviz_frame_;
    for (int iteration = 0; iteration < 1000; ++iteration)
    {
      map.header.stamp = ros::Time::now();
      map.data[(iteration * 13) % map.data.size()] = iteration % 101;
      global_pub.publish(map);
      local_pub.publish(map);
      nav_msgs::Path path;
      path.header = map.header;
      for (int point = 0; point < 60; ++point)
      {
        geometry_msgs::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = point * 0.2;
        pose.pose.position.y = 4.0 + std::sin(point * 0.1 + iteration * 0.03);
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
      }
      path_pub.publish(path);
      geometry_msgs::PolygonStamped footprint;
      footprint.header = map.header;
      for (const auto& xy : {std::pair<float, float>{-.62f, -.45f},
                             {.62f, -.45f}, {.62f, .45f}, {-.62f, .45f}})
      {
        geometry_msgs::Point32 point;
        point.x = xy.first;
        point.y = xy.second;
        footprint.polygon.points.push_back(point);
      }
      footprint_pub.publish(footprint);
      boost::shared_ptr<ProbeCallback> callback(new ProbeCallback([=]() {
        if (QThread::currentThread() != gui_thread)
          ++(*wrong_thread);
        ++(*callbacks);
      }));
      window_->rviz_frame_->getManager()->getUpdateQueue()->addCallback(callback);
      const int tabs[] = {window_->coverage_tab_index_, window_->overview_tab_index_,
                          3, window_->overview_tab_index_};
      window_->tabs_->setCurrentIndex(tabs[iteration % 4]);
      if (iteration % 10 == 0)
      {
        window_->resize(1320 + (iteration % 5) * 20,
                        880 + (iteration % 3) * 30);
        window_->setGlobalCostmapDisplayEnabled((iteration / 10) % 2);
      }
      if (iteration % 20 == 0)
        window_->setOverview3dMapView((iteration / 20) % 2, window_->snapshot());
      pump(20);
      ASSERT_EQ(initial_frame, window_->rviz_frame_);
    }
    window_->tabs_->setCurrentIndex(window_->overview_tab_index_);
    pump(500);
    EXPECT_EQ(1000, callbacks->load());
    EXPECT_EQ(0, wrong_thread->load());
    EXPECT_TRUE(window_->ensureStaticMapDisplayReady(window_->snapshot()));
    // Reconnect without destroying the process-wide Ogre render surface.
    for (int iteration = 0; iteration < 5; ++iteration)
    {
      ASSERT_TRUE(window_->reconnectEmbeddedRviz());
      EXPECT_EQ(initial_frame, window_->rviz_frame_);
      map_pub.publish(map);
      transforms.sendTransform(transform);
      pump(300);
      ASSERT_TRUE(window_->rviz_initialized_);
      // The first readiness call starts the required latched-map resubscribe.
      // This fixture stopped refreshUi(), so drive its asynchronous handshake
      // explicitly instead of expecting READY from that first call.
      const auto ready_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      while (!window_->ensureStaticMapDisplayReady(window_->snapshot()) &&
             std::chrono::steady_clock::now() < ready_deadline)
        pump(50);
      ASSERT_TRUE(window_->ensureStaticMapDisplayReady(window_->snapshot()));
    }
    ROS_INFO("RViz stress passed: 1000 tab transitions, 100 costmap toggles, "
             "50 view changes, 5 reconnects, 1000 callbacks on the GUI thread");
  }

  void verifySurroundAndNavigationTuningCoexist()
  {
    const QByteArray previous = qgetenv("SURROUND_PREVIEW_URL");
    qputenv("SURROUND_PREVIEW_URL", "http://127.0.0.1:9");
    MainWindow combined;
    if (previous.isNull()) qunsetenv("SURROUND_PREVIEW_URL");
    else qputenv("SURROUND_PREVIEW_URL", previous);
    ASSERT_TRUE(combined.surround_mode_);
    ASSERT_NE(nullptr, combined.surround_preview_);
    int tuning = -1, vision = -1;
    for (int i=0; i<combined.tabs_->count(); ++i)
    {
      if (combined.tabs_->tabText(i) == QStringLiteral("导航调参")) tuning = i;
      if (combined.tabs_->tabText(i) == QStringLiteral("视觉")) vision = i;
    }
    ASSERT_GE(tuning, 0);
    ASSERT_GE(vision, 0);
    ASSERT_NE(nullptr, combined.coverage_parameters_widget_);
    EXPECT_TRUE(combined.tabs_->widget(tuning)->isAncestorOf(combined.coverage_parameters_widget_));
    combined.tabs_->setCurrentIndex(tuning);
    for (int p : {1, 2, 0})
    {
      auto* button = combined.findChild<QPushButton*>(QStringLiteral("surroundPort%1Button").arg(p));
      ASSERT_NE(nullptr, button);
      EXPECT_TRUE(combined.tabs_->widget(vision)->isAncestorOf(button));
      button->click();
      EXPECT_EQ(p, combined.surround_preview_->port());
      EXPECT_EQ(tuning, combined.tabs_->currentIndex());
      EXPECT_FALSE(combined.surround_preview_->live());
    }
  }

  std::unique_ptr<MainWindow> window_;
};

TEST_F(RvizCallbackIsolationTest, RvizGlobalCallbacksRunOnlyWhenGuiPumpsThem)
{
  std::atomic<int> calls{0};
  std::atomic<bool> wrong_thread{false};
  auto* gui_thread = QThread::currentThread();
  boost::shared_ptr<ProbeCallback> callback(new ProbeCallback([&]() {
    wrong_thread = QThread::currentThread() != gui_thread;
    ++calls;
  }));
  // RViz 1.14 getUpdateQueue() is this exact queue. Its callbacks access
  // MapDisplay buffers and render objects without background-thread locking.
  ros::getGlobalCallbackQueue()->addCallback(callback);
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  EXPECT_EQ(0, calls.load()) << "Application spinner stole RViz's GUI callback";
  ros::spinOnce();
  EXPECT_EQ(1, calls.load());
  EXPECT_FALSE(wrong_thread.load());
}

TEST_F(RvizCallbackIsolationTest, TelemetryStillRunsWithoutGuiEventProcessing)
{
  ASSERT_NE(ros::getGlobalCallbackQueue(), telemetryQueue());
  std::atomic<int> calls{0};
  std::atomic<bool> background{false};
  auto* gui_thread = QThread::currentThread();
  boost::shared_ptr<ProbeCallback> callback(new ProbeCallback([&]() {
    background = QThread::currentThread() != gui_thread;
    ++calls;
  }));
  telemetryQueue()->addCallback(callback);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (calls.load() == 0 && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  EXPECT_EQ(1, calls.load());
  EXPECT_TRUE(background.load());
}

TEST_F(RvizCallbackIsolationTest, ReconnectPreservesQueueSeparation)
{
  auto* queue = telemetryQueue();
  for (int iteration = 0; iteration < 5; ++iteration)
  {
    reconnect();
    EXPECT_EQ(queue, telemetryQueue());
    EXPECT_NE(ros::getGlobalCallbackQueue(), telemetryQueue());
  }
}

TEST_F(RvizCallbackIsolationTest, ActualTelemetrySubscriptionUsesPrivateWorker)
{
  verifyTelemetryDelivery();
}

TEST_F(RvizCallbackIsolationTest, RemovedMapAnnotationIsReplacedByBevCleaningControls)
{
  verifyBevCleaningControls();
}

TEST_F(RvizCallbackIsolationTest, VoiceDraftAndClarificationAreNotExecutableSteps)
{
  verifyVoiceDraftAndClarification();
}

TEST_F(RvizCallbackIsolationTest, RestoresEndpointsForgottenByMaster)
{
  XmlRpc::XmlRpcValue args, response, state;
  const auto name = ros::this_node::getName();
  args[0] = name;
  ASSERT_TRUE(ros::master::execute("getSystemState", args, response, state, false));
  std::set<std::pair<int, std::string>> expected;
  for (int category = 0; category < 2; ++category)
  {
    for (int index = 0; index < state[category].size(); ++index)
    {
      auto& endpoint = state[category][index];
      for (int node = 0; node < endpoint[1].size(); ++node)
      {
        if (static_cast<std::string>(endpoint[1][node]) != name)
          continue;
        expected.emplace(category, static_cast<std::string>(endpoint[0]));
      }
    }
  }
  ASSERT_GT(expected.size(), 10U);
  for (const auto& endpoint : expected)
  {
    XmlRpc::XmlRpcValue unregister_args, payload;
    unregister_args[0] = name;
    unregister_args[1] = endpoint.second;
    unregister_args[2] = ros::XMLRPCManager::instance()->getServerURI();
    ASSERT_TRUE(ros::master::execute(endpoint.first ? "unregisterSubscriber" : "unregisterPublisher",
                                     unregister_args, response, payload, false));
  }
  std::string error;
  ASSERT_TRUE(restoreRosGraphRegistration(&error)) << error;
  ASSERT_TRUE(ros::master::execute("getSystemState", args, response, state, false));
  for (int category = 0; category < 2; ++category)
    for (int index = 0; index < state[category].size(); ++index)
      for (int node = 0; node < state[category][index][1].size(); ++node)
        if (static_cast<std::string>(state[category][index][1][node]) == name)
          expected.erase({category, static_cast<std::string>(state[category][index][0])});
  EXPECT_TRUE(expected.empty());
  verifyTelemetryDelivery();
}

TEST_F(RvizCallbackIsolationTest, ParameterEditingDoesNotRequireMapButRetainsTaskAndFreshnessLocks)
{
  verifyParameterEditingWithoutMapRetainsTaskLocks();
}

TEST_F(RvizCallbackIsolationTest, NavigationTuningPageContainsAllThreeParameterGroups)
{
  QWidget* tuning_page = nullptr;
  for (auto* tabs : window_->findChildren<QTabWidget*>())
    for (int i=0; i<tabs->count(); ++i)
      if (tabs->tabText(i) == QStringLiteral("导航调参")) tuning_page=tabs->widget(i);
  ASSERT_NE(nullptr, tuning_page);
  std::map<QString, int> expected{{QStringLiteral("覆盖规划参数"),10},
                                 {QStringLiteral("转场 TEB"),16},
                                 {QStringLiteral("混合 A*"),11}};
  for (auto* group : tuning_page->findChildren<QGroupBox*>()) {
    auto it=expected.find(group->title());
    if (it != expected.end()) {
      EXPECT_EQ(it->second, group->findChildren<QDoubleSpinBox*>().size());
      expected.erase(it);
    }
  }
  EXPECT_TRUE(expected.empty());
}

TEST_F(RvizCallbackIsolationTest, SurroundSwitchingCoexistsWithNavigationTuning)
{
  verifySurroundAndNavigationTuningCoexist();
}

TEST_F(RvizCallbackIsolationTest, RealRenderingStress)
{
  ASSERT_TRUE(qEnvironmentVariableIsSet("AUTOLABOR_RVIZ_STRESS"));
  runRenderStress();
}
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "rviz_callback_isolation");
  QApplication application(argc, argv);
  application.setOrganizationName("AutolaborTests");
  application.setApplicationName("RvizCallbackIsolation");
  QTemporaryDir settings;
  qputenv("XDG_CONFIG_HOME", settings.path().toUtf8());
  const int result = RUN_ALL_TESTS();
  ros::shutdown();
  return result;
}
