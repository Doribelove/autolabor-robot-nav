#include <autolabor_operator_gui/zed_camera_panel.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <ros/ros.h>
#include <QApplication>
#include <QCheckBox>
#include <QElapsedTimer>
#include <QLabel>
#include <QPushButton>
#include <QSignalSpy>
#include <QSpinBox>
#include <QtTest>
#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

using autolabor_operator_gui::ZedCameraPanel;

class ZedCameraPanelTest : public QObject
{
  Q_OBJECT
private:
  std::unique_ptr<ros::NodeHandle> node_;
  std::unique_ptr<ros::AsyncSpinner> spinner_;
  ros::Publisher publisher_;
  ros::ServiceServer service_;
  dynamic_reconfigure::Config config_;
  dynamic_reconfigure::Config last_request_;
  std::mutex mutex_;
  std::atomic<int> calls_{0};

  bool configure(dynamic_reconfigure::Reconfigure::Request& request,
                 dynamic_reconfigure::Reconfigure::Response& response)
  {
    ros::WallDuration(0.15).sleep();
    std::lock_guard<std::mutex> lock(mutex_);
    last_request_ = request.config;
    for (const auto& wanted : request.config.ints)
      for (auto& current : config_.ints)
        if (wanted.name == current.name)
          current.value = wanted.value;
    for (const auto& wanted : request.config.bools)
      for (auto& current : config_.bools)
        if (wanted.name == current.name)
          current.value = wanted.value;
    response.config = config_;
    publisher_.publish(config_);
    ++calls_;
    return true;
  }

private Q_SLOTS:
  void initTestCase()
  {
    node_.reset(new ros::NodeHandle());
    spinner_.reset(new ros::AsyncSpinner(2));
    spinner_->start();
    publisher_ = node_->advertise<dynamic_reconfigure::Config>("/mock_zed/parameter_updates", 1, true);
    service_ = node_->advertiseService("/mock_zed/set_parameters", &ZedCameraPanelTest::configure, this);
    node_->setParam("/mock_zed/general/grab_resolution", "HD720");
    node_->setParam("/mock_zed/general/grab_frame_rate", 60);
    node_->setParam("/mock_zed/general/pub_frame_rate", 30.0);
    node_->setParam("/mock_zed/depth/depth_mode", "ULTRA");
  }

  void init()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    calls_ = 0;
    config_ = dynamic_reconfigure::Config();
    for (const auto& item : std::map<std::string, int>{
             {"brightness", 4}, {"contrast", 4}, {"saturation", 4}, {"sharpness", 4},
             {"gamma", 8}, {"exposure", 65}, {"gain", 30}, {"whitebalance_temperature", 50},
             {"depth_confidence", 50}, {"depth_texture_conf", 100}})
    {
      dynamic_reconfigure::IntParameter value;
      value.name = item.first;
      value.value = item.second;
      config_.ints.push_back(value);
    }
    for (const char* name : {"auto_exposure_gain", "auto_whitebalance"})
    {
      dynamic_reconfigure::BoolParameter value;
      value.name = name;
      value.value = true;
      config_.bools.push_back(value);
    }
    publisher_.publish(config_);
  }

  void queryReadsNativeUnitsWithoutWriting()
  {
    ZedCameraPanel panel("/mock_zed");
    panel.setOnline(true);
    QSignalSpy complete(&panel, &ZedCameraPanel::operationEvent);
    panel.findChild<QPushButton*>("zedQuery")->click();
    QTRY_COMPARE(complete.count(), 1);
    QVERIFY(!complete.at(0).at(1).toBool());
    QCOMPARE(calls_.load(), 0);
    QCOMPARE(panel.findChild<QSpinBox*>("exposure")->value(), 65);
    QCOMPARE(panel.findChild<QSpinBox*>("whitebalance_temperature")->value(), 50);
    QVERIFY(!panel.findChild<QSpinBox*>("exposure")->isEnabled());
    QVERIFY(panel.findChild<QLabel*>("zedAcquisition")->text().contains("60 FPS"));
  }

  void manualChangesUseTheZedServiceAndKeepUiResponsive()
  {
    ZedCameraPanel panel("/mock_zed");
    panel.setOnline(true);
    QSignalSpy complete(&panel, &ZedCameraPanel::operationEvent);
    panel.findChild<QPushButton*>("zedQuery")->click();
    QTRY_COMPARE(complete.count(), 1);
    panel.findChild<QCheckBox*>("zedAutoExposure")->setChecked(false);
    panel.findChild<QCheckBox*>("zedAutoWhitebalance")->setChecked(false);
    QVERIFY(panel.findChild<QSpinBox*>("gain")->isEnabled());
    panel.findChild<QSpinBox*>("exposure")->setValue(25);
    panel.findChild<QSpinBox*>("gain")->setValue(45);
    panel.findChild<QSpinBox*>("whitebalance_temperature")->setValue(55);
    QElapsedTimer elapsed;
    elapsed.start();
    panel.findChild<QPushButton*>("zedApply")->click();
    QVERIFY(elapsed.elapsed() < 100);
    QVERIFY(!panel.findChild<QPushButton*>("zedApply")->isEnabled());
    QTRY_COMPARE(complete.count(), 2);
    QVERIFY(!complete.at(1).at(1).toBool());
    QCOMPARE(calls_.load(), 1);
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& item : config_.ints)
    {
      if (item.name == "exposure") QCOMPARE(item.value, 25);
      if (item.name == "gain") QCOMPARE(item.value, 45);
      if (item.name == "whitebalance_temperature") QCOMPARE(item.value, 55);
    }
    for (const auto& item : config_.bools) QVERIFY(!item.value);
  }

  void automaticModeDoesNotOverwriteOtherSettings()
  {
    ZedCameraPanel panel("/mock_zed");
    panel.setOnline(true);
    QSignalSpy complete(&panel, &ZedCameraPanel::operationEvent);
    panel.findChild<QPushButton*>("zedAutomatic")->click();
    QTRY_COMPARE(complete.count(), 1);
    QVERIFY(!complete.at(0).at(1).toBool());
    QCOMPARE(calls_.load(), 1);
    std::lock_guard<std::mutex> lock(mutex_);
    QVERIFY(last_request_.ints.empty());
    QCOMPARE(last_request_.bools.size(), std::size_t(2));
    for (const auto& item : last_request_.bools) QVERIFY(item.value);
  }

  void missingCameraReportsFailureWithoutBlocking()
  {
    ZedCameraPanel panel("/missing_zed");
    panel.setOnline(true);
    QSignalSpy complete(&panel, &ZedCameraPanel::operationEvent);
    QElapsedTimer elapsed;
    elapsed.start();
    panel.findChild<QPushButton*>("zedQuery")->click();
    QVERIFY(elapsed.elapsed() < 100);
    QTRY_COMPARE(complete.count(), 1);
    QVERIFY(complete.at(0).at(1).toBool());
    QVERIFY(!panel.findChild<QPushButton*>("zedApply")->isEnabled());
    QVERIFY(panel.findChild<QPushButton*>("zedQuery")->isEnabled());
    QCOMPARE(calls_.load(), 0);
  }

  void incompleteDriverSchemaDoesNotEnableWrites()
  {
    publisher_.publish(dynamic_reconfigure::Config());
    ZedCameraPanel panel("/mock_zed");
    panel.setOnline(true);
    QSignalSpy complete(&panel, &ZedCameraPanel::operationEvent);
    panel.findChild<QPushButton*>("zedQuery")->click();
    QTRY_COMPARE(complete.count(), 1);
    QVERIFY(complete.at(0).at(1).toBool());
    QVERIFY(!panel.findChild<QPushButton*>("zedApply")->isEnabled());
    QCOMPARE(calls_.load(), 0);
  }

  void cleanupTestCase()
  {
    service_.shutdown();
    publisher_.shutdown();
    spinner_->stop();
    node_.reset();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_zed_camera_panel");
  QApplication application(argc, argv);
  ZedCameraPanelTest test;
  // rostest supplies a GoogleTest XML switch. Emit Qt's compatible xUnit
  // document to that path while retaining readable output in the ROS log.
  std::vector<QByteArray> arguments;
  for (int index = 0; index < argc; ++index)
  {
    const QByteArray value(argv[index]);
    const QByteArray prefix("--gtest_output=xml:");
    if (value.startsWith(prefix))
    {
      arguments.push_back("-o");
      arguments.push_back(value.mid(prefix.size()) + ",xunitxml");
      arguments.push_back("-o");
      arguments.push_back("-,txt");
    }
    else
      arguments.push_back(value);
  }
  std::vector<char*> pointers;
  for (auto& argument : arguments)
    pointers.push_back(argument.data());
  return QTest::qExec(&test, static_cast<int>(pointers.size()), pointers.data());
}

#include "test_zed_camera_panel.moc"
