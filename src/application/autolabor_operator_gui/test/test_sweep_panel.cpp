#include <autolabor_operator_gui/sweep_panel.h>

#include <QLabel>
#include <QComboBox>
#include <QLineEdit>
#include <QProcess>
#include <QPushButton>
#include <QSignalSpy>
#include <QSpinBox>
#include <QTest>
#include <QTimer>

using autolabor_operator_gui::SweepPanel;

class SweepPanelTest : public QObject
{
  Q_OBJECT
private Q_SLOTS:
  void init()
  {
    server_.start(QStringLiteral("/usr/bin/python3"),
                  {QStringLiteral(TEST_WORKSPACE "/src/application/autolabor_operator_gui/test/sweep_mock_server.py")});
    QVERIFY(server_.waitForStarted(2000));
    QVERIFY(server_.waitForReadyRead(2000));
    port_ = server_.readLine().trimmed().toInt();
    QVERIFY(port_ > 0);
  }

  void cleanup()
  {
    server_.terminate();
    if (!server_.waitForFinished(2000))
    {
      server_.kill();
      server_.waitForFinished(1000);
    }
  }

  void buttonsAndResponsiveUi()
  {
    SweepPanel panel(QStringLiteral(TEST_WORKSPACE "/scripts/sweep_test.py"));
    configure(panel);
    QSignalSpy states(&panel, &SweepPanel::stateChanged);
    panel.queryStatus();
    QTRY_COMPARE(states.count(), 1);
    QCOMPARE(label(panel)->text(), QStringLiteral("已关闭"));
    auto* start = panel.findChild<QPushButton*>(QStringLiteral("sweepStartButton"));
    auto* stop = panel.findChild<QPushButton*>(QStringLiteral("sweepStopButton"));
    auto* query = panel.findChild<QPushButton*>(QStringLiteral("sweepQueryButton"));
    start->click();
    QVERIFY(!start->isEnabled());
    QVERIFY(!query->isEnabled());
    QVERIFY(stop->isEnabled());
    bool timer_fired = false;
    QTimer::singleShot(10, &panel, [&timer_fired]() { timer_fired = true; });
    QTest::qWait(40);
    QVERIFY(timer_fired);
    QTRY_COMPARE(states.count(), 2);
    QCOMPARE(label(panel)->text(), QStringLiteral("已开启"));
    start->click();
    QTRY_COMPARE(states.count(), 3);
    QCOMPARE(label(panel)->text(), QStringLiteral("已开启"));
    stop->click();
    QTRY_COMPARE(states.count(), 4);
    QCOMPARE(label(panel)->text(), QStringLiteral("已关闭"));
    QVERIFY(query->isEnabled());
  }

  void stopDuringStartIsExecutedAfterCurrentRequest()
  {
    SweepPanel panel(QStringLiteral(TEST_WORKSPACE "/scripts/sweep_test.py"));
    configure(panel);
    QSignalSpy states(&panel, &SweepPanel::stateChanged);
    panel.findChild<QPushButton*>(QStringLiteral("sweepStartButton"))->click();
    panel.findChild<QPushButton*>(QStringLiteral("sweepStopButton"))->click();
    QVERIFY(panel.findChild<QLabel*>(QStringLiteral("sweepResult"))->text().contains(QStringLiteral("停止请求")));
    QTRY_COMPARE(states.count(), 2);
    QCOMPARE(label(panel)->text(), QStringLiteral("已关闭"));
    QVERIFY(panel.findChild<QPushButton*>(QStringLiteral("sweepStartButton"))->isEnabled());
  }

  void communicationFailureIsVisibleAndCanBeRetried()
  {
    server_.terminate();
    QVERIFY(server_.waitForFinished(2000));
    SweepPanel panel(QStringLiteral(TEST_WORKSPACE "/scripts/sweep_test.py"));
    configure(panel);
    QSignalSpy states(&panel, &SweepPanel::stateChanged);
    panel.queryStatus();
    QTRY_COMPARE(states.count(), 1);
    QCOMPARE(label(panel)->text(), QStringLiteral("状态未确认"));
    QVERIFY(states.at(0).at(2).toBool());
    QVERIFY(panel.findChild<QPushButton*>(QStringLiteral("sweepQueryButton"))->isEnabled());
  }

  void hydraulicButtonsDefaultToTwoSecondsAndPreserveSweepStatus()
  {
    SweepPanel panel(QStringLiteral(TEST_WORKSPACE "/scripts/sweep_test.py"));
    configure(panel);
    QSignalSpy states(&panel, &SweepPanel::stateChanged);
    QSignalSpy events(&panel, &SweepPanel::operationEvent);
    panel.queryStatus();
    QTRY_COMPARE(states.count(), 1);
    auto* duration = panel.findChild<QComboBox*>(QStringLiteral("hydraulicDuration"));
    auto* extend = panel.findChild<QPushButton*>(QStringLiteral("hydraulicExtendButton"));
    auto* retract = panel.findChild<QPushButton*>(QStringLiteral("hydraulicRetractButton"));
    auto* result = panel.findChild<QLabel*>(QStringLiteral("hydraulicResult"));
    QCOMPARE(duration->count(), 2);
    QCOMPARE(duration->currentData().toInt(), 2);
    extend->click();
    QVERIFY(!extend->isEnabled());
    QVERIFY(!retract->isEnabled());
    QVERIFY(!duration->isEnabled());
    bool responsive = false;
    QTimer::singleShot(10, &panel, [&responsive]() { responsive = true; });
    QTest::qWait(40);
    QVERIFY(responsive);
    QTRY_VERIFY(extend->isEnabled());
    QVERIFY(result->text().contains(QStringLiteral("升出 2 秒指令已接收")));
    QVERIFY(!events.last().at(1).toBool());
    QCOMPARE(states.count(), 1);
    QCOMPARE(label(panel)->text(), QStringLiteral("已关闭"));
    duration->setCurrentIndex(1);
    retract->click();
    QTRY_VERIFY(retract->isEnabled());
    QVERIFY(result->text().contains(QStringLiteral("回收 3 秒指令已接收")));
    QCOMPARE(states.count(), 1);
    const auto last_hydraulic = result->text();
    panel.queryStatus();
    QTRY_COMPARE(states.count(), 2);
    QCOMPARE(result->text(), last_hydraulic);
    QCOMPARE(label(panel)->text(), QStringLiteral("已关闭"));
  }

  void sweepStopDuringHydraulicRunsAfterThePulse()
  {
    SweepPanel panel(QStringLiteral(TEST_WORKSPACE "/scripts/sweep_test.py"));
    configure(panel);
    QSignalSpy states(&panel, &SweepPanel::stateChanged);
    panel.findChild<QPushButton*>(QStringLiteral("sweepStartButton"))->click();
    QTRY_COMPARE(states.count(), 1);
    QCOMPARE(label(panel)->text(), QStringLiteral("已开启"));
    panel.findChild<QPushButton*>(QStringLiteral("hydraulicExtendButton"))->click();
    auto* stop = panel.findChild<QPushButton*>(QStringLiteral("sweepStopButton"));
    QVERIFY(stop->isEnabled());
    stop->click();
    QTRY_COMPARE(states.count(), 2);
    QCOMPARE(label(panel)->text(), QStringLiteral("已关闭"));
    QVERIFY(panel.findChild<QLabel*>(QStringLiteral("hydraulicResult"))->text().contains(QStringLiteral("升出 2 秒指令已接收")));
    QVERIFY(panel.findChild<QPushButton*>(QStringLiteral("hydraulicExtendButton"))->isEnabled());
  }

  void hydraulicFailureDoesNotClaimSweepStateOrRodPosition()
  {
    SweepPanel panel(QStringLiteral(TEST_WORKSPACE "/scripts/sweep_test.py"));
    configure(panel);
    QSignalSpy states(&panel, &SweepPanel::stateChanged);
    QSignalSpy events(&panel, &SweepPanel::operationEvent);
    panel.queryStatus();
    QTRY_COMPARE(states.count(), 1);
    server_.terminate();
    QVERIFY(server_.waitForFinished(2000));
    auto* extend = panel.findChild<QPushButton*>(QStringLiteral("hydraulicExtendButton"));
    extend->click();
    QTRY_VERIFY(extend->isEnabled());
    QVERIFY(events.last().at(1).toBool());
    QCOMPARE(states.count(), 1);
    QCOMPARE(label(panel)->text(), QStringLiteral("已关闭"));
    QVERIFY(panel.findChild<QLabel*>(QStringLiteral("hydraulicResult"))->text().contains(QStringLiteral("无法连接")));
  }

private:
  void configure(SweepPanel& panel)
  {
    panel.findChild<QLineEdit*>(QStringLiteral("sweepAddress"))->setText(QStringLiteral("127.0.0.1"));
    panel.findChild<QSpinBox*>(QStringLiteral("sweepPort"))->setValue(port_);
  }
  QLabel* label(SweepPanel& panel)
  {
    return panel.findChild<QLabel*>(QStringLiteral("sweepState"));
  }
  QProcess server_;
  int port_ = 0;
};

QTEST_MAIN(SweepPanelTest)
#include "test_sweep_panel.moc"
