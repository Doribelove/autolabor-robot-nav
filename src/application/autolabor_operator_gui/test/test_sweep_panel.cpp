#include <autolabor_operator_gui/sweep_panel.h>

#include <QLabel>
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
