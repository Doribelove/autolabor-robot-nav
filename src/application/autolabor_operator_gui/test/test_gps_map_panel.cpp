#include <autolabor_operator_gui/gps_map_panel.h>
#include <QApplication>
#include <QDoubleSpinBox>
#include <QGraphicsView>
#include <QCheckBox>
#include <QPushButton>
#include <QSignalSpy>
#include <QTableWidget>
#include <QtTest>

using autolabor_operator_gui::KeepoutPanel;
using autolabor_operator_gui::FodTargetsPanel;

class GpsMapPanelTest: public QObject
{
  Q_OBJECT
private Q_SLOTS:
  void editingManualZonesPreservesAirportUntilExplicitRemoval()
  {
    KeepoutPanel panel;
    QJsonArray ring{QJsonArray{104.,30.},QJsonArray{104.01,30.},QJsonArray{104.01,30.01},QJsonArray{104.,30.}};
    QJsonObject airport{{"name","airport"},{"geometry",QJsonObject{{"type","Polygon"},{"coordinates",QJsonArray{ring}}}}};
    panel.loadConfiguration({{"enabled",true},{"clearance_m",1.},{"polygons",QJsonArray()},{"airport",airport}});
    panel.resize(1300,850);panel.show();QTest::qWait(20);
    // Loading while the tab is hidden must still fit the map when shown.
    QVERIFY(panel.findChild<QGraphicsView*>()->transform().m11()>.05);
    panel.findChild<QPushButton*>("keepoutAddZone")->click();
    QCOMPARE(panel.configuration()["airport"].toObject(),airport);
    panel.findChild<QCheckBox*>("keepoutEnabled")->setChecked(false);
    QCOMPARE(panel.configuration()["airport"].toObject(),airport);
    panel.findChild<QPushButton*>("keepoutClearAirport")->click();
    QVERIFY(!panel.configuration().contains("airport"));
    QCOMPARE(panel.configuration()["polygons"].toArray().size(),1);
  }

  void editVerticesAndRemovePolygon()
  {
    KeepoutPanel panel;
    QCOMPARE(panel.configuration()["clearance_m"].toDouble(),1.0);
    QVERIFY(!panel.configuration()["enabled"].toBool());
    panel.findChild<QPushButton*>("keepoutAddZone")->click();
    for (int i=0;i<3;++i)
    {
      panel.findChild<QDoubleSpinBox*>("keepoutLongitude")->setValue(104.+i*.001);
      panel.findChild<QDoubleSpinBox*>("keepoutLatitude")->setValue(30.+i*.001);
      panel.findChild<QPushButton*>("keepoutAddPoint")->click();
    }
    auto* table=panel.findChild<QTableWidget*>("keepoutPoints");
    QCOMPARE(table->rowCount(),3);
    table->selectRow(2);
    panel.findChild<QPushButton*>("keepoutPointUp")->click();
    auto points=panel.configuration()["polygons"].toArray()[0].toObject()["points"].toArray();
    QCOMPARE(points[1].toArray()[1].toDouble(),104.002);
    panel.findChild<QPushButton*>("keepoutDeletePoint")->click();
    QCOMPARE(table->rowCount(),2);
    panel.findChild<QPushButton*>("keepoutDeleteZone")->click();
    QVERIFY(panel.configuration()["polygons"].toArray().isEmpty());
  }

  void importingAndSelectingFodDoesNotStartNavigation()
  {
    FodTargetsPanel panel;
    QSignalSpy requests(&panel,&FodTargetsPanel::navigationRequested);
    QJsonArray points{
      QJsonObject{{"name","pt0"},{"longitude",104.3892865},{"latitude",30.29387435}},
      QJsonObject{{"name","pt1"},{"longitude",104.3898634},{"latitude",30.29726472}},
      QJsonObject{{"name","pt2"},{"longitude",104.3904909},{"latitude",30.30145079}}};
    panel.loadPoints(points);
    auto* table=panel.findChild<QTableWidget*>("fodTargetsTable");
    QCOMPARE(table->rowCount(),3);
    QCOMPARE(requests.count(),0);
    table->selectRow(2);
    QCOMPARE(requests.count(),0);
    panel.findChild<QPushButton*>("fodTargetsSend")->click();
    QCOMPARE(requests.count(),1);
    QCOMPARE(requests.at(0).at(0).toString(),QString("pt2"));
    QCOMPARE(requests.at(0).at(1).toDouble(),30.30145079);
    QCOMPARE(requests.at(0).at(2).toDouble(),104.3904909);
    KeepoutPanel keepout;
    QVERIFY(keepout.configuration()["polygons"].toArray().isEmpty());
  }
};

int main(int argc,char** argv)
{
  ros::init(argc,argv,"test_gps_map_panel",ros::init_options::AnonymousName);
  QApplication app(argc,argv);
  GpsMapPanelTest test;
  return QTest::qExec(&test,argc,argv);
}
#include "test_gps_map_panel.moc"
