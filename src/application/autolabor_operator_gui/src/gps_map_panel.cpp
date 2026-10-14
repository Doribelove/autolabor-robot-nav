#include <autolabor_operator_gui/gps_map_panel.h>
#include <autolabor_gps_keepout/KeepoutCommand.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFutureWatcher>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsTextItem>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPainterPath>
#include <QResizeEvent>
#include <QPushButton>
#include <QSignalBlocker>
#include <QStandardPaths>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QtConcurrent/QtConcurrentRun>
#include <cmath>
#include <functional>

namespace autolabor_operator_gui
{
namespace
{
class MapPreview : public QGraphicsView
{
public:
  explicit MapPreview(QWidget* parent):QGraphicsView(parent)
  {
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  }
protected:
  void resizeEvent(QResizeEvent* event) override
  {
    QGraphicsView::resizeEvent(event);
    if(scene()&&!sceneRect().isEmpty())fitInView(sceneRect(),Qt::KeepAspectRatio);
  }
};

struct Reply { bool success=false; QString message; QJsonValue data; };
void request(QObject* owner,const QString& command,const QJsonValue& payload,
             std::function<void(const Reply&)> finished)
{
  auto* watcher=new QFutureWatcher<Reply>(owner);
  QObject::connect(watcher,&QFutureWatcher<Reply>::finished,owner,[watcher,finished](){
    finished(watcher->result()); watcher->deleteLater();
  });
  watcher->setFuture(QtConcurrent::run([command,payload](){
    Reply result;
    try
    {
      ros::NodeHandle node;
      auto client=node.serviceClient<autolabor_gps_keepout::KeepoutCommand>("/gps/keepout/command");
      if (!client.waitForExistence(ros::Duration(1.0)))
      { result.message=QStringLiteral("GPS 地图模块尚未就绪");return result; }
      autolabor_gps_keepout::KeepoutCommand call;
      call.request.command=command.toStdString();
      const QJsonDocument data=payload.isArray()?QJsonDocument(payload.toArray()):QJsonDocument(payload.toObject());
      call.request.payload=data.toJson(QJsonDocument::Compact).toStdString();
      if (!client.call(call)) { result.message=QStringLiteral("GPS 地图操作未完成");return result; }
      result.success=call.response.success;
      result.message=QString::fromStdString(call.response.message);
      auto json=QJsonDocument::fromJson(QByteArray::fromStdString(call.response.config_json));
      result.data=json.isArray()?QJsonValue(json.array()):QJsonValue(json.object());
    }
    catch(const std::exception& e) { result.message=QString::fromUtf8(e.what()); }
    return result;
  }));
}
QPointF local(double lat,double lon,double lat0,double lon0)
{
  const double radians=M_PI/180.;
  return QPointF(6378137.*(lon-lon0)*radians*std::cos(lat0*radians),
                 -6378137.*(lat-lat0)*radians);
}
QPushButton* button(const QString& text,const char* name,QBoxLayout* layout,QWidget* parent)
{
  auto* b=new QPushButton(text,parent); b->setObjectName(QString::fromLatin1(name)); layout->addWidget(b);return b;
}
void tableSetup(QTableWidget* table,const QStringList& labels)
{
  table->setColumnCount(labels.size());table->setHorizontalHeaderLabels(labels);
  table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  table->setSelectionBehavior(QAbstractItemView::SelectRows);
  table->setSelectionMode(QAbstractItemView::SingleSelection);
  table->setEditTriggers(QAbstractItemView::NoEditTriggers);
}
void item(QTableWidget* table,int row,int col,const QString& value)
{ table->setItem(row,col,new QTableWidgetItem(value)); }
void label(QGraphicsScene* scene,const QPointF& point,const QString& text,const QColor& color)
{
  auto* dot=scene->addEllipse(-4,-4,8,8,QPen(color),QBrush(color));
  dot->setFlag(QGraphicsItem::ItemIgnoresTransformations);dot->setPos(point);
  auto* t=scene->addText(text);t->setDefaultTextColor(color);
  t->setFlag(QGraphicsItem::ItemIgnoresTransformations);t->setPos(point+QPointF(.4,.4));
}
void preview(QGraphicsView* view,const QJsonArray& zones,const QJsonArray& targets,
             bool have_fix,double lat,double lon,const QJsonObject& airport=QJsonObject())
{
  auto* scene=view->scene();scene->clear();
  if (!have_fix && !targets.isEmpty())
  {auto p=targets[0].toObject();lat=p["latitude"].toDouble();lon=p["longitude"].toDouble();}
  else if (!have_fix && !zones.isEmpty())
  {auto a=zones[0].toObject()["points"].toArray();if(!a.isEmpty()){lat=a[0].toArray()[0].toDouble();lon=a[0].toArray()[1].toDouble();}}
  else if (!have_fix && !airport.isEmpty())
  {
    auto g=airport["geometry"].toObject();auto a=g["coordinates"].toArray();
    if(g["type"].toString()=="MultiPolygon"&&!a.isEmpty())a=a[0].toArray();
    if(!a.isEmpty()&&!a[0].toArray().isEmpty())
    {auto p=a[0].toArray()[0].toArray();lon=p[0].toDouble();lat=p[1].toDouble();}
  }
  if(!airport.isEmpty())
  {
    auto g=airport["geometry"].toObject();auto polygons=g["coordinates"].toArray();
    if(g["type"].toString()=="Polygon")polygons=QJsonArray{polygons};
    QPainterPath area;area.setFillRule(Qt::OddEvenFill);
    for(auto polygon:polygons)
      for(auto ring:polygon.toArray())
      {
        QPolygonF boundary;
        for(auto coordinate:ring.toArray())
        {auto p=coordinate.toArray();boundary<<local(p[1].toDouble(),p[0].toDouble(),lat,lon);}
        area.addPolygon(boundary);area.closeSubpath();
      }
    scene->addPath(area,QPen(QColor("#168879"),0),QBrush(QColor(69,167,146,70)));
    QPainterPath roads;
    for(auto line:airport["context"].toObject()["coordinates"].toArray())
    {
      bool first=true;
      for(auto coordinate:line.toArray())
      {auto p=coordinate.toArray();auto xy=local(p[1].toDouble(),p[0].toDouble(),lat,lon);if(first)roads.moveTo(xy);else roads.lineTo(xy);first=false;}
    }
    scene->addPath(roads,QPen(QColor("#97a6b0"),0));
  }
  for (const auto& value:zones)
  {
    auto zone=value.toObject();QPolygonF polygon;
    for (const auto& p:zone["points"].toArray())
    {auto a=p.toArray();polygon<<local(a[0].toDouble(),a[1].toDouble(),lat,lon);}
    scene->addPolygon(polygon,QPen(QColor("#dc732e"),0),QBrush(QColor(244,155,68,55)));
    for(int i=0;i<polygon.size();++i)label(scene,polygon[i],QString::number(i+1),QColor("#b95914"));
  }
  for(const auto& v:targets)
  {auto p=v.toObject();label(scene,local(p["latitude"].toDouble(),p["longitude"].toDouble(),lat,lon),p["name"].toString(),QColor("#007f9b"));}
  if(have_fix)label(scene,QPointF(0,0),QStringLiteral("车辆"),QColor("#148348"));
  QRectF bounds=scene->itemsBoundingRect();
  if(bounds.isEmpty())bounds=QRectF(-10,-10,20,20);
  const double pad=std::max(3.,std::max(bounds.width(),bounds.height())*.12);
  view->setSceneRect(bounds.adjusted(-pad,-pad,pad,pad));view->fitInView(view->sceneRect(),Qt::KeepAspectRatio);
}
}

KeepoutPanel::KeepoutPanel(QWidget* parent):QWidget(parent)
{
  setObjectName("gpsKeepoutPanel");auto* root=new QVBoxLayout(this);
  auto* note=new QLabel(QStringLiteral("可导入机场地图自动建立通行范围，或手动添加禁入多边形。按 1.6 × 0.8 米车体预留距离。绿色为机场可行区域，内部空洞及外部禁行；编辑后点击“应用到导航”。"),this);
  note->setWordWrap(true);root->addWidget(note);
  auto* options=new QHBoxLayout();root->addLayout(options);
  enabled_=new QCheckBox(QStringLiteral("启用通行范围 / 禁入区"),this);enabled_->setObjectName("keepoutEnabled");options->addWidget(enabled_);
  options->addWidget(new QLabel(QStringLiteral("车体外沿距离"),this));clearance_=new QDoubleSpinBox(this);
  clearance_->setObjectName("keepoutClearance");clearance_->setRange(0,20);clearance_->setValue(1);clearance_->setSuffix(QStringLiteral(" 米"));options->addWidget(clearance_);
  auto* airport_row=new QHBoxLayout();root->addLayout(airport_row);
  auto* import_airport=button(QStringLiteral("导入机场地图 ZIP"),"keepoutImportAirport",airport_row,this);
  auto* clear_airport=button(QStringLiteral("移除机场地图"),"keepoutClearAirport",airport_row,this);
  airport_status_=new QLabel(QStringLiteral("未导入机场地图"),this);airport_status_->setWordWrap(true);airport_row->addWidget(airport_status_,1);
  auto* zones=new QHBoxLayout();root->addLayout(zones);zones_box_=new QComboBox(this);zones_box_->setObjectName("keepoutZones");zones->addWidget(zones_box_,1);
  auto* add_zone=button(QStringLiteral("新增多边形"),"keepoutAddZone",zones,this);
  auto* remove_zone=button(QStringLiteral("删除多边形"),"keepoutDeleteZone",zones,this);
  auto* coord=new QHBoxLayout();root->addLayout(coord);
  coord->addWidget(new QLabel(QStringLiteral("经度"),this));longitude_=new QDoubleSpinBox(this);longitude_->setRange(-180,180);longitude_->setDecimals(8);longitude_->setObjectName("keepoutLongitude");coord->addWidget(longitude_);
  coord->addWidget(new QLabel(QStringLiteral("纬度"),this));latitude_=new QDoubleSpinBox(this);latitude_->setRange(-85,85);latitude_->setDecimals(8);latitude_->setObjectName("keepoutLatitude");coord->addWidget(latitude_);
  auto* add_point=button(QStringLiteral("添加顶点"),"keepoutAddPoint",coord,this);
  auto* current=button(QStringLiteral("填入车辆 GPS"),"keepoutCurrent",coord,this);
  auto* middle=new QHBoxLayout();root->addLayout(middle,1);
  points_=new QTableWidget(this);points_->setObjectName("keepoutPoints");tableSetup(points_,{QStringLiteral("顶点"),QStringLiteral("经度"),QStringLiteral("纬度")});middle->addWidget(points_,1);
  preview_=new MapPreview(this);preview_->setScene(new QGraphicsScene(preview_));preview_->setMinimumSize(350,220);middle->addWidget(preview_,1);
  auto* editing=new QHBoxLayout();root->addLayout(editing);
  auto* remove=button(QStringLiteral("删除选中顶点"),"keepoutDeletePoint",editing,this);
  auto* up=button(QStringLiteral("顶点上移"),"keepoutPointUp",editing,this);
  auto* down=button(QStringLiteral("顶点下移"),"keepoutPointDown",editing,this);
  auto* apply=button(QStringLiteral("应用到导航"),"keepoutApply",editing,this);
  apply->setStyleSheet(QStringLiteral("font-weight:bold;background:#19846b;color:white;"));
  auto* files=new QHBoxLayout();root->addLayout(files);
  auto* save=button(QStringLiteral("保存 GPS 禁入区"),"keepoutSave",files,this);
  auto* load=button(QStringLiteral("加载 GPS 禁入区"),"keepoutLoad",files,this);
  resolution_=new QDoubleSpinBox(this);resolution_->setRange(.05,2);resolution_->setDecimals(2);resolution_->setValue(.1);resolution_->setSuffix(QStringLiteral(" 米/格"));files->addWidget(resolution_);
  auto* export_map=button(QStringLiteral("导出禁入区栅格地图"),"keepoutExport",files,this);
  status_=new QLabel(QStringLiteral("尚未连接 GPS 地图模块"),this);status_->setObjectName("keepoutStatus");status_->setWordWrap(true);root->addWidget(status_);
  auto* map_timer=new QTimer(this);map_timer->setInterval(1000);
  connect(map_timer,&QTimer::timeout,this,[this](){if(isVisible()&&have_fix_)preview(preview_,zones_,{},true,fix_lat_,fix_lon_,airport_);});map_timer->start();
  connect(enabled_,&QCheckBox::toggled,this,[this](){changed();});
  connect(clearance_,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,[this](){changed();});
  connect(import_airport,&QPushButton::clicked,this,[this](){auto p=QFileDialog::getOpenFileName(this,QStringLiteral("导入机场 Shapefile 压缩包"),QStandardPaths::writableLocation(QStandardPaths::DesktopLocation),"ZIP (*.zip)");if(!p.isEmpty())run("airport_import",{{"path",p},{"config",configuration()}});});
  connect(clear_airport,&QPushButton::clicked,this,[this](){airport_=QJsonObject();airport_status_->setText(QStringLiteral("未导入机场地图"));showPoints();changed();});
  connect(zones_box_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,&KeepoutPanel::selectZone);
  connect(add_zone,&QPushButton::clicked,this,[this](){
    QJsonObject zone{{"name",QStringLiteral("禁区 %1").arg(zones_.size()+1)},{"points",QJsonArray()}};
    zones_.append(zone);zones_box_->addItem(zone["name"].toString());zones_box_->setCurrentIndex(zones_.size()-1);changed();
  });
  connect(remove_zone,&QPushButton::clicked,this,[this](){if(current_<0)return;zones_.removeAt(current_);QSignalBlocker block(zones_box_);zones_box_->removeItem(current_);current_=zones_box_->currentIndex();showPoints();changed();});
  connect(add_point,&QPushButton::clicked,this,[this](){if(current_<0){status_->setText(QStringLiteral("请先新增多边形"));return;}auto a=zones_[current_].toObject()["points"].toArray();a.append(QJsonArray{latitude_->value(),longitude_->value()});changePoints(a);});
  connect(current,&QPushButton::clicked,this,[this](){if(have_fix_){latitude_->setValue(fix_lat_);longitude_->setValue(fix_lon_);}});
  connect(remove,&QPushButton::clicked,this,[this](){int row=points_->currentRow();if(current_<0||row<0)return;auto a=zones_[current_].toObject()["points"].toArray();a.removeAt(row);changePoints(a);});
  auto move=[this](int step){int row=points_->currentRow();if(current_<0||row<0)return;auto a=zones_[current_].toObject()["points"].toArray();int to=row+step;if(to<0||to>=a.size())return;const QJsonValue v=a.at(row);a[row]=a.at(to);a[to]=v;changePoints(a);points_->selectRow(to);};
  connect(up,&QPushButton::clicked,this,[move](){move(-1);});connect(down,&QPushButton::clicked,this,[move](){move(1);});
  connect(apply,&QPushButton::clicked,this,[this](){run("apply",configuration());});
  connect(save,&QPushButton::clicked,this,[this](){auto p=QFileDialog::getSaveFileName(this,QStringLiteral("保存 GPS 禁入区"),QStandardPaths::writableLocation(QStandardPaths::DesktopLocation)+"/gps_keepout.json","JSON (*.json)");if(!p.isEmpty())run("save",{{"path",p},{"config",configuration()}});});
  connect(load,&QPushButton::clicked,this,[this](){auto p=QFileDialog::getOpenFileName(this,QStringLiteral("加载 GPS 禁入区"),QString(),"JSON (*.json)");if(!p.isEmpty())run("load",{{"path",p}});});
  connect(export_map,&QPushButton::clicked,this,[this](){auto p=QFileDialog::getSaveFileName(this,QStringLiteral("导出禁入区栅格（PGM + YAML + GPS JSON）"),QStandardPaths::writableLocation(QStandardPaths::DesktopLocation)+"/gps_keepout_map.yaml","YAML (*.yaml)");if(!p.isEmpty())run("export",{{"path",p},{"config",configuration()},{"resolution",resolution_->value()}});});
}
QJsonObject KeepoutPanel::configuration() const
{
  QJsonObject config{{"version",1},{"enabled",enabled_->isChecked()},{"clearance_m",clearance_->value()},{"polygons",zones_}};
  if(!airport_.isEmpty())config["airport"]=airport_;
  return config;
}
void KeepoutPanel::loadConfiguration(const QJsonObject& c)
{
  QSignalBlocker b1(enabled_),b2(clearance_),b3(zones_box_);
  enabled_->setChecked(c["enabled"].toBool());clearance_->setValue(c["clearance_m"].toDouble(1));
  airport_=c["airport"].toObject();
  airport_status_->setText(airport_.isEmpty()?QStringLiteral("未导入机场地图"):QStringLiteral("%1 · %2 万平方米").arg(airport_.value("name").toString()).arg(airport_.value("area_m2").toDouble()/10000.,0,'f',1));
  if(!airport_.isEmpty())resolution_->setValue(.5);
  zones_=c["polygons"].toArray();zones_box_->clear();for(auto z:zones_)zones_box_->addItem(z.toObject()["name"].toString());
  current_=zones_.isEmpty()?-1:0;zones_box_->setCurrentIndex(current_);showPoints();loaded_=true;dirty_=false;
}
void KeepoutPanel::selectZone(int row){current_=row;showPoints();}
void KeepoutPanel::showPoints()
{
  auto a=current_>=0&&current_<zones_.size()?zones_[current_].toObject()["points"].toArray():QJsonArray();
  points_->setRowCount(a.size());for(int i=0;i<a.size();++i){auto p=a[i].toArray();item(points_,i,0,QString::number(i+1));item(points_,i,1,QString::number(p[1].toDouble(),'f',8));item(points_,i,2,QString::number(p[0].toDouble(),'f',8));}
  preview(preview_,zones_,{},have_fix_,fix_lat_,fix_lon_,airport_);
}
void KeepoutPanel::changePoints(const QJsonArray& p){auto zone=zones_[current_].toObject();zone["points"]=p;zones_[current_]=zone;showPoints();changed();}
void KeepoutPanel::changed(){dirty_=true;status_->setText(QStringLiteral("编辑内容尚未应用，导航仍使用上次已应用的禁入区。"));}
void KeepoutPanel::run(const QString& command,const QJsonObject& payload)
{
  if(!online_||busy_){status_->setText(QStringLiteral("请等待地图模块连接或当前操作完成"));return;}busy_=true;
  request(this,command,payload,[this,command](const Reply& r){busy_=false;status_->setText(r.message);if(r.success&&(command=="load"||command=="apply"||command=="airport_import")){loadConfiguration(r.data.toObject());dirty_=command!="apply";}Q_EMIT operationEvent(r.message,!r.success);});
}
void KeepoutPanel::setOnline(bool online)
{
  if(online==online_)return;online_=online;
  if(!online){state_sub_.shutdown();fix_sub_.shutdown();node_.reset();loaded_=false;return;}
  node_.reset(new ros::NodeHandle());
  state_sub_=node_->subscribe<std_msgs::String>("/gps/keepout/state",1,[this](const std_msgs::String::ConstPtr& msg){auto s=QJsonDocument::fromJson(QByteArray::fromStdString(msg->data)).object();QMetaObject::invokeMethod(this,[this,s](){if(!loaded_&&!dirty_)loadConfiguration(s["config"].toObject());if(!dirty_)status_->setText(s["message"].toString());},Qt::QueuedConnection);});
  fix_sub_=node_->subscribe<sensor_msgs::NavSatFix>("/gps/fix",1,[this](const sensor_msgs::NavSatFix::ConstPtr& msg){if(msg->status.status<0)return;double lat=msg->latitude,lon=msg->longitude;QMetaObject::invokeMethod(this,[this,lat,lon](){have_fix_=true;fix_lat_=lat;fix_lon_=lon;},Qt::QueuedConnection);});
}

FodTargetsPanel::FodTargetsPanel(QWidget* parent):QWidget(parent)
{
  setObjectName("fodTargetsPanel");auto* root=new QVBoxLayout(this);
  auto* note=new QLabel(QStringLiteral("FOD 位置点：每行“点名 经度 纬度”，支持空格、Tab、逗号，以及测量 CSV 的点名 / L / B 字段。导入只显示目标；选择后点击“发送所选 FOD 导航目标”才导航。"),this);note->setWordWrap(true);root->addWidget(note);
  input_=new QPlainTextEdit(this);input_->setObjectName("fodTargetInput");input_->setPlaceholderText("pt0\t104.3892865\t30.29387435");input_->setMaximumHeight(110);root->addWidget(input_);
  auto* imports=new QHBoxLayout();root->addLayout(imports);
  auto* parse=button(QStringLiteral("导入文本并显示"),"fodTargetsParse",imports,this);
  auto* file=button(QStringLiteral("导入 CSV / TXT"),"fodTargetsFile",imports,this);
  auto* remove=button(QStringLiteral("删除所选 FOD 点"),"fodTargetsDelete",imports,this);
  auto* nearest=button(QStringLiteral("选择最近 FOD"),"fodTargetsNearest",imports,this);
  auto* middle=new QHBoxLayout();root->addLayout(middle,1);
  table_=new QTableWidget(this);table_->setObjectName("fodTargetsTable");tableSetup(table_,{QStringLiteral("点名"),QStringLiteral("经度"),QStringLiteral("纬度"),QStringLiteral("直线距离 / 米")});middle->addWidget(table_,1);
  preview_=new MapPreview(this);preview_->setScene(new QGraphicsScene(preview_));preview_->setMinimumSize(350,230);middle->addWidget(preview_,1);
  auto* actions=new QHBoxLayout();root->addLayout(actions);
  auto* map=button(QStringLiteral("查看导航地图"),"fodTargetsMap",actions,this);
  auto* send=button(QStringLiteral("发送所选 FOD 导航目标"),"fodTargetsSend",actions,this);send->setStyleSheet(QStringLiteral("font-weight:bold;background:#19846b;color:white;"));
  status_=new QLabel(QStringLiteral("等待 GPS 地图模块"),this);status_->setObjectName("fodTargetsStatus");status_->setWordWrap(true);root->addWidget(status_);
  auto* map_timer=new QTimer(this);map_timer->setInterval(1000);
  connect(map_timer,&QTimer::timeout,this,[this](){if(isVisible()&&have_fix_)preview(preview_,{},targets_,true,fix_lat_,fix_lon_,airport_);});map_timer->start();
  connect(parse,&QPushButton::clicked,this,[this](){run("targets_parse",QJsonObject{{"text",input_->toPlainText()}},true);});
  connect(file,&QPushButton::clicked,this,[this](){auto p=QFileDialog::getOpenFileName(this,QStringLiteral("导入 FOD 目标点"),QStandardPaths::writableLocation(QStandardPaths::DesktopLocation),"Points (*.csv *.txt);;All (*)");if(!p.isEmpty())run("targets_parse",QJsonObject{{"path",p}},true);});
  connect(remove,&QPushButton::clicked,this,[this](){int row=table_->currentRow();if(row<0)return;auto a=targets_;a.removeAt(row);run("targets_set",a);});
  connect(nearest,&QPushButton::clicked,this,[this](){if(!have_fix_||targets_.isEmpty())return;int best=0;double distance=1e100;for(int i=0;i<targets_.size();++i){auto p=targets_[i].toObject();auto xy=local(p["latitude"].toDouble(),p["longitude"].toDouble(),fix_lat_,fix_lon_);double d=std::hypot(xy.x(),xy.y());if(d<distance){distance=d;best=i;}}table_->selectRow(best);});
  connect(map,&QPushButton::clicked,this,&FodTargetsPanel::showNavigationMap);
  connect(send,&QPushButton::clicked,this,[this](){int row=table_->currentRow();if(row<0||row>=targets_.size()){status_->setText(QStringLiteral("请先选择一个 FOD 点"));return;}auto p=targets_[row].toObject();Q_EMIT navigationRequested(p["name"].toString(),p["latitude"].toDouble(),p["longitude"].toDouble());});
  connect(table_,&QTableWidget::itemSelectionChanged,this,[this](){int row=table_->currentRow();if(row<0||row>=targets_.size()||!online_)return;std_msgs::String m;m.data=targets_[row].toObject()["name"].toString().toStdString();selection_pub_.publish(m);});
}
void FodTargetsPanel::loadPoints(const QJsonArray& points)
{
  targets_=points;QSignalBlocker block(table_);table_->setRowCount(points.size());
  for(int i=0;i<points.size();++i){auto p=points[i].toObject();item(table_,i,0,p["name"].toString());item(table_,i,1,QString::number(p["longitude"].toDouble(),'f',8));item(table_,i,2,QString::number(p["latitude"].toDouble(),'f',8));}
  loaded_=true;refreshDistances();preview(preview_,{},targets_,have_fix_,fix_lat_,fix_lon_,airport_);
}
void FodTargetsPanel::refreshDistances()
{
  for(int i=0;i<targets_.size();++i){auto p=targets_[i].toObject();auto xy=local(p["latitude"].toDouble(),p["longitude"].toDouble(),fix_lat_,fix_lon_);item(table_,i,3,have_fix_?QString::number(std::hypot(xy.x(),xy.y()),'f',1):QStringLiteral("等待 GPS"));}
}
void FodTargetsPanel::run(const QString& command,const QJsonValue& payload,bool apply_after)
{
  if(!online_||busy_){status_->setText(QStringLiteral("请等待地图模块连接或当前操作完成"));return;}busy_=true;
  request(this,command,payload,[this,apply_after](const Reply& r){busy_=false;status_->setText(r.message);Q_EMIT operationEvent(r.message,!r.success);if(r.success){if(apply_after)run("targets_set",r.data);else loadPoints(r.data.toArray());}});
}
void FodTargetsPanel::setOnline(bool online)
{
  if(online==online_)return;online_=online;
  if(!online){state_sub_.shutdown();fix_sub_.shutdown();keepout_sub_.shutdown();selection_pub_.shutdown();node_.reset();loaded_=false;return;}
  node_.reset(new ros::NodeHandle());selection_pub_=node_->advertise<std_msgs::String>("/gps/targets/selection",1,true);
  state_sub_=node_->subscribe<std_msgs::String>("/gps/targets/state",1,[this](const std_msgs::String::ConstPtr& msg){auto s=QJsonDocument::fromJson(QByteArray::fromStdString(msg->data)).object();QMetaObject::invokeMethod(this,[this,s](){if(!loaded_||targets_!=s["points"].toArray())loadPoints(s["points"].toArray());status_->setText(QStringLiteral("地图中已保存 %1 个 FOD 点；选择目标后发送。").arg(targets_.size()));},Qt::QueuedConnection);});
  keepout_sub_=node_->subscribe<std_msgs::String>("/gps/keepout/state",1,[this](const std_msgs::String::ConstPtr& msg){auto s=QJsonDocument::fromJson(QByteArray::fromStdString(msg->data)).object();QMetaObject::invokeMethod(this,[this,s](){airport_=s["config"].toObject()["airport"].toObject();preview(preview_,{},targets_,have_fix_,fix_lat_,fix_lon_,airport_);},Qt::QueuedConnection);});
  fix_sub_=node_->subscribe<sensor_msgs::NavSatFix>("/gps/fix",1,[this](const sensor_msgs::NavSatFix::ConstPtr& msg){if(msg->status.status<0)return;double lat=msg->latitude,lon=msg->longitude;QMetaObject::invokeMethod(this,[this,lat,lon](){bool first=!have_fix_;have_fix_=true;fix_lat_=lat;fix_lon_=lon;refreshDistances();if(first)preview(preview_,{},targets_,true,lat,lon,airport_);},Qt::QueuedConnection);});
}
}
