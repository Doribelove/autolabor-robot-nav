#include <autolabor_operator_gui/main_window.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFutureWatcher>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QTimer>
#include <QTabWidget>
#include <QSignalBlocker>
#include <QWheelEvent>
#include <QVBoxLayout>
#include <QtConcurrent/QtConcurrentRun>
#include <cmath>

namespace autolabor_operator_gui {
namespace {
class TuningSpinBox : public QDoubleSpinBox {
public:
  using QDoubleSpinBox::QDoubleSpinBox;
protected:
  void wheelEvent(QWheelEvent* event) override { event->ignore(); }
};
struct TuningField { const char* key; const char* label; double minimum, maximum; bool integer; };
struct TuningReply { bool success = false; dynamic_reconfigure::Config config; QString message; };
QGroupBox* tuningGroup(QWidget* parent, const QString& title, const std::string& endpoint,
                      const std::vector<TuningField>& fields) {
  auto* group = new QGroupBox(title, parent);
  auto* layout = new QVBoxLayout(group);
  auto* form = new QFormLayout();
  std::vector<QDoubleSpinBox*> inputs;
  for (const auto& field : fields) {
    auto* input = new TuningSpinBox(group);
    input->setRange(field.minimum, field.maximum);
    input->setDecimals(field.integer ? 0 : 3);
    input->setSingleStep(field.integer ? 1 : .05);
    input->setKeyboardTracking(false);
    input->setEnabled(false);
    input->setToolTip(QString::fromUtf8(field.key));
    form->addRow(QString::fromUtf8(field.label), input);
    inputs.push_back(input);
  }
  layout->addLayout(form);
  auto* read = new QPushButton(QStringLiteral("读取后端当前值"), group);
  auto* status = new QLabel(QStringLiteral("待读取；读取成功后修改自动应用（400 ms 防抖）"), group);
  status->setWordWrap(true);
  layout->addWidget(read); layout->addWidget(status);
  auto* timer = new QTimer(group); timer->setSingleShot(true); timer->setInterval(400);
  const auto invoke = [=](bool write) {
    if (!read->isEnabled()) return;
    dynamic_reconfigure::Config request;
    if (write) for (size_t i=0; i<fields.size(); ++i) {
      if (fields[i].integer) {
        dynamic_reconfigure::IntParameter p; p.name=fields[i].key; p.value=static_cast<int>(inputs[i]->value()); request.ints.push_back(p);
      } else {
        dynamic_reconfigure::DoubleParameter p; p.name=fields[i].key; p.value=inputs[i]->value(); request.doubles.push_back(p);
      }
    }
    read->setEnabled(false);
    for (auto* input : inputs) input->setEnabled(false);
    status->setText(QStringLiteral("等待后端应用并回读……"));
    auto* watcher = new QFutureWatcher<TuningReply>(group);
    QObject::connect(watcher, &QFutureWatcher<TuningReply>::finished, group, [=]() {
      const auto result=watcher->result();
      bool complete=result.success;
      for (size_t i=0; i<fields.size(); ++i) {
        bool found=false; double value=0;
        if (fields[i].integer) { for (const auto& p:result.config.ints) if(p.name==fields[i].key) {found=true;value=p.value;} }
        else { for (const auto& p:result.config.doubles) if(p.name==fields[i].key) {found=true;value=p.value;} }
        complete=complete && found;
        if(found) { const QSignalBlocker blocker(inputs[i]); inputs[i]->setValue(value); }
      }
      for(auto* input:inputs) input->setEnabled(complete);
      read->setEnabled(true);
      status->setText(complete ? result.message : QStringLiteral("未确认应用，请重新读取：")+result.message);
      watcher->deleteLater();
    });
    watcher->setFuture(QtConcurrent::run([endpoint,request,write]() {
      TuningReply result;
      ros::NodeHandle node;
      int persistence_protocol = 0;
      if (!node.getParam(endpoint+"/tuning_persistence_protocol", persistence_protocol) || persistence_protocol != 1) {
        result.message=QStringLiteral("后端未支持永久保存，请启动配套新版后端"); return result;
      }
      auto client=node.serviceClient<dynamic_reconfigure::Reconfigure>(endpoint+"/set_parameters",false);
      if(!client.waitForExistence(ros::Duration(2.0))) {result.message=QStringLiteral("参数服务未就绪");return result;}
      dynamic_reconfigure::Reconfigure call; call.request.config=request;
      if(!client.call(call)) {result.message=QStringLiteral("参数服务调用失败");return result;}
      result.config=call.response.config; result.success=true;
      bool matched=true;
      for(const auto& p:request.doubles) {bool found=false;for(const auto& q:result.config.doubles)if(p.name==q.name){found=true;matched=matched && std::isfinite(q.value) && std::abs(p.value-q.value)<1e-6;}matched=matched&&found;}
      for(const auto& p:request.ints) {bool found=false;for(const auto& q:result.config.ints)if(p.name==q.name){found=true;matched=matched&&p.value==q.value;}matched=matched&&found;}
      result.message = !write ? QStringLiteral("已读取运行值；修改后自动应用") : matched ? QStringLiteral("后端已应用并永久保存，重启后保留") : QStringLiteral("后端拒绝或修正了请求，已回显实际值；请取消覆盖预览或等待任务结束后重试");
      return result;
    }));
  };
  QObject::connect(read,&QPushButton::clicked,group,[=](){timer->stop();invoke(false);});
  QObject::connect(timer,&QTimer::timeout,group,[=](){invoke(true);});
  for(auto* input:inputs) QObject::connect(input,QOverload<double>::of(&QDoubleSpinBox::valueChanged),timer,[=](double){timer->start();});
  QTimer::singleShot(1500, group, [=](){invoke(false);});
  return group;
}
}
QWidget* MainWindow::buildNavigationTuningPage() {
  auto* scroll=new QScrollArea(); scroll->setWidgetResizable(true);
  auto* content=new QWidget(scroll); auto* layout=new QVBoxLayout(content);
  content->setObjectName(QStringLiteral("navigationTuningContent"));
  content->setStyleSheet(QStringLiteral("QWidget#navigationTuningContent {background:#101721;} QScrollArea {background:#101721;border:0;}"));
  auto* note=new QLabel(QStringLiteral("覆盖规划、转场 TEB 与 Hybrid A* 参数。覆盖参数沿用事务持久化；高级参数由后端原子保存，重启优先加载已保存值。转场参数在取消预览或任务结束后可修改，下一次转场立即采用；Hybrid 参数在当前搜索结束后切换，下一次搜索采用。"),content);
  note->setWordWrap(true); note->setStyleSheet(QStringLiteral("color:#b9c8dc;font-size:11pt;")); layout->addWidget(note);
  auto* sections = new QTabWidget(content);
  const auto add_section = [=](const QString& title, QWidget* group) {
    auto* area = new QScrollArea(sections); area->setWidgetResizable(true);
    auto* page = new QWidget(area); page->setStyleSheet(QStringLiteral("background:#101721;"));
    auto* rows = new QVBoxLayout(page); rows->addWidget(group); rows->addStretch();
    area->setWidget(page); sections->addTab(area,title);
  };
  if(coverage_parameters_widget_) add_section(QStringLiteral("覆盖规划"), coverage_parameters_widget_);
  add_section(QStringLiteral("转场 TEB"), tuningGroup(content, QStringLiteral("转场 TEB"), "/coverage_manager", {
    {"hybrid_transit_max_forward_speed_mps", "转场前进速度上限 (m/s)，0 跟随覆盖参数", 0.0, 1.6, false},
    {"hybrid_transit_max_reverse_speed_mps", "转场倒车速度上限 (m/s)，0 跟随覆盖参数", 0.0, 0.8, false},
    {"hybrid_transit_max_angular_speed_rps", "转场角速度上限 (rad/s)，0 跟随覆盖参数", 0.0, 1.0, false},
    {"hybrid_transit_linear_accel_mps2", "转场线加速度 (m/s²)，0 跟随覆盖参数", 0.0, 2.0, false},
    {"hybrid_transit_angular_accel_rps2", "转场角加速度 (rad/s²)，0 跟随覆盖参数", 0.0, 1.0, false},
    {"hybrid_transit_viapoint_separation_m", "路径点间距 (m)", 0.05, 5.0, false},
    {"hybrid_transit_lookahead_dist_m", "转场前视距离 (m)", 1.0, 8.0, false},
    {"hybrid_transit_min_obstacle_dist_m", "障碍距离 (m)", 0.15, 0.3, false},
    {"hybrid_transit_inflation_dist_m", "膨胀距离 (m)", 0.3, 0.8, false},
    {"hybrid_transit_weight_viapoint", "路径点权重", 0.0, 1000.0, false},
    {"hybrid_transit_weight_viapoint_lateral", "横向误差权重", 0.0, 1000.0, false},
    {"hybrid_transit_weight_viapoint_heading", "航向误差权重", 0.0, 1000.0, false},
    {"hybrid_transit_weight_kinematics_forward_drive", "前进运动学权重", 0.0, 1000.0, false},
    {"hybrid_transit_selection_viapoint_cost_scale", "候选路径点代价倍率", 0.0, 100.0, false},
    {"hybrid_transit_inner_iterations", "TEB 内迭代次数", 1, 20, true},
    {"hybrid_transit_outer_iterations", "TEB 外迭代次数", 1, 10, true},
  }));
  add_section(QStringLiteral("Hybrid A*"), tuningGroup(content, QStringLiteral("混合 A*"), "/move_base/CoverageGlobalPlanner", {
    {"hybrid_motion_step", "Hybrid 运动步长 (m)", 0.05, 1.0, false},
    {"hybrid_collision_check_step", "碰撞采样间距 (m)", 0.02, 0.1, false},
    {"hybrid_state_resolution", "状态栅格分辨率 (m)", 0.05, 0.5, false},
    {"hybrid_heading_bins", "航向离散数量", 24, 180, true},
    {"hybrid_steering_samples", "转向采样数量（奇数）", 3, 11, true},
    {"hybrid_max_expansions", "最大扩展节点数", 1000, 200000, true},
    {"hybrid_planning_timeout", "规划时间预算 (s)", 0.1, 5.0, false},
    {"hybrid_heuristic_weight", "启发式权重", 1.0, 3.0, false},
    {"hybrid_steering_penalty", "转向惩罚", 0.0, 5.0, false},
    {"hybrid_steering_change_penalty", "转向变化惩罚", 0.0, 5.0, false},
    {"hybrid_obstacle_cost_scale", "障碍代价倍率", 0.0, 5.0, false},
  }));
  layout->addWidget(sections); sections->setCurrentIndex(2);
  scroll->setWidget(content); return scroll;
}
}
