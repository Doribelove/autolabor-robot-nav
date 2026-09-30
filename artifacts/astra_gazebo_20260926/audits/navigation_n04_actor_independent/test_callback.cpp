#include <m2_gazebo/astra_event_actor_gate_n04.h>
#include <memory>
#include <mutex>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>
#include <iostream>
namespace ros {static std::atomic<int64_t> clock_ns{0};struct Time {static Time now(){return Time();}uint64_t toNSec()const{return clock_ns.load();}};}
namespace std_msgs {struct String {using ConstPtr=std::shared_ptr<const String>;std::string data;};}
struct FakeModel{std::string GetName()const{return "synthetic_actor";}};
struct FakePublisher{std::vector<m2_gazebo::N04Json> rows;void publish(const std_msgs::String&m){rows.push_back(m2_gazebo::N04Json::parse(m.data));}};
class ActualBodies {public:
 m2_gazebo::N04Identity identity_{"fixture_n04",260926123,"dwa_common_route"};
 std::unique_ptr<m2_gazebo::N04EventGate> gate_{new m2_gazebo::N04EventGate(0,m2_gazebo::kN04TransportLimitNs)};
 std::mutex mutex_;m2_gazebo::N04Json event_;std::string pending_fault_;bool consumed_{false};uint64_t duplicates_{0},published_duplicates_{0};
 int64_t receipt_ros_ns_{0},receipt_steady_ns_{0},last_receipt_ros_ns_{0},last_status_ns_{0},physics_ns{7000000000LL};
 std::shared_ptr<FakeModel> model_{new FakeModel};FakePublisher status_pub_;
 int64_t nowNs()const{return physics_ns;}
#include "actual_callback_bodies.inc"
};
using namespace m2_gazebo;
int main(){int checks=0;auto ok=[&](bool x){if(!x)throw std::runtime_error("check"+std::to_string(checks));++checks;};
N04Json event={{"schema","astra_common_navigation_v1"},{"run_id","fixture_n04"},{"seed",260926123},{"method","dwa_common_route"},{"source_ns",7000000000LL},{"source","actual_odom_header_first_crossing"},{"pose_map",{-4.,0.,0.}},{"body_twist",{.8,0.,0.}},{"activation_source_ns",5000000000LL},{"goal_pose",{5.,0.,0.}},{"deadline_source_ns",29000000000LL},{"deadline_ns",29000000000LL}};
auto msg=[&](N04Json x){auto m=std::make_shared<std_msgs::String>();m->data=x.dump();return m;};
{ActualBodies x;ros::clock_ns=1;x.receive(msg(event));ok(x.gate_->source()==0&&x.status_pub_.rows.empty());x.physics_ns=7040000000LL;x.consumeEvent();ok(x.gate_->firstBegin()==7040000000LL&&x.receipt_ros_ns_==1);auto g=x.gate_->update(x.physics_ns,0,0);ok(g.active&&g.active_dt_s==0.);x.publishStatus();ok(x.status_pub_.rows[0]["status_source_ns"]==x.status_pub_.rows[1]["status_source_ns"]);ok(x.status_pub_.rows[0]["first_active_begin_ns"]==0&&x.status_pub_.rows[1]["first_active_begin_ns"]==x.physics_ns);}
{ActualBodies x;ros::clock_ns=7000000000LL;auto m=msg(event);std::vector<std::thread> threads;for(int i=0;i<8;++i)threads.emplace_back([&](){for(int n=0;n<100;++n)x.receive(m);});for(auto&t:threads)t.join();ok(x.duplicates_==799);x.physics_ns=7002000000LL;x.consumeEvent();ok(x.gate_->valid()&&x.gate_->source()==7000000000LL);ok(x.status_pub_.rows.back()["duplicate_count"]==799);}
{ActualBodies x;ros::clock_ns=7000000000LL;x.receive(msg(event));auto bad=event;bad["pose_map"][0]=-3.99;x.receive(msg(bad));x.physics_ns=7002000000LL;x.consumeEvent();ok(!x.gate_->valid()&&x.gate_->reason()=="conflicting_duplicate_gate");x.receive(msg(event));x.consumeEvent();ok(!x.gate_->valid());}
{ActualBodies x;ros::clock_ns=7000000000LL;x.receive(msg(event));ros::clock_ns=6999999999LL;x.receive(msg(event));x.physics_ns=7002000000LL;x.consumeEvent();ok(!x.gate_->valid()&&x.gate_->reason()=="receipt_clock_regression");}
{ActualBodies x;auto wrong=event;wrong["seed"]=260926124;ros::clock_ns=7000000000LL;x.receive(msg(wrong));x.receive(msg(event));x.physics_ns=7002000000LL;x.consumeEvent();ok(!x.gate_->valid()&&x.gate_->source()==0);}
{ActualBodies x;ros::clock_ns=0;x.receive(msg(event));x.physics_ns=7050000001LL;x.consumeEvent();ok(!x.gate_->valid()&&x.gate_->source()==0);ok(x.status_pub_.rows.back()["fault"]==true);}
std::cout<<"checks="<<checks<<" passed; actual three method bodies with synthetic clocks/transport\n";}
