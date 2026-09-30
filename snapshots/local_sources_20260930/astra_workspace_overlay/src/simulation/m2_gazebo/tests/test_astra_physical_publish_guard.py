#!/usr/bin/env python3
"""Run the actual source-extracted End guard with fake physical joint states."""
from pathlib import Path
import subprocess
import tempfile

source = Path(__file__).resolve().parents[1] / 'src/astra_physical_ackermann_plugin.cpp'
text = source.read_text()
finite = text[text.index('  bool finiteJointState() const {'):text.index('  ros::Time stamp(')]
prefix = text[text.index('  void publish() {'):text.index('    const auto pose=base_->WorldPose();const auto relative=')]
assert '!finiteJointState()' in prefix
program = r'''
#include <array>
#include <cassert>
#include <cmath>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
namespace std_msgs {struct Bool {bool data;};}
std::ostringstream gzerr;
struct Joint {double position=0,rate=0;double Position(int) const{return position;}double GetVelocity(int)const{return rate;}};
struct Stamp{double Double() const{return 1.;}};
struct World{Stamp SimTime()const{return {};}};
struct Publisher{int count=0;bool last=false;void publish(std_msgs::Bool x){++count;last=x.data;}};
struct Fixture{
  World world_value;World* world_=&world_value;
  std::array<std::shared_ptr<Joint>,6> joints_;
  bool fault_=false,fault_logged_=false,base_finite=true;
  double last_publish_=0,publish_hz_=50;
  std::mutex mutex_;std::deque<int> pending_{1,2};Publisher fault_pub_;int valid_publications=0;
  Fixture(){for(auto& j:joints_)j=std::make_shared<Joint>();}
  bool finiteBaseState()const{return base_finite;}
'''+finite+prefix+r'''
  ++valid_publications;
  }
};
int main(){
  {Fixture f;f.publish();assert(f.valid_publications==1&&!f.fault_&&f.fault_pub_.count==0);}
  int rejected=0;
  for(int i=0;i<6;++i)for(int component=0;component<2;++component)for(double x:{NAN,INFINITY,-INFINITY}){
    Fixture f;if(component==0)f.joints_[i]->position=x;else f.joints_[i]->rate=x;
    f.publish();assert(f.fault_&&f.pending_.empty()&&f.valid_publications==0&&f.fault_pub_.count==1&&f.fault_pub_.last);++rejected;
  }
  {Fixture f;f.base_finite=false;f.publish();assert(f.fault_&&f.pending_.empty()&&f.valid_publications==0&&f.fault_pub_.last);}
  std::cout<<"PASS actual-source End guard: "<<rejected<<" joint nonfinite cases + base invalid + finite baseline; no invalid data published.\n";
}
'''
with tempfile.TemporaryDirectory(prefix='m12_publish_guard_') as tmp:
    cpp=Path(tmp)/'guard.cpp';binary=Path(tmp)/'guard';cpp.write_text(program)
    subprocess.run(['g++','-std=c++14','-Wall','-Wextra',str(cpp),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
