#include <gazebo/ode/ode.h>
#include <dlfcn.h>
#include <cassert>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <random>
void vec(const dReal* p,int n) {std::cout<<"[";for(int i=0;i<n;i++){if(i)std::cout<<",";std::cout<<p[i];}std::cout<<"]";}
int main(){
 static_assert(sizeof(dReal)==8,"double ODE required");dInitODE2(0);std::cout<<std::setprecision(17);
 Dl_info info;dladdr(reinterpret_cast<void*>(dWorldQuickStep),&info);
 std::cout<<"{\"type\":\"library\",\"path\":\""<<info.dli_fname<<"\"}\n";
 std::mt19937 rng(260926);std::uniform_real_distribution<double> random(-3.,3.);
 for(int k=0;k<302;k++){
  auto w=dWorldCreate();dWorldSetGravity(w,0,0,0);dWorldSetERP(w,.2);dWorldSetCFM(w,1e-9);
  auto body=dBodyCreate(w);dBodySetAutoDisableFlag(body,0);dBodySetGyroscopicMode(body,0);
  dMass mass;dMassSetSphereTotal(&mass,1.,.5);dBodySetMass(body,&mass);
  const double damp=k==300?.2:0.;dBodySetDamping(body,damp,damp);
  dBodySetLinearDampingThreshold(body,0.);dBodySetAngularDampingThreshold(body,0.);
  dBodySetPosition(body,random(rng),random(rng),random(rng));
  dQuaternion q;dQFromAxisAndAngle(q,random(rng),random(rng),random(rng),random(rng));dBodySetQuaternion(body,q);
  dBodySetLinearVel(body,random(rng),random(rng),random(rng));dBodySetAngularVel(body,random(rng)*20,random(rng)*20,random(rng)*20);
  dBodyAddForce(body,random(rng)*20,random(rng)*20,random(rng)*20);
  dBodyAddTorque(body,random(rng),random(rng),random(rng));
  dJointGroupID group=nullptr;
  if(k==301){
   dBodySetPosition(body,0,0,.45);dBodySetLinearVel(body,0,0,0);dBodySetAngularVel(body,0,0,0);dBodySetForce(body,0,0,0);dBodySetTorque(body,0,0,0);
   group=dJointGroupCreate(0);dContact c{};c.surface.mode=dContactSoftERP|dContactSoftCFM;c.surface.mu=0.;c.surface.soft_erp=.2;c.surface.soft_cfm=1e-9;
   c.geom.pos[0]=0;c.geom.pos[1]=0;c.geom.pos[2]=0;c.geom.normal[2]=1;c.geom.depth=.05;
   auto j=dJointCreateContact(w,group,&c);dJointAttach(j,body,0);
  }
  std::cout<<"{\"type\":\"step\",\"case\":"<<k<<",\"dt\":0.002,\"damping\":"<<damp<<",\"finite_rotation_mode\":"<<dBodyGetFiniteRotationMode(body)<<",\"p0\":";vec(dBodyGetPosition(body),3);
  std::cout<<",\"q0_wxyz\":";vec(dBodyGetQuaternion(body),4);
  dWorldQuickStep(w,.002);
  std::cout<<",\"p1\":";vec(dBodyGetPosition(body),3);std::cout<<",\"q1_wxyz\":";vec(dBodyGetQuaternion(body),4);
  std::cout<<",\"v1\":";vec(dBodyGetLinearVel(body),3);std::cout<<",\"w1\":";vec(dBodyGetAngularVel(body),3);std::cout<<"}\n";
  if(group)dJointGroupDestroy(group);dWorldDestroy(w);
 }
 dCloseODE();
}
