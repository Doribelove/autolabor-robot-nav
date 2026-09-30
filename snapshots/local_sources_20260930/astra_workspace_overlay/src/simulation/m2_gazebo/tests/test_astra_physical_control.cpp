#include <cassert>
#include <cmath>
#include <iostream>
#include <m2_gazebo/astra_physical_control.h>
#include <m2_gazebo/actuator_dynamics.h>
using namespace m2_gazebo;
int main() {
  int geometries=0;
  for (double vx:{-1.2,-.4,.0,.4,1.2}) for (double center:{-.4964,-.3,0.,.3,.4964}) {
    const auto g=steeringGeometry(center,vx,.65,.60,.4964);
    const auto w=physicalWheelSpeeds(vx,g,.65,.60,.15);
    assert(std::abs(actualCenterSteering(g.left,g.right,.65,.60)-center)<1e-10);
    for (size_t i=0;i<4;++i) {
      const double x=i<2?.325:-.325,y=i%2==0?.30:-.30;
      const double phi=i==0?g.left:i==1?g.right:0.;
      const double ux=vx-g.yaw_rate*y,uy=.325*g.yaw_rate+g.yaw_rate*x;
      // Ground-contact velocity perpendicular to each no-slip wheel is zero.
      assert(std::abs(-ux*std::sin(phi)+uy*std::cos(phi))<1e-10);
      assert(std::abs(w[i]*.15-(ux*std::cos(phi)+uy*std::sin(phi)))<1e-10);
    }
    ++geometries;
  }
  assert(twistToSteering(0.,1.,.65,.4964)==0.); // No differential-drive spin.
  BoundedPI pi;pi.kp=4.;pi.ki=2.;pi.integral_limit=1.5;
  for (int i=0;i<10000;++i) {const double u=pi.update(100.,0.,0.,.002,8.);assert(u==8.);assert(pi.integral==0.);}
  assert(pi.update(0.,0.,0.,.002,8.)==0.);
  assert(pi.update(NAN,0.,0.,.002,8.)==0.);
  // Counterexample to tracking residual target during a stop: the wheel is
  // blocked at zero with a positive speed target/integral. Stop torque must
  // still be zero, and all moving-wheel cases must be dissipative.
  pi.integral=1.5;
  assert(pi.update(2.,0.,0.,.002,12.)>0.);
  for (double rate:{-20.,-1.,-.01,0.,.01,1.,20.}) {
    const double torque=passiveWheelBrake(rate,4.,12.);
    assert(std::abs(torque)<=12.);assert(torque*rate<=0.);
    if (rate==0.) assert(torque==0.);
  }
  assert(passiveWheelBrake(NAN,4.,12.)==0.);
  for (double error:{-.7,-.1,.0,.1,.7}) for (double rate:{-5.,0.,5.})
    assert(std::abs(boundedSteeringTorque(error,0.,0.,rate,60.,4.,20.))<=20.);
  ActuatorLimits lim;lim.max_acceleration_mps2=.6;lim.max_deceleration_mps2=.9;
  lim.max_brake_deceleration_mps2=.9;lim.max_emergency_deceleration_mps2=.9;lim.max_steering_rate_radps=.75;
  ActuatorState s;
  for (int i=0;i<6000;++i) {
    const auto old=s;const double target=i<2500?1.2:i<3500?0.:-.4;
    s=advanceActuators(s,target,i%2000<1000?.4964:-.4964,false,false,.002,lim);
    assert(std::abs(s.steering_rad-old.steering_rad)/.002<=.75+1e-10);
    assert(std::abs(s.speed_mps-old.speed_mps)/.002<=.9+1e-10);
    if (old.speed_mps*s.speed_mps>=0.&&std::abs(s.speed_mps)>std::abs(old.speed_mps))
      assert(std::abs(s.speed_mps-old.speed_mps)/.002<=.6+1e-10);
  }
  std::cout<<"PASS "<<geometries<<" forward/reverse/zero Ackermann geometries with midpoint vy and wheel-plane no-slip, finite torque, anti-windup, NaN rejection, and6000 finite target dynamics steps. Actual physical tracking remains untested.\n";
}
