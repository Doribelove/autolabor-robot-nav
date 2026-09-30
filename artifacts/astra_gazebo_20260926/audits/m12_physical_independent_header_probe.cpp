#include <m2_gazebo/astra_physical_control.h>
#include <m2_gazebo/actuator_dynamics.h>
#include <cassert>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <random>
#include <limits>
using namespace m2_gazebo;
int main() {
  std::mt19937 rng(260926);
  std::uniform_real_distribution<double> speed(-2.778,2.778),angle(-.4964,.4964),signed_unit(-1.,1.);
  double maximum_lateral_residual=0.,maximum_rolling_residual=0.,maximum_inverse_error=0.;
  double maximum_wheel_rate=0.,maximum_steering_angle=0.;
  for (int sample=0;sample<20000;++sample) {
    const double v=speed(rng),center=angle(rng),omega=v*std::tan(center)/.65;
    auto g=steeringGeometry(center,v,.65,.60,.4964);
    auto rates=physicalWheelSpeeds(v,g,.65,.60,.15);
    maximum_inverse_error=std::max(maximum_inverse_error,std::abs(actualCenterSteering(g.left,g.right,.65,.60)-center));
    for (int i=0;i<4;++i) {
      // Independent rigid-body velocity about the actual axle-midpoint root.
      const double px=i<2?.325:-.325,py=i%2==0?.30:-.30;
      const double phi=i==0?g.left:i==1?g.right:0.;
      const double vx=v-omega*py,vy=omega*.325+omega*px;
      maximum_lateral_residual=std::max(maximum_lateral_residual,std::abs(-vx*std::sin(phi)+vy*std::cos(phi)));
      maximum_rolling_residual=std::max(maximum_rolling_residual,std::abs(rates[i]*.15-(vx*std::cos(phi)+vy*std::sin(phi))));
      maximum_wheel_rate=std::max(maximum_wheel_rate,std::abs(rates[i]));
      maximum_steering_angle=std::max(maximum_steering_angle,std::abs(phi));
    }
  }
  assert(maximum_lateral_residual<1e-12 && maximum_rolling_residual<1e-12 && maximum_inverse_error<1e-12);
  BoundedPI p;
  for (int n=0;n<100000;++n) {
    const double limit=n%2?8.:12.;
    const double u=p.update(signed_unit(rng)*40.,signed_unit(rng)*40.,signed_unit(rng)*50.,.002,limit);
    assert(std::isfinite(u)&&std::abs(u)<=limit&&std::abs(p.integral)<=1.5);
  }
  for (double direction:{-1.,1.}) {
    BoundedPI pi;
    for (int n=0;n<10000;++n) {assert(pi.update(direction*100.,0.,0.,.002,8.)==direction*8.);assert(pi.integral==0.);}
    pi.integral=direction*1.;pi.update(-direction*.1,0.,0.,.002,8.);
    assert(std::abs(pi.integral)<1.);
    const auto held=pi.integral;
    assert(pi.update(NAN,0.,0.,.002,8.)==0. && pi.integral==held);
    assert(pi.update(0.,INFINITY,0.,.002,8.)==0. && pi.integral==held);
    assert(pi.update(0.,0.,NAN,.002,8.)==0. && pi.integral==held);
    assert(pi.update(0.,0.,0.,0.,8.)==0. && pi.integral==held);
  }
  // Replay the original plugin's stopping branch without any simulator.
  // This deliberately exposes a real defect: a lagging wheel can be driven
  // while brake/timeout is active because a residual moving reference remains.
  ActuatorLimits limits;limits.max_acceleration_mps2=.6;limits.max_deceleration_mps2=.9;
  limits.max_brake_deceleration_mps2=.9;limits.max_emergency_deceleration_mps2=.9;
  ActuatorState old;old.speed_mps=1.2;
  auto next=advanceActuators(old,0.,0.,true,false,.002,limits);
  const double old_rate=old.speed_mps/.15,new_rate=next.speed_mps/.15;
  const double ff=(79.6*.15*.15/4.+3.*.15*.15/2.)*(new_rate-old_rate)/.002+.05*new_rate+.01;
  BoundedPI stopped;
  const double stationary_torque=stopped.update(new_rate,0.,ff,.002,12.);
  BoundedPI lagging;
  const double lagging_torque=lagging.update(new_rate,.1,ff,.002,12.);
  assert(stationary_torque>0. && lagging_torque*.1>0.);
  std::cout<<std::setprecision(17)<<"{\"geometry_samples\":20000,\"pi_samples\":100000,\"maximum_lateral_residual_mps\":"<<maximum_lateral_residual
    <<",\"maximum_rolling_residual_mps\":"<<maximum_rolling_residual<<",\"maximum_inverse_error_rad\":"<<maximum_inverse_error
    <<",\"maximum_sampled_wheel_rate_radps\":"<<maximum_wheel_rate<<",\"maximum_sampled_steering_angle_rad\":"<<maximum_steering_angle
    <<",\"original_stopping_stationary_wheel_drive_torque_nm\":"<<stationary_torque<<",\"original_stopping_lagging_wheel_power_w\":"<<lagging_torque*.1
    <<",\"original_stopping_branch_defect_reproduced\":true}"<<std::endl;
}
