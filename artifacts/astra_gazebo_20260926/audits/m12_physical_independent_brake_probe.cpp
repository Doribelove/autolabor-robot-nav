#include <m2_gazebo/astra_physical_control.h>
#include <cassert>
#include <cmath>
#include <random>
#include <iostream>
#include <iomanip>
int main() {
  std::mt19937 rng(260926007);
  std::uniform_real_distribution<double> rate(-1000.,1000.),gain(.001,100.),limit(.001,100.);
  double largest_positive_power=0.,maximum_limit_excess=0.;
  for (int i=0;i<200000;++i) {
    const double w=rate(rng),kp=gain(rng),cap=limit(rng);
    const double torque=m2_gazebo::passiveWheelBrake(w,kp,cap);
    assert(std::isfinite(torque));assert(std::abs(torque)<=cap);
    assert(torque*w<=0.);
    largest_positive_power=std::max(largest_positive_power,torque*w);
    maximum_limit_excess=std::max(maximum_limit_excess,std::abs(torque)-cap);
  }
  for (double cap:{.1,8.,12.,100.}) for (double kp:{.01,4.,100.}) {
    assert(m2_gazebo::passiveWheelBrake(0.,kp,cap)==0.);
    assert(m2_gazebo::passiveWheelBrake(1e-12,kp,cap)<0.);
    assert(m2_gazebo::passiveWheelBrake(-1e-12,kp,cap)>0.);
    assert(m2_gazebo::passiveWheelBrake(1e308,kp,cap)==-cap);
    assert(m2_gazebo::passiveWheelBrake(-1e308,kp,cap)==cap);
  }
  for (double invalid:{NAN,INFINITY,-INFINITY}) assert(m2_gazebo::passiveWheelBrake(invalid,4.,12.)==0.);
  for (double invalid:{0.,-1.,double(NAN),double(INFINITY)}) {
    assert(m2_gazebo::passiveWheelBrake(1.,invalid,12.)==0.);
    assert(m2_gazebo::passiveWheelBrake(1.,4.,invalid)==0.);
  }
  std::cout<<std::setprecision(17)<<"{\"passive_brake_random_samples\":200000,\"maximum_positive_commanded_rotational_power_w\":"<<largest_positive_power
    <<",\"maximum_torque_limit_excess_nm\":"<<maximum_limit_excess
    <<",\"zero_speed_brake_torque_nm\":"<<m2_gazebo::passiveWheelBrake(0.,4.,12.)
    <<",\"old_counterexample_lagging_rate_0_1_torque_nm\":"<<m2_gazebo::passiveWheelBrake(.1,4.,12.)
    <<",\"nonfinite_and_invalid_parameters_return_zero\":true,\"actual_physical_tracking_qualified\":false}"<<std::endl;
}
