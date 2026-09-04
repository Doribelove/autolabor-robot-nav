/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// The closed-form families below are adapted from OMPL's
// ReedsSheppStateSpace implementation by Mark Moll. They implement the 48
// paths described by Reeds and Shepp without adding an OMPL runtime
// dependency to the robot.

#include "autolabor_coverage/reeds_shepp.h"

#include <algorithm>
#include <cmath>

namespace autolabor_coverage
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kZero = 10.0 * std::numeric_limits<double>::epsilon();

using Segment = ReedsSheppSegmentType;
using Path = ReedsSheppPath;

const Segment kPathTypes[18][5] = {
    {Segment::LEFT, Segment::RIGHT, Segment::LEFT, Segment::NOP, Segment::NOP},
    {Segment::RIGHT, Segment::LEFT, Segment::RIGHT, Segment::NOP, Segment::NOP},
    {Segment::LEFT, Segment::RIGHT, Segment::LEFT, Segment::RIGHT, Segment::NOP},
    {Segment::RIGHT, Segment::LEFT, Segment::RIGHT, Segment::LEFT, Segment::NOP},
    {Segment::LEFT, Segment::RIGHT, Segment::STRAIGHT, Segment::LEFT, Segment::NOP},
    {Segment::RIGHT, Segment::LEFT, Segment::STRAIGHT, Segment::RIGHT, Segment::NOP},
    {Segment::LEFT, Segment::STRAIGHT, Segment::RIGHT, Segment::LEFT, Segment::NOP},
    {Segment::RIGHT, Segment::STRAIGHT, Segment::LEFT, Segment::RIGHT, Segment::NOP},
    {Segment::LEFT, Segment::RIGHT, Segment::STRAIGHT, Segment::RIGHT, Segment::NOP},
    {Segment::RIGHT, Segment::LEFT, Segment::STRAIGHT, Segment::LEFT, Segment::NOP},
    {Segment::RIGHT, Segment::STRAIGHT, Segment::RIGHT, Segment::LEFT, Segment::NOP},
    {Segment::LEFT, Segment::STRAIGHT, Segment::LEFT, Segment::RIGHT, Segment::NOP},
    {Segment::LEFT, Segment::STRAIGHT, Segment::RIGHT, Segment::NOP, Segment::NOP},
    {Segment::RIGHT, Segment::STRAIGHT, Segment::LEFT, Segment::NOP, Segment::NOP},
    {Segment::LEFT, Segment::STRAIGHT, Segment::LEFT, Segment::NOP, Segment::NOP},
    {Segment::RIGHT, Segment::STRAIGHT, Segment::RIGHT, Segment::NOP, Segment::NOP},
    {Segment::LEFT, Segment::RIGHT, Segment::STRAIGHT, Segment::LEFT, Segment::RIGHT},
    {Segment::RIGHT, Segment::LEFT, Segment::STRAIGHT, Segment::RIGHT, Segment::LEFT}};

double mod2pi(double value)
{
  double result = std::fmod(value, kTwoPi);
  if (result < -kPi)
    result += kTwoPi;
  else if (result > kPi)
    result -= kTwoPi;
  return result;
}

void polar(double x, double y, double& radius, double& angle)
{
  radius = std::hypot(x, y);
  angle = std::atan2(y, x);
}

void consider(std::vector<Path>& paths, int type, double t, double u, double v,
              double w = 0.0, double x = 0.0)
{
  Path candidate;
  const double length = std::abs(t) + std::abs(u) + std::abs(v) +
                        std::abs(w) + std::abs(x);
  std::copy(kPathTypes[type], kPathTypes[type] + 5,
            candidate.types.begin());
  candidate.lengths = {{t, u, v, w, x}};
  candidate.normalized_length = length;
  paths.push_back(std::move(candidate));
}

void tauOmega(double u, double v, double xi, double eta, double phi,
              double& tau, double& omega)
{
  const double delta = mod2pi(u - v);
  const double a = std::sin(u) - std::sin(delta);
  const double b = std::cos(u) - std::cos(delta) - 1.0;
  const double first = std::atan2(eta * a - xi * b, xi * a + eta * b);
  const double second = 2.0 * (std::cos(delta) - std::cos(v) -
                               std::cos(u)) + 3.0;
  tau = second < 0.0 ? mod2pi(first + kPi) : mod2pi(first);
  omega = mod2pi(tau - u + v - phi);
}

bool lpSpLp(double x, double y, double phi,
            double& t, double& u, double& v)
{
  polar(x - std::sin(phi), y - 1.0 + std::cos(phi), u, t);
  if (t < -kZero)
    return false;
  v = mod2pi(phi - t);
  return v >= -kZero;
}

bool lpSpRp(double x, double y, double phi,
            double& t, double& u, double& v)
{
  double first = 0.0;
  double squared = 0.0;
  polar(x + std::sin(phi), y - 1.0 - std::cos(phi), squared, first);
  squared *= squared;
  if (squared < 4.0)
    return false;
  u = std::sqrt(squared - 4.0);
  t = mod2pi(first + std::atan2(2.0, u));
  v = mod2pi(t - phi);
  return t >= -kZero && v >= -kZero;
}

void csc(double x, double y, double phi, std::vector<Path>& paths)
{
  double t = 0.0, u = 0.0, v = 0.0;
  if (lpSpLp(x, y, phi, t, u, v)) consider(paths, 14, t, u, v);
  if (lpSpLp(-x, y, -phi, t, u, v)) consider(paths, 14, -t, -u, -v);
  if (lpSpLp(x, -y, -phi, t, u, v)) consider(paths, 15, t, u, v);
  if (lpSpLp(-x, -y, phi, t, u, v)) consider(paths, 15, -t, -u, -v);
  if (lpSpRp(x, y, phi, t, u, v)) consider(paths, 12, t, u, v);
  if (lpSpRp(-x, y, -phi, t, u, v)) consider(paths, 12, -t, -u, -v);
  if (lpSpRp(x, -y, -phi, t, u, v)) consider(paths, 13, t, u, v);
  if (lpSpRp(-x, -y, phi, t, u, v)) consider(paths, 13, -t, -u, -v);
}

bool lpRmL(double x, double y, double phi,
           double& t, double& u, double& v)
{
  const double xi = x - std::sin(phi);
  const double eta = y - 1.0 + std::cos(phi);
  double radius = 0.0;
  double theta = 0.0;
  polar(xi, eta, radius, theta);
  if (radius > 4.0)
    return false;
  u = -2.0 * std::asin(0.25 * radius);
  t = mod2pi(theta + 0.5 * u + kPi);
  v = mod2pi(phi - t + u);
  return t >= -kZero && u <= kZero;
}

void ccc(double x, double y, double phi, std::vector<Path>& paths)
{
  double t = 0.0, u = 0.0, v = 0.0;
  if (lpRmL(x, y, phi, t, u, v)) consider(paths, 0, t, u, v);
  if (lpRmL(-x, y, -phi, t, u, v)) consider(paths, 0, -t, -u, -v);
  if (lpRmL(x, -y, -phi, t, u, v)) consider(paths, 1, t, u, v);
  if (lpRmL(-x, -y, phi, t, u, v)) consider(paths, 1, -t, -u, -v);

  const double backward_x = x * std::cos(phi) + y * std::sin(phi);
  const double backward_y = x * std::sin(phi) - y * std::cos(phi);
  if (lpRmL(backward_x, backward_y, phi, t, u, v)) consider(paths, 0, v, u, t);
  if (lpRmL(-backward_x, backward_y, -phi, t, u, v)) consider(paths, 0, -v, -u, -t);
  if (lpRmL(backward_x, -backward_y, -phi, t, u, v)) consider(paths, 1, v, u, t);
  if (lpRmL(-backward_x, -backward_y, phi, t, u, v)) consider(paths, 1, -v, -u, -t);
}

bool lpRupLumRm(double x, double y, double phi,
                double& t, double& u, double& v)
{
  const double xi = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  const double rho = 0.25 * (2.0 + std::hypot(xi, eta));
  if (rho > 1.0)
    return false;
  u = std::acos(rho);
  tauOmega(u, -u, xi, eta, phi, t, v);
  return t >= -kZero && v <= kZero;
}

bool lpRumLumRp(double x, double y, double phi,
                double& t, double& u, double& v)
{
  const double xi = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  const double rho = (20.0 - xi * xi - eta * eta) / 16.0;
  if (rho < 0.0 || rho > 1.0)
    return false;
  u = -std::acos(rho);
  if (u < -0.5 * kPi)
    return false;
  tauOmega(u, u, xi, eta, phi, t, v);
  return t >= -kZero && v >= -kZero;
}

void cccc(double x, double y, double phi, std::vector<Path>& paths)
{
  double t = 0.0, u = 0.0, v = 0.0;
  if (lpRupLumRm(x, y, phi, t, u, v)) consider(paths, 2, t, u, -u, v);
  if (lpRupLumRm(-x, y, -phi, t, u, v)) consider(paths, 2, -t, -u, u, -v);
  if (lpRupLumRm(x, -y, -phi, t, u, v)) consider(paths, 3, t, u, -u, v);
  if (lpRupLumRm(-x, -y, phi, t, u, v)) consider(paths, 3, -t, -u, u, -v);
  if (lpRumLumRp(x, y, phi, t, u, v)) consider(paths, 2, t, u, u, v);
  if (lpRumLumRp(-x, y, -phi, t, u, v)) consider(paths, 2, -t, -u, -u, -v);
  if (lpRumLumRp(x, -y, -phi, t, u, v)) consider(paths, 3, t, u, u, v);
  if (lpRumLumRp(-x, -y, phi, t, u, v)) consider(paths, 3, -t, -u, -u, -v);
}

bool lpRmSmLm(double x, double y, double phi,
              double& t, double& u, double& v)
{
  const double xi = x - std::sin(phi);
  const double eta = y - 1.0 + std::cos(phi);
  double rho = 0.0;
  double theta = 0.0;
  polar(xi, eta, rho, theta);
  if (rho < 2.0)
    return false;
  const double root = std::sqrt(rho * rho - 4.0);
  u = 2.0 - root;
  t = mod2pi(theta + std::atan2(root, -2.0));
  v = mod2pi(phi - 0.5 * kPi - t);
  return t >= -kZero && u <= kZero && v <= kZero;
}

bool lpRmSmRm(double x, double y, double phi,
              double& t, double& u, double& v)
{
  const double xi = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  double rho = 0.0;
  double theta = 0.0;
  polar(-eta, xi, rho, theta);
  if (rho < 2.0)
    return false;
  t = theta;
  u = 2.0 - rho;
  v = mod2pi(t + 0.5 * kPi - phi);
  return t >= -kZero && u <= kZero && v <= kZero;
}

void ccsc(double x, double y, double phi, std::vector<Path>& paths)
{
  double t = 0.0, u = 0.0, v = 0.0;
  if (lpRmSmLm(x, y, phi, t, u, v)) consider(paths, 4, t, -0.5 * kPi, u, v);
  if (lpRmSmLm(-x, y, -phi, t, u, v)) consider(paths, 4, -t, 0.5 * kPi, -u, -v);
  if (lpRmSmLm(x, -y, -phi, t, u, v)) consider(paths, 5, t, -0.5 * kPi, u, v);
  if (lpRmSmLm(-x, -y, phi, t, u, v)) consider(paths, 5, -t, 0.5 * kPi, -u, -v);
  if (lpRmSmRm(x, y, phi, t, u, v)) consider(paths, 8, t, -0.5 * kPi, u, v);
  if (lpRmSmRm(-x, y, -phi, t, u, v)) consider(paths, 8, -t, 0.5 * kPi, -u, -v);
  if (lpRmSmRm(x, -y, -phi, t, u, v)) consider(paths, 9, t, -0.5 * kPi, u, v);
  if (lpRmSmRm(-x, -y, phi, t, u, v)) consider(paths, 9, -t, 0.5 * kPi, -u, -v);

  const double backward_x = x * std::cos(phi) + y * std::sin(phi);
  const double backward_y = x * std::sin(phi) - y * std::cos(phi);
  if (lpRmSmLm(backward_x, backward_y, phi, t, u, v)) consider(paths, 6, v, u, -0.5 * kPi, t);
  if (lpRmSmLm(-backward_x, backward_y, -phi, t, u, v)) consider(paths, 6, -v, -u, 0.5 * kPi, -t);
  if (lpRmSmLm(backward_x, -backward_y, -phi, t, u, v)) consider(paths, 7, v, u, -0.5 * kPi, t);
  if (lpRmSmLm(-backward_x, -backward_y, phi, t, u, v)) consider(paths, 7, -v, -u, 0.5 * kPi, -t);
  if (lpRmSmRm(backward_x, backward_y, phi, t, u, v)) consider(paths, 10, v, u, -0.5 * kPi, t);
  if (lpRmSmRm(-backward_x, backward_y, -phi, t, u, v)) consider(paths, 10, -v, -u, 0.5 * kPi, -t);
  if (lpRmSmRm(backward_x, -backward_y, -phi, t, u, v)) consider(paths, 11, v, u, -0.5 * kPi, t);
  if (lpRmSmRm(-backward_x, -backward_y, phi, t, u, v)) consider(paths, 11, -v, -u, 0.5 * kPi, -t);
}

bool lpRmSLmRp(double x, double y, double phi,
               double& t, double& u, double& v)
{
  const double xi = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  double rho = 0.0;
  double theta = 0.0;
  polar(xi, eta, rho, theta);
  if (rho < 2.0)
    return false;
  u = 4.0 - std::sqrt(rho * rho - 4.0);
  if (u > kZero)
    return false;
  t = mod2pi(std::atan2((4.0 - u) * xi - 2.0 * eta,
                         -2.0 * xi + (u - 4.0) * eta));
  v = mod2pi(t - phi);
  return t >= -kZero && v >= -kZero;
}

void ccscc(double x, double y, double phi, std::vector<Path>& paths)
{
  double t = 0.0, u = 0.0, v = 0.0;
  if (lpRmSLmRp(x, y, phi, t, u, v)) consider(paths, 16, t, -0.5 * kPi, u, -0.5 * kPi, v);
  if (lpRmSLmRp(-x, y, -phi, t, u, v)) consider(paths, 16, -t, 0.5 * kPi, -u, 0.5 * kPi, -v);
  if (lpRmSLmRp(x, -y, -phi, t, u, v)) consider(paths, 17, t, -0.5 * kPi, u, -0.5 * kPi, v);
  if (lpRmSLmRp(-x, -y, phi, t, u, v)) consider(paths, 17, -t, 0.5 * kPi, -u, 0.5 * kPi, -v);
}

std::vector<Path> normalizedPaths(double x, double y, double yaw)
{
  std::vector<Path> paths;
  paths.reserve(48);
  csc(x, y, yaw, paths);
  ccc(x, y, yaw, paths);
  cccc(x, y, yaw, paths);
  ccsc(x, y, yaw, paths);
  ccscc(x, y, yaw, paths);
  return paths;
}

}  // namespace

bool ReedsSheppPath::valid() const
{
  return std::isfinite(normalized_length);
}

ReedsSheppPath shortestReedsSheppPath(
    double start_x, double start_y, double start_yaw,
    double goal_x, double goal_y, double goal_yaw,
    double turning_radius)
{
  if (!std::isfinite(start_x) || !std::isfinite(start_y) ||
      !std::isfinite(start_yaw) || !std::isfinite(goal_x) ||
      !std::isfinite(goal_y) || !std::isfinite(goal_yaw) ||
      !std::isfinite(turning_radius) || turning_radius <= 0.0)
  {
    return ReedsSheppPath();
  }
  const std::vector<ReedsSheppPath> paths = allReedsSheppPaths(
      start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw,
      turning_radius);
  if (paths.empty())
    return ReedsSheppPath();
  return *std::min_element(
      paths.begin(), paths.end(),
      [](const ReedsSheppPath& left, const ReedsSheppPath& right) {
        return left.normalized_length < right.normalized_length;
      });
}

std::vector<ReedsSheppPath> allReedsSheppPaths(
    double start_x, double start_y, double start_yaw,
    double goal_x, double goal_y, double goal_yaw,
    double turning_radius)
{
  if (!std::isfinite(start_x) || !std::isfinite(start_y) ||
      !std::isfinite(start_yaw) || !std::isfinite(goal_x) ||
      !std::isfinite(goal_y) || !std::isfinite(goal_yaw) ||
      !std::isfinite(turning_radius) || turning_radius <= 0.0)
  {
    return {};
  }
  const double delta_x = goal_x - start_x;
  const double delta_y = goal_y - start_y;
  const double cosine = std::cos(start_yaw);
  const double sine = std::sin(start_yaw);
  const double local_x = (cosine * delta_x + sine * delta_y) /
                         turning_radius;
  const double local_y = (-sine * delta_x + cosine * delta_y) /
                         turning_radius;
  return normalizedPaths(local_x, local_y, goal_yaw - start_yaw);
}

}  // namespace autolabor_coverage
