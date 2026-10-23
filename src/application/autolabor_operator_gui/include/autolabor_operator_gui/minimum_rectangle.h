#pragma once
#include <geometry_msgs/Point.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace autolabor_operator_gui {
inline std::vector<geometry_msgs::Point> minimumRectangle(
    std::vector<geometry_msgs::Point> points) {
  for (const auto& p : points)
    if (!std::isfinite(p.x) || !std::isfinite(p.y))
      throw std::invalid_argument("选区坐标必须为有限数值");
  std::sort(points.begin(), points.end(), [](const auto& a, const auto& b) {
    return a.x < b.x || (a.x == b.x && a.y < b.y);
  });
  points.erase(std::unique(points.begin(), points.end(), [](const auto& a, const auto& b) {
    return a.x == b.x && a.y == b.y;
  }), points.end());
  if (points.size() < 3) throw std::invalid_argument("至少需要三个不共线的点");
  const auto cross = [](const auto& a, const auto& b, const auto& c) {
    return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
  };
  std::vector<geometry_msgs::Point> hull;
  for (const auto& p : points) {
    while (hull.size() >= 2 && cross(hull[hull.size()-2], hull.back(), p) <= 0)
      hull.pop_back();
    hull.push_back(p);
  }
  const auto lower = hull.size();
  for (auto it = points.rbegin()+1; it != points.rend(); ++it) {
    while (hull.size() > lower && cross(hull[hull.size()-2], hull.back(), *it) <= 0)
      hull.pop_back();
    hull.push_back(*it);
  }
  hull.pop_back();
  if (hull.size() < 3) throw std::invalid_argument("选区顶点不能共线");
  double best = -1;
  std::vector<geometry_msgs::Point> rectangle;
  const double ox = hull[0].x, oy = hull[0].y;
  for (size_t i = 0; i < hull.size(); ++i) {
    const auto& a = hull[i]; const auto& b = hull[(i+1)%hull.size()];
    const double angle = std::atan2(b.y-a.y,b.x-a.x), c = std::cos(angle), s = std::sin(angle);
    double u0=0,u1=0,v0=0,v1=0;
    for (const auto& p : hull) {
      const double u=(p.x-ox)*c+(p.y-oy)*s, v=-(p.x-ox)*s+(p.y-oy)*c;
      u0=std::min(u0,u);u1=std::max(u1,u);v0=std::min(v0,v);v1=std::max(v1,v);
    }
    const double area=(u1-u0)*(v1-v0);
    if (best < 0 || area < best-1e-9) {
      best=area; rectangle.clear();
      for (const auto& uv : std::vector<std::pair<double,double>>{{u0,v0},{u1,v0},{u1,v1},{u0,v1}}) {
        geometry_msgs::Point p; p.x=ox+uv.first*c-uv.second*s; p.y=oy+uv.first*s+uv.second*c;
        rectangle.push_back(p);
      }
    }
  }
  if (best < .5) throw std::invalid_argument("外接长方形面积不得小于 0.5 平方米");
  return rectangle;
}
}
