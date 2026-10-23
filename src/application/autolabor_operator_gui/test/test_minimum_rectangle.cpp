#include <autolabor_operator_gui/minimum_rectangle.h>
#include <gtest/gtest.h>
using namespace autolabor_operator_gui;
static geometry_msgs::Point p(double x,double y) {geometry_msgs::Point v;v.x=x;v.y=y;return v;}
static double area(const std::vector<geometry_msgs::Point>& r) {
  return std::hypot(r[1].x-r[0].x,r[1].y-r[0].y)*std::hypot(r[2].x-r[1].x,r[2].y-r[1].y);
}
TEST(MinimumRectangle, RotatedAndUnordered) {
  const double a=.39,c=std::cos(a),s=std::sin(a);
  std::vector<geometry_msgs::Point> points;
  for(const auto& v:std::vector<geometry_msgs::Point>{p(6,2),p(0,0),p(3,1),p(0,2),p(6,0),p(0,0)})
    points.push_back(p(1000+v.x*c-v.y*s,2000+v.x*s+v.y*c));
  const auto r=minimumRectangle(points);
  ASSERT_EQ(4u,r.size()); EXPECT_NEAR(12.,area(r),1e-8);
  for(const auto& v:points) for(size_t i=0;i<4;++i) {
    const auto& a=r[i];const auto& b=r[(i+1)%4];
    EXPECT_GE((b.x-a.x)*(v.y-a.y)-(b.y-a.y)*(v.x-a.x),-1e-8);
  }
  EXPECT_NEAR(area(r),area(minimumRectangle(r)),1e-8);
}
TEST(MinimumRectangle, RejectsDegenerateAndNonfinite) {
  EXPECT_THROW(minimumRectangle({p(0,0),p(1,1),p(2,2)}),std::invalid_argument);
  EXPECT_THROW(minimumRectangle({p(0,0),p(.1,0),p(0,.1)}),std::invalid_argument);
  EXPECT_THROW(minimumRectangle({p(0,0),p(1,0),p(NAN,1)}),std::invalid_argument);
}
int main(int argc,char** argv) {
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
