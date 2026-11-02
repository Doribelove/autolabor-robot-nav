// Offline C ABI for the production Hybrid A*; no ROS node or publishers.
#include <autolabor_coverage/hybrid_a_star.h>
#include <ros/time.h>
#include <navfn/navfn.h>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>

struct GaragePlanner {
  costmap_2d::Costmap2D map;
  std::vector<geometry_msgs::Point> footprint;
  std::string reason;
  GaragePlanner(unsigned w,unsigned h,double resolution,double x,double y,const unsigned char* data)
    : map(w,h,resolution,x,y,255), footprint(4) {
    std::memcpy(map.getCharMap(),data,w*h);
    footprint[0].x=1.32;footprint[0].y=.50;
    footprint[1].x=1.32;footprint[1].y=-.50;
    footprint[2].x=-.62;footprint[2].y=-.50;
    footprint[3].x=-.62;footprint[3].y=.50;
  }
};

extern "C" void* garage_create(unsigned w,unsigned h,double resolution,double x,double y,
                                const unsigned char* data) {
  ros::Time::init();
  return new GaragePlanner(w,h,resolution,x,y,data);
}
extern "C" void garage_destroy(void* handle) {delete static_cast<GaragePlanner*>(handle);}
extern "C" const char* garage_reason(void* handle) {return static_cast<GaragePlanner*>(handle)->reason.c_str();}
extern "C" int garage_navfn(void* handle,const double* start,const double* goal,
                            double* output,int capacity) {
  auto* p=static_cast<GaragePlanner*>(handle);unsigned sx,sy,gx,gy;
  if(!p->map.worldToMap(start[0],start[1],sx,sy)||!p->map.worldToMap(goal[0],goal[1],gx,gy))return 0;
  navfn::NavFn nav(p->map.getSizeInCellsX(),p->map.getSizeInCellsY());
  nav.setCostmap(p->map.getCharMap(),true,false);
  int a[2]={static_cast<int>(sx),static_cast<int>(sy)};
  int b[2]={static_cast<int>(gx),static_cast<int>(gy)};
  nav.setStart(a);nav.setGoal(b);
  if(!nav.calcNavFnAstar())return 0;
  int count=nav.calcPath(capacity);
  if(count<=0)return 0;
  for(int i=0;i<count;++i){
    output[2*i]=p->map.getOriginX()+(nav.getPathX()[i]+.5)*p->map.getResolution();
    output[2*i+1]=p->map.getOriginY()+(nav.getPathY()[i]+.5)*p->map.getResolution();
  }
  return count;
}
extern "C" int garage_plan(void* handle,const double* start,const double* goal,double seconds,
                           double* output,int capacity,double* metrics) {
  auto* p=static_cast<GaragePlanner*>(handle);
  auto pose=[](const double* values) {
    geometry_msgs::PoseStamped v;v.header.frame_id="map";
    v.pose.position.x=values[0];v.pose.position.y=values[1];
    v.pose.orientation.z=std::sin(values[2]/2);v.pose.orientation.w=std::cos(values[2]/2);
    return v;
  };
  autolabor_coverage::HybridAStarConfig config;
  config.minimum_turning_radius=1.35;config.planning_timeout=seconds;
  config.max_expansions=400000;config.collision_check_step=.05;
  autolabor_coverage::HybridAStarProfile profile;
  autolabor_coverage::HybridAStarStatistics statistics;
  std::vector<geometry_msgs::PoseStamped> result;
  autolabor_coverage::HybridAStarPlanner planner;
  const bool ok=planner.makePlan(&p->map,p->footprint,pose(start),pose(goal),config,profile,
                                 result,statistics,p->reason);
  metrics[0]=statistics.expansions;metrics[1]=statistics.estimated_time;
  metrics[2]=statistics.reverse_distance;metrics[3]=statistics.direction_changes;
  if(!ok)return 0;
  if(result.size()>static_cast<size_t>(capacity)){p->reason="output capacity exceeded";return -1;}
  for(size_t i=0;i<result.size();++i){
    const auto& v=result[i].pose;output[3*i]=v.position.x;output[3*i+1]=v.position.y;
    output[3*i+2]=std::atan2(2*v.orientation.w*v.orientation.z,
                           1-2*v.orientation.z*v.orientation.z);
  }
  return static_cast<int>(result.size());
}
