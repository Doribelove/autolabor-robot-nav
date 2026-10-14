#include <autolabor_gps_keepout/KeepoutPolygons.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/cost_values.h>
#include <pluginlib/class_loader.h>
#include <tf2_ros/buffer.h>
#include <gtest/gtest.h>

TEST(KeepoutLayer, MarksGlobalAndRollingMapsAndClearsDeletedAreas)
{
  ros::NodeHandle node;
  auto publisher=node.advertise<autolabor_gps_keepout::KeepoutPolygons>("/gps/keepout/polygons",1,true);
  pluginlib::ClassLoader<costmap_2d::Layer> loader("costmap_2d","costmap_2d::Layer");
  tf2_ros::Buffer buffer;
  costmap_2d::LayeredCostmap global("camera_init",false,false);
  costmap_2d::LayeredCostmap rolling("camera_init",true,false);
  global.resizeMap(400,400,.1,-20,-20);
  rolling.resizeMap(160,160,.1,-8,-8);
  auto layer=loader.createInstance("autolabor_gps_keepout/KeepoutLayer");
  auto local=loader.createInstance("autolabor_gps_keepout/KeepoutLayer");
  global.addPlugin(layer);rolling.addPlugin(local);
  layer->initialize(&global,"global/gps_keepout",&buffer);
  local->initialize(&rolling,"local/gps_keepout",&buffer);
  auto cycle=[&](const autolabor_gps_keepout::KeepoutPolygons& message,double x=0.){
    publisher.publish(message);
    for(int i=0;i<15;++i){ros::WallDuration(.02).sleep();ros::spinOnce();global.updateMap(x,0,0);rolling.updateMap(x,0,0);}
  };
  auto cost=[](costmap_2d::LayeredCostmap& map,double x,double y){
    unsigned int mx,my;
    if(!map.getCostmap()->worldToMap(x,y,mx,my))return -1;
    return int(map.getCostmap()->getCost(mx,my));
  };
  autolabor_gps_keepout::KeepoutPolygons msg;msg.header.frame_id="camera_init";
  geometry_msgs::Polygon polygon;
  for(auto xy:std::vector<std::pair<float,float>>{{4,-2},{6,-2},{6,2},{4,2}})
  {geometry_msgs::Point32 p;p.x=xy.first;p.y=xy.second;polygon.points.push_back(p);}
  msg.polygons.push_back(polygon);cycle(msg);
  EXPECT_EQ(cost(global,5,0),costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(cost(rolling,5,0),costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(cost(global,3,0),costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(rolling,3,0),costmap_2d::FREE_SPACE);
  cycle(msg,8.);
  EXPECT_EQ(cost(rolling,5,0),costmap_2d::LETHAL_OBSTACLE);
  msg.polygons.clear();cycle(msg,8.);
  EXPECT_EQ(cost(global,5,0),costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(rolling,5,0),costmap_2d::FREE_SPACE);
  EXPECT_TRUE(layer->isCurrent());EXPECT_TRUE(local->isCurrent());
  // A permitted outer boundary with an internal hole, plus a manual keepout.
  auto rectangle=[](float x0,float y0,float x1,float y1){
    geometry_msgs::Polygon p;
    for(auto xy:std::vector<std::pair<float,float>>{{x0,y0},{x1,y0},{x1,y1},{x0,y1}})
    {geometry_msgs::Point32 point;point.x=xy.first;point.y=xy.second;p.points.push_back(point);}
    return p;
  };
  msg.keep_in_enabled=true;
  msg.keep_in_polygons={rectangle(-6,-6,12,6),rectangle(4,-2,6,2)};
  msg.polygons={rectangle(8,-2,10,2)};
  cycle(msg,8.);
  for(auto* map : {&global,&rolling})
  {
    EXPECT_EQ(cost(*map,3,0),costmap_2d::FREE_SPACE);
    EXPECT_EQ(cost(*map,5,0),costmap_2d::LETHAL_OBSTACLE);
    EXPECT_EQ(cost(*map,9,0),costmap_2d::LETHAL_OBSTACLE);
    EXPECT_EQ(cost(*map,13,0),costmap_2d::LETHAL_OBSTACLE);
  }
  msg.keep_in_polygons.clear();cycle(msg,8.);
  EXPECT_EQ(cost(global,3,0),costmap_2d::LETHAL_OBSTACLE);
  msg.keep_in_enabled=false;msg.polygons.clear();cycle(msg,8.);
  EXPECT_EQ(cost(global,3,0),costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(rolling,13,0),costmap_2d::FREE_SPACE);
}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"test_keepout_layer");
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
