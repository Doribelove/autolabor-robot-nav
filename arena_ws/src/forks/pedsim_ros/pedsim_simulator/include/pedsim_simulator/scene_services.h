/*
 * @name	 	scene_services.cpp
 * @brief	 	Provides services to spawn and remove pedestrians dynamically. 
 *          The spawned agents are forwarded to flatland
 * @author 	Ronja Gueldenring
 * @date 		2019/04/05
 **/

#ifndef _scene_service_h_
#define _scene_service_h_

#include <ros/ros.h>
#include <pedsim_simulator/scene.h>
#include <pedsim_simulator/scenarioreader.h>
#include <pedsim_simulator/waypointplanner/waypointplanner.h>
#include <pedsim_srvs/SpawnPeds.h>
#include <pedsim_srvs/SpawnObstacle.h>
#include <pedsim_srvs/SpawnInteractiveObstacles.h>
#include <pedsim_srvs/MovePeds.h>
#include <pedsim_srvs/SetObstacles.h>
#include <gazebo_msgs/ModelState.h>
#include <pedsim_msgs/Ped.h>
#include <pedsim_msgs/LineObstacle.h>
#include <pedsim_msgs/LineObstacles.h>
#include <pedsim_msgs/InteractiveObstacle.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <yaml-cpp/yaml.h>

/**
 * This class provides services to spawn and remove pedestrians dynamically.
 */
class SceneServices
{
  // Constructor and Destructor
public:
  SceneServices();
  virtual ~SceneServices() = default;

  ros::ServiceServer respawn_peds_service_;
  ros::ServiceServer remove_all_peds_service_;
  ros::ServiceServer remove_all_peds_behavior_modelling_service_;
  ros::ServiceServer spawn_ped_service_;
  ros::ServiceServer add_obstacle_service_;
  ros::ServiceServer move_peds_service_;
  ros::ServiceServer spawn_peds_service_;
  ros::ServiceServer reset_peds_service_;
  ros::ServiceServer respawn_interactive_obstacles_service_;
  // ros::ServiceServer spawn_interactive_obstacles_service_;
  ros::ServiceServer remove_all_interactive_obstacles_service_;
  ros::ServiceServer set_obstacles_service_;
  static int agents_index_;
  static int static_obstacles_index_;
  static std::vector<std::string> static_obstacle_names_;

  /**
  * @brief Spawns pedestrian in pedsim and flatland.
  */
  bool spawnPeds(pedsim_srvs::SpawnPeds::Request &request, pedsim_srvs::SpawnPeds::Response &response);

  /**
  * @brief Removes all pedestrians in flatland.
  */
  bool removeAllPeds(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);

  /**
  * @brief Resets all pedestrians to their initial position and state
  */
  bool resetPeds(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);

  /**
  * @brief Repawn interactive obstacles
  */
  // bool respawnInteractiveObstacles(pedsim_srvs::SpawnInteractiveObstacles::Request &request, pedsim_srvs::SpawnInteractiveObstacles::Response &response);

  /**
  * @brief Spawn interactive obstacles
  */
  // bool spawnInteractiveObstacles(pedsim_srvs::SpawnInteractiveObstacles::Request &request, pedsim_srvs::SpawnInteractiveObstacles::Response &response);

  /**
  * @brief Remove all interactive obstacles
  */
  bool removeAllInteractiveObstacles(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);

  /**
  * @brief Respawning means reusing objects from previous tasks.
  * It is a more efficient way to setup a task during learning.
  */
  bool respawnPeds(pedsim_srvs::SpawnPeds::Request &request,
                   pedsim_srvs::SpawnPeds::Response &response);

  bool moveAgentClustersInPedsim(pedsim_srvs::MovePeds::Request &request,
                                 pedsim_srvs::MovePeds::Response &response);

  /**
  * @brief Adding static obstacles to pedsim.
  */
  bool addStaticObstacles(pedsim_srvs::SpawnObstacle::Request &request,
                          pedsim_srvs::SpawnObstacle::Response &response);
  /**
   * @brief Removing existing obstacles and adding new ones from the requested map file
   */
  bool setObstacles(pedsim_srvs::SetObstacles::Request &request, pedsim_srvs::SetObstacles::Response &response);

protected:
  ros::NodeHandle nh_;

private:
  std::vector<std::string> removePedsInPedsim();
  void addAgentClusterToPedsim(pedsim_msgs::Ped ped, std::vector<int> ids);
  std::vector<gazebo_msgs::ModelState> getGazeboModels(pedsim_msgs::Ped ped, std::vector<int> ids);
  std::vector<int> generateAgentIds(int n);
  bool removeModelsInGazebo(std::vector<std::string> model_names);
  bool spawnModelsInGazebo(std::vector<gazebo_msgs::ModelState> gazebo_models);
  // bool respawnModelsInFlatland(std::vector<std::string> old_model_names, std::vector<flatland_msgs::Model> new_models);
  void removeAllReferencesToInteractiveObstacles();
  void removeAllInteractiveObstaclesFromPedsim();
  std::vector<Obstacle *> getWallsFromFlatlandModel(pedsim_msgs::InteractiveObstacle obstacle, double yaw);
  int stringToEnumIndex(std::string str, std::vector<std::string> values);

  std::string spawn_gazebo_topic_;
  ros::ServiceClient spawn_gazebo_client_;
  std::string human_model;
  // std::string respawn_models_topic_;
  // ros::ServiceClient respawn_models_client_;

  std::string delete_models_topic_;
  ros::ServiceClient delete_models_client_;

  std::vector<Obstacle *> walls;
};

#endif /* _scene_service_h_ */
