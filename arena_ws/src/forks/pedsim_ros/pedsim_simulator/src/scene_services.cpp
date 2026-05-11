/*
 * @name	 	scene_services.cpp
 * @brief	 	Provides services to spawn and remove pedestrians dynamically and add static obstacle spawning. 
 *          The spawned agents are forwarded to flatland
 * @author 	Ronja Gueldenring
 * @date 		2019/04/05
 * @author Junhui Li
 * @date 24.3.2021
 **/

#include <pedsim_simulator/element/agent.h>
#include <pedsim_simulator/element/agentcluster.h>
#include <pedsim_simulator/element/areawaypoint.h>
#include <pedsim_simulator/element/obstacle.h>
#include <pedsim_simulator/scene_services.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <std_srvs/SetBool.h>
#include <iostream>
#include <pedsim_simulator/rng.h>
#include <ros/package.h>
#include <fstream>

int SceneServices::agents_index_ = 1;
int SceneServices::static_obstacles_index_ = 1;
std::vector<std::string> SceneServices::static_obstacle_names_;

SceneServices::SceneServices()
{
  // pedsim services
  respawn_peds_service_ = nh_.advertiseService("pedsim_simulator/respawn_peds", &SceneServices::respawnPeds, this);
  spawn_peds_service_ = nh_.advertiseService("pedsim_simulator/spawn_peds", &SceneServices::spawnPeds, this);
  remove_all_peds_service_ = nh_.advertiseService("pedsim_simulator/remove_all_peds", &SceneServices::removeAllPeds, this);
  reset_peds_service_ = nh_.advertiseService("pedsim_simulator/reset_all_peds", &SceneServices::resetPeds, this);
  add_obstacle_service_ = nh_.advertiseService("pedsim_simulator/add_obstacle", &SceneServices::addStaticObstacles, this);
  move_peds_service_ = nh_.advertiseService("pedsim_simulator/move_peds", &SceneServices::moveAgentClustersInPedsim, this);
  // respawn_interactive_obstacles_service_ = nh_.advertiseService("pedsim_simulator/respawn_interactive_obstacles", &SceneServices::respawnInteractiveObstacles, this);
  // spawn_interactive_obstacles_service_ = nh_.advertiseService("pedsim_simulator/spawn_interactive_obstacles", &SceneServices::spawnInteractiveObstacles, this);
  remove_all_interactive_obstacles_service_ = nh_.advertiseService("pedsim_simulator/remove_all_interactive_obstacles", &SceneServices::removeAllInteractiveObstacles, this);
  set_obstacles_service_ = nh_.advertiseService("pedsim_simulator/set_obstacles", &SceneServices::setObstacles, this);

  //gazebo service clients
  spawn_gazebo_topic_ = "gazebo/spawn_sdf_model";
  spawn_gazebo_client_ = nh_.serviceClient<gazebo_msgs::SpawnModel>(spawn_gazebo_topic_, true);

  delete_models_topic_ = "gazebo/delete_model";
  delete_models_client_ = nh_.serviceClient<gazebo_msgs::DeleteModel>(delete_models_topic_, true);

  // read in default human gazebo model
  std::string path = ros::package::getPath("pedsim_gazebo_plugin");
  std::ifstream f(path + "/models/actor_model.sdf");
  std::stringstream ss;
  ss << f.rdbuf();
  human_model = ss.str();
}

bool SceneServices::setObstacles(pedsim_srvs::SetObstacles::Request &request, pedsim_srvs::SetObstacles::Response &response)
{
  // ROS_ERROR("GOT obstacles message");
  SCENE.removeAllObstacles();
  std::string sim_setup_path = ros::package::getPath("simulator_setup");
  std::string path = sim_setup_path + "/scenarios/ped_scenarios/" + request.map_name + ".xml";
  const QString scenefile = QString::fromStdString(path);
  ScenarioReader scenario_reader;
  if (scenario_reader.readFromFile(scenefile) == false)
  {
    ROS_ERROR_STREAM(
        "Could not load the scene file, please check the paths and param "
        "names : "
        << path);
    return false;
  }
  response.finished = true;
  return true;
}

bool SceneServices::spawnPeds(pedsim_srvs::SpawnPeds::Request &request, pedsim_srvs::SpawnPeds::Response &response)
{
  std::vector<gazebo_msgs::ModelState> gazebo_models;

  for (int ped_i = 0; ped_i < (int)request.peds.size(); ped_i++)
  {
    pedsim_msgs::Ped ped = request.peds[ped_i];
    std::vector<int> new_agent_ids = generateAgentIds(ped.number_of_peds);

    // add ped to pedsim
    addAgentClusterToPedsim(ped, new_agent_ids);

    // add gazebo model states to spawn_models service request
    std::vector<gazebo_msgs::ModelState> new_gz_models = getGazeboModels(ped, new_agent_ids);
    gazebo_models.insert(gazebo_models.end(), new_gz_models.begin(), new_gz_models.end());
  }

  return true;
}

bool SceneServices::respawnPeds(pedsim_srvs::SpawnPeds::Request &request,
                                pedsim_srvs::SpawnPeds::Response &response)
{
  ROS_INFO("called respawnPeds() with %ld peds", request.peds.size());

  std_srvs::SetBool::Request request_;
  std_srvs::SetBool::Response response_;
  bool res = removeAllPeds(request_, response_);
  if (!res)
  {
    response.success = false;
    return false;
  }

  res = spawnPeds(request, response);
  if (!res)
  {
    response.success = false;
    return false;
  }

  response.success = true;
  return true;
}

bool SceneServices::removeAllPeds(std_srvs::SetBool::Request &request,
                                  std_srvs::SetBool::Response &response)
{
  std::vector<std::string> model_names = removePedsInPedsim();
  // bool res = removeModelsInGazebo(model_names);
  // response.success = res;
  return true;
}

// remove all agents and return a list of their names
std::vector<std::string> SceneServices::removePedsInPedsim()
{
  // Remove all waypoints
  auto waypoints = SCENE.getWaypoints();
  for (auto waypoint : waypoints.values())
  {
    if (!waypoint->isInteractive())
    {
      SCENE.removeWaypoint(waypoint);
    }
  }

  // Remove all agents
  std::vector<std::string> names;
  QList<Agent *> agents = SCENE.getAgents();
  for (Agent *a : agents)
  {
    // don't remove robot
    if (a->getType() == Ped::Tagent::ROBOT)
    {
      continue;
    }
    names.push_back(a->agentName);
    SCENE.removeAgent(a);
  }

  // reset agent counters
  Ped::Tagent::staticid = 1;
  agents_index_ = 1;

  return names;
}

// bool SceneServices::respawnInteractiveObstacles(pedsim_srvs::SpawnInteractiveObstacles::Request &request, pedsim_srvs::SpawnInteractiveObstacles::Response &response)
// {
//   // remove old
//   std_srvs::Trigger::Request trigger_request;
//   std_srvs::Trigger::Response trigger_response;
//   removeAllInteractiveObstacles(trigger_request, trigger_response);

//   // spawn new
//   bool res = spawnInteractiveObstacles(request, response);

//   response.success = res;
//   return res;
// }

// bool SceneServices::spawnInteractiveObstacles(pedsim_srvs::SpawnInteractiveObstacles::Request &request, pedsim_srvs::SpawnInteractiveObstacles::Response &response)
// {
//   std::vector<flatland_msgs::Model> new_models;

//   for (auto obstacle : request.obstacles)
//   {
//     // get name
//     std::string name = "";
//     if (obstacle.name == "")
//     {
//       name = "interactive_waypoint_" + std::to_string(static_obstacles_index_);
//     }
//     else
//     {
//       name = obstacle.name;
//     }
//     static_obstacles_index_++;
//     static_obstacle_names_.push_back(name);

//     // get random angle
//     uniform_real_distribution<double> Distribution(0.0, 2 * M_PI);
//     double yaw = Distribution(RNG());

//     // add to pedsim
//     auto direction = Ped::Tvector::fromPolar(Ped::Tangle::fromRadian(yaw), 2.0);
//     auto waypoint_pos = Ped::Tvector(obstacle.pose.position.x, obstacle.pose.position.y) + direction;
//     auto waypoint = new AreaWaypoint(QString(name.c_str()), waypoint_pos, 0.3);
//     waypoint->interactionRadius = obstacle.interaction_radius;
//     if (obstacle.interaction_radius < 0.1)
//     {
//       ROS_WARN("interaction_radius is smaller than 0.1. agents will not interact with this obstacle");
//     }
//     waypoint->staticObstacleAngle = fmod(yaw + M_PI, 2 * M_PI);

//     // set type
//     std::string type_string = obstacle.type;
//     int type = 0;
//     // convert type string to enum value
//     auto types = SCENE.obstacle_types;
//     auto it = find(types.begin(), types.end(), type_string);
//     // If element was found
//     if (it != types.end())
//     {
//       type = it - types.begin();
//     }
//     waypoint->setType(static_cast<Ped::Twaypoint::WaypointType>(type));

//     // set radius for obstacle force calculation
//     int radius = 1.0;
//     int radius_index = waypoint->getType();
//     if (radius_index < (int)SCENE.obstacle_radius.size())
//     {
//       radius = SCENE.obstacle_radius[radius_index];
//     }
//     waypoint->modelRadius = radius;

//     SCENE.addWaypoint(waypoint);

//     // create circular obstacles for applying obstacle force to agents
//     auto circle_obstacle = Ped::Twaypoint(obstacle.pose.position.x, obstacle.pose.position.y);
//     circle_obstacle.modelRadius = radius;
//     SCENE.circleObstacles.push_back(circle_obstacle);

//     // create wall obstacles along model footprint for applying obstacle force to agents
//     std::vector<Obstacle *> new_walls = getWallsFromFlatlandModel(obstacle, yaw);
//     // append to current list of walls
//     walls.insert(walls.end(), new_walls.begin(), new_walls.end());
//     // spawn walls in pedsim
//     for (auto wall : new_walls)
//     {
//       SCENE.addObstacle(wall);
//     }

//     // create flatland model
//     flatland_msgs::Model model;
//     model.name = name;
//     model.ns = name;
//     model.pose.x = obstacle.pose.position.x;
//     model.pose.y = obstacle.pose.position.y;
//     model.pose.theta = yaw;
//     model.yaml_path = obstacle.yaml_path;
//     new_models.push_back(model);
//   }

//   auto res = spawnModelsInFlatland(new_models);

//   return res;
// }

void SceneServices::removeAllReferencesToInteractiveObstacles()
{
  // check agents that are referencing interactive waypoints
  auto agents = SCENE.getAgents();
  for (auto agent : agents)
  {
    // check lastInteractedWithWaypoint
    if (agent->lastInteractedWithWaypoint != nullptr)
    {
      if (agent->lastInteractedWithWaypoint->isInteractive())
      {
        agent->lastInteractedWithWaypoint = nullptr;
        agent->lastInteractedWithWaypointId = -1;
      }
    }

    // check lastInteractedWithWaypointId
    if (agent->lastInteractedWithWaypointId != -1)
    {
      agent->lastInteractedWithWaypoint = nullptr;
      agent->lastInteractedWithWaypointId = -1;
    }

    // check currentDestination
    if (agent->currentDestination != nullptr)
    {
      if (agent->currentDestination->isInteractive())
      {
        agent->updateDestination();
      }
    }

    // check waypointplanner
    auto waypointplanner = agent->getWaypointPlanner();
    if (waypointplanner != nullptr)
    {
      auto waypoint = waypointplanner->getCurrentWaypoint();
      if (waypoint != nullptr)
      {
        if (waypoint->isInteractive())
        {
          // waypointplanner is pointing to an interactive obstacle
          // -> just copy destination from agent
          // we ensured above that the current agent destination is not interactive
          waypointplanner->setDestination(agent->getCurrentDestination());
        }
      }
    }
  }
}

void SceneServices::removeAllInteractiveObstaclesFromPedsim()
{
  removeAllReferencesToInteractiveObstacles();

  // actually remove waypoints from scene
  auto waypoints = SCENE.getWaypoints();
  std::vector<Waypoint *> to_remove;
  for (auto waypoint : waypoints.values())
  {
    if (waypoint->isInteractive())
    {
      to_remove.push_back(waypoint);
    }
  }

  for (auto waypoint : to_remove)
  {
    SCENE.removeWaypoint(waypoint);
  }

  // remove point obstacles
  SCENE.circleObstacles.clear();

  // remove walls
  for (auto wall : walls)
  {
    SCENE.removeObstacle(wall);
  }
  walls.clear();
}

bool SceneServices::removeAllInteractiveObstacles(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  removeAllInteractiveObstaclesFromPedsim();
  static_obstacles_index_ = 1;
  static_obstacle_names_.clear();
  response.success = true;
  response.message = "";
  return true;
}

bool SceneServices::resetPeds(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  QList<Agent *> agents = SCENE.getAgents();
  for (Agent *a : agents)
  {
    a->reset();
  }

  response.success = true;
  return true;
}

int SceneServices::stringToEnumIndex(std::string str, std::vector<std::string> values)
{
  int index = 0;
  auto it = find(values.begin(), values.end(), str);
  // If element was found
  if (it != values.end())
  {
    index = it - values.begin();
  }
  return index;
}

void SceneServices::addAgentClusterToPedsim(pedsim_msgs::Ped ped, std::vector<int> ids)
{
  uniform_real_distribution<double> distribution_x(ped.pos.x - 1.0, ped.pos.x + 1.0);
  uniform_real_distribution<double> distribution_y(ped.pos.y - 1.0, ped.pos.y + 1.0);

  for (int i = 0; i < ped.number_of_peds; i++)
  {
    std::string name = "person_" + std::to_string(ids[i]);
    Agent *a = new Agent(name);

    // randomize location if we have more than one agent
    if (ped.number_of_peds > 1)
    {
      a->setPosition(distribution_x(RNG()), distribution_y(RNG()));
    }
    else
    {
      a->setPosition(ped.pos.x, ped.pos.y);
    }

    // record initial positions used when agent is reset
    a->initialPosX = a->getx();
    a->initialPosY = a->gety();

    // set type
    a->setType(static_cast<Ped::Tagent::AgentType>(stringToEnumIndex(ped.type, SCENE.agent_types)));

    // set agent radius
    int radius = 1.0;
    int radius_index = a->getType();
    if (radius_index < (int)SCENE.agent_radius.size())
    {
      radius = SCENE.agent_radius[radius_index];
    }
    a->SetRadius(radius);

    a->setVmax(ped.vmax);
    if (a->getType() == Ped::Tagent::ELDER)
    {
      a->setVmax(ped.vmax * 0.7);
    }
    a->vmaxDefault = a->getVmax();

    // startup mode
    a->startUpMode = static_cast<Agent::StartUpMode>(stringToEnumIndex(ped.start_up_mode, SCENE.start_up_modes));
    if (a->startUpMode == Agent::StartUpMode::WAITTIMER)
    {
      a->getStateMachine()->activateState(AgentStateMachine::AgentState::StateWaitForTimer);
    }
    else if (a->startUpMode == Agent::StartUpMode::TRIGGERZONE)
    {
      a->getStateMachine()->activateState(AgentStateMachine::AgentState::StateWaitForTrigger);
    }
    else
    {
      a->getStateMachine()->activateState(AgentStateMachine::AgentState::StateNone);
    }
    // wait time
    a->waitTime = ped.wait_time;
    // trigger zone radius
    a->triggerZoneRadius = ped.trigger_zone_radius;

    a->chattingProbability = ped.chatting_probability;
    a->stateTalkingBaseTime = ped.talking_base_time;

    a->tellStoryProbability = ped.tell_story_probability;
    a->stateTellStoryBaseTime = ped.tell_story_base_time;

    a->groupTalkingProbability = ped.group_talking_probability;
    a->stateGroupTalkingBaseTime = ped.group_talking_base_time;

    a->talkingAndWalkingProbability = ped.talking_and_walking_probability;
    a->stateTalkingAndWalkingBaseTime = ped.talking_and_walking_base_time;

    a->requestingServiceProbability = ped.requesting_service_probability;
    a->stateRequestingServiceBaseTime = ped.requesting_service_base_time;
    a->stateReceivingServiceBaseTime = ped.receiving_service_base_time;

    a->requestingGuideProbability = ped.requesting_guide_probability;

    a->requestingFollowerProbability = ped.requesting_follower_probability;

    a->maxTalkingDistance = ped.max_talking_distance;
    if (
        a->getType() == Ped::Tagent::AgentType::ADULT ||
        a->getType() == Ped::Tagent::AgentType::ELDER ||
        a->getType() == Ped::Tagent::AgentType::CHILD)
    {
      if (a->maxTalkingDistance < 0.1)
      {
        ROS_WARN("maxTalkingDistance is very small. ped will probably not interact with others");
      }
    }

    a->maxServicingRadius = ped.max_servicing_radius;
    if (a->getType() == Ped::Tagent::AgentType::SERVICEROBOT)
    {
      if (a->maxServicingRadius < 0.1)
      {
        ROS_WARN("maxServicingRadius is very small. service robot will not provide service");
      }
    }

    a->waypointMode = static_cast<Agent::WaypointMode>(ped.waypoint_mode);

    // set force factors
    a->forceFactorDesired = ped.force_factor_desired;
    if (a->forceFactorDesired < 0.1)
    {
      ROS_ERROR("forceFactorDesired is very small. ped will probably not move");
    }
    a->forceFactorObstacle = ped.force_factor_obstacle;
    a->forceFactorSocial = ped.force_factor_social;
    a->forceFactorRobot = ped.force_factor_robot;

    // add waypoints to agentcluster and scene
    for (int i = 0; i < (int)ped.waypoints.size(); i++)
    {
      const double x = ped.waypoints[i].x;
      const double y = ped.waypoints[i].y;
      AreaWaypoint *w = new AreaWaypoint(QString(std::to_string(Ped::Twaypoint::staticid).c_str()), x, y, 0.3);
      uniform_real_distribution<double> Distribution(0.0, 2 * M_PI);
      w->staticObstacleAngle = Distribution(RNG());
      w->setBehavior(static_cast<Ped::Twaypoint::Behavior>(0));
      SCENE.addWaypoint(w);
      a->addWaypoint(w);
    }

    // add agent to scene
    SCENE.addAgent(a);
  }
}

std::vector<gazebo_msgs::ModelState> SceneServices::getGazeboModels(pedsim_msgs::Ped ped, std::vector<int> ids)
{
  std::vector<gazebo_msgs::ModelState> model_states;

  for (int i = 0; i < ped.number_of_peds; i++)
  {
    gazebo_msgs::ModelState model;
    model.model_name = std::to_string(ids[i]);
    geometry_msgs::Pose pose;
    pose.position = ped.pos;
    pose.orientation = geometry_msgs::Quaternion();
    model.pose = pose;
    model_states.push_back(model);
  }

  return model_states;
}
bool SceneServices::addStaticObstacles(pedsim_srvs::SpawnObstacle::Request &request,
                                       pedsim_srvs::SpawnObstacle::Response &response)
{
  int k = (int)request.staticObstacles.obstacles.size();
  for (int i = 0; i < k; i++)
  {
    pedsim_msgs::LineObstacle obstacle = request.staticObstacles.obstacles[i];
    Obstacle *o = new Obstacle(obstacle.start.x, obstacle.start.y, obstacle.end.x, obstacle.end.y);
    SCENE.addObstacle(o);
  }

  response.finished = true;
  return true;
}

bool SceneServices::moveAgentClustersInPedsim(pedsim_srvs::MovePeds::Request &request,
                                              pedsim_srvs::MovePeds::Response &response)
{
  SCENE.episode = request.episode;
  SCENE.moveClusters(request.episode);
  // static obstacle infos are updated every episode too
  SCENE.removeAllObstacles();
  response.finished = true;
  return true;
}

std::vector<int> SceneServices::generateAgentIds(int n)
{
  std::vector<int> ids;
  for (int i = 0; i < n; i++)
  {
    ids.push_back(agents_index_);
    agents_index_++;
  }
  return ids;
}

bool SceneServices::removeModelsInGazebo(std::vector<std::string> model_names)
{
  // ROS_INFO("deleting %ld models", model_names.size());
  for (const auto &model : model_names)
  {
    gazebo_msgs::DeleteModel msg;
    msg.request.model_name = model;

    // check validity of client
    while (!delete_models_client_.isValid())
    {
      ROS_WARN("Reconnecting delete_model_client_-server....");
      delete_models_client_.waitForExistence(ros::Duration(5.0));
      delete_models_client_ = nh_.serviceClient<gazebo_msgs::DeleteModel>(delete_models_topic_, true);
    }
    ROS_INFO("Deleting model with name %s", model.c_str());
    // delete_models_client_.call(msg);

    if (!msg.response.success)
    {
      ROS_ERROR("Failed to delete model %s.", model.c_str());
      return false;
    }
  }
  return true;
}

bool SceneServices::spawnModelsInGazebo(std::vector<gazebo_msgs::ModelState> gazebo_models)
{
  for (auto const &model : gazebo_models)
  {
    gazebo_msgs::SpawnModel srv;
    srv.request.model_name = "person_" + model.model_name; // Ped ID !
    srv.request.model_xml = human_model;
    srv.request.robot_namespace = "/" + model.model_name;
    srv.request.reference_frame = "world";

    // check validity of client
    while (!spawn_gazebo_client_.isValid())
    {
      ROS_WARN("Reconnecting spawn_models_client_-server....");
      spawn_gazebo_client_.waitForExistence(ros::Duration(5.0));
      spawn_gazebo_client_ = nh_.serviceClient<gazebo_msgs::SpawnModel>(spawn_gazebo_topic_, true);
    }
    spawn_gazebo_client_.call(srv);

    if (!srv.response.success)
    {
      ROS_ERROR("Failed to respawn models.");
      return false;
    }
  }

  return true;
}

// bool SceneServices::respawnModelsInFlatland(std::vector<std::string> old_model_names, std::vector<flatland_msgs::Model> new_models) {
//   flatland_msgs::RespawnModels msg;
//   msg.request.old_model_names = old_model_names;
//   msg.request.new_models = new_models;

//   // check validity of client
//   while (!respawn_models_client_.isValid()) {
//     ROS_WARN("Reconnecting respawn_models_client_-server....");
//     respawn_models_client_.waitForExistence(ros::Duration(5.0));
//     respawn_models_client_ = nh_.serviceClient<flatland_msgs::RespawnModels>(respawn_models_topic_, true);
//   }

//   respawn_models_client_.call(msg);

//   if (!msg.response.success) {
//     ROS_ERROR("Failed to respawn models. Flatland response: %s", msg.response.message.c_str());
//     return false;
//   }

//   return true;
// }

std::vector<Obstacle *> SceneServices::getWallsFromFlatlandModel(pedsim_msgs::InteractiveObstacle obstacle, double yaw)
{
  std::vector<Obstacle *> new_walls;
  YAML::Node model = YAML::LoadFile(obstacle.yaml_path);
  auto bodies = model["bodies"];
  if (bodies.size() < 1)
  {
    ROS_ERROR("getWallsFromFlatlandModel(): No body found in yaml node.");
    return std::vector<Obstacle *>();
  }

  auto footprints = bodies[0]["footprints"];
  if (footprints.size() < 1)
  {
    ROS_ERROR("getWallsFromFlatlandModel(): No footprint found in body.");
    return std::vector<Obstacle *>();
  }

  auto footprint = footprints[0];
  auto type = footprint["type"];
  if (type == NULL)
  {
    ROS_ERROR("getWallsFromFlatlandModel(): No type found in footprint.");
    return std::vector<Obstacle *>();
  }

  auto pos = obstacle.pose.position;
  if (type.as<std::string>() == "polygon")
  {
    auto points = footprint["points"];
    for (int i = 0; i < (int)points.size() - 1; i++)
    {
      auto start = Ped::Tvector(points[i][0].as<float>(), points[i][1].as<float>());
      start.rotate(Ped::Tangle::fromRadian(yaw));
      auto end = Ped::Tvector(points[i + 1][0].as<float>(), points[i + 1][1].as<float>());
      end.rotate(Ped::Tangle::fromRadian(yaw));
      Obstacle *wall = new Obstacle(
          pos.x + start.x,
          pos.y + start.y,
          pos.x + end.x,
          pos.y + end.y);
      new_walls.push_back(wall);
    }

    // connect last and first point
    auto first = Ped::Tvector(points[0][0].as<float>(), points[0][1].as<float>());
    first.rotate(Ped::Tangle::fromRadian(yaw));
    auto last = Ped::Tvector(points[points.size() - 1][0].as<float>(), points[points.size() - 1][1].as<float>());
    last.rotate(Ped::Tangle::fromRadian(yaw));
    Obstacle *closing_wall = new Obstacle(
        pos.x + first.x,
        pos.y + first.y,
        pos.x + last.x,
        pos.y + last.y);
    new_walls.push_back(closing_wall);
  }
  else if (type.as<std::string>() == "circle")
  {
    // TODO handle circles
  }
  return new_walls;
}
