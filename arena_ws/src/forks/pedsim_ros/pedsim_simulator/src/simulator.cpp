/**
* Copyright 2014-2016 Social Robotics Lab, University of Freiburg
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*    # Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*    # Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*    # Neither the name of the University of Freiburg nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \author Billy Okal <okal@cs.uni-freiburg.de>
* \author Sven Wehner <mail@svenwehner.de>
*/

#include <QApplication>
#include <algorithm>

#include <pedsim_simulator/element/agentcluster.h>
#include <pedsim_simulator/scene.h>
#include <pedsim_simulator/simulator.h>

#include <pedsim_utils/geometry.h>

using namespace pedsim;

Simulator::Simulator(const ros::NodeHandle &node) : nh_(node)
{
  dynamic_reconfigure::Server<SimConfig>::CallbackType f;
  f = boost::bind(&Simulator::reconfigureCB, this, _1, _2);
  server_.setCallback(f);
}

Simulator::~Simulator()
{
  // shutdown service servers and publishers
  pub_obstacles_.shutdown();
  pub_agent_states_.shutdown();
  pub_agent_groups_.shutdown();
  pub_robot_position_.shutdown();
  pub_waypoints_.shutdown();

  srv_pause_simulation_.shutdown();
  srv_unpause_simulation_.shutdown();

  delete robot_;
  QCoreApplication::exit(0);
}

bool Simulator::initializeSimulation()
{
  int queue_size = 0;
  nh_.param<int>("default_queue_size", queue_size, 1);
  ROS_INFO_STREAM("Using default queue size of "
                  << queue_size << " for publisher queues... "
                  << (queue_size == 0
                          ? "NOTE: This means the queues are of infinite size!"
                          : ""));

  // setup ros publishers
  pub_obstacles_ =
      nh_.advertise<pedsim_msgs::LineObstacles>("simulated_walls", queue_size);
  pub_agent_states_ =
      nh_.advertise<pedsim_msgs::AgentStates>("simulated_agents", queue_size);
  pub_agent_groups_ =
      nh_.advertise<pedsim_msgs::AgentGroups>("simulated_groups", queue_size);
  pub_robot_position_ =
      nh_.advertise<nav_msgs::Odometry>("robot_position", queue_size);
  pub_waypoints_ =
      nh_.advertise<pedsim_msgs::Waypoints>("simulated_waypoints", queue_size);

  // services
  srv_pause_simulation_ = nh_.advertiseService(
      "pause_simulation", &Simulator::onPauseSimulation, this);
  srv_unpause_simulation_ = nh_.advertiseService(
      "unpause_simulation", &Simulator::onUnpauseSimulation, this);

  // setup TF listener and other pointers
  transform_listener_.reset(new tf::TransformListener());
  std::string odom_topic = "/odometry/ground_truth";
  odom_sub_ = nh_.subscribe(odom_topic, 1, &Simulator::odomCallback, this);

  robot_ = nullptr;

  // load additional parameters
  std::string scene_file_param;
  nh_.param<std::string>("scene_file", scene_file_param, "");
  if (scene_file_param == "")
  {
    ROS_ERROR_STREAM("Invalid scene file: " << scene_file_param);
    return false;
  }

  ROS_INFO_STREAM("Loading scene [" << scene_file_param << "] for simulation");

  const QString scenefile = QString::fromStdString(scene_file_param);
  ScenarioReader scenario_reader;
  if (scenario_reader.readFromFile(scenefile) == false)
  {
    ROS_ERROR_STREAM(
        "Could not load the scene file, please check the paths and param "
        "names : "
        << scene_file_param);
    return false;
  }

  nh_.param<bool>("enable_groups", CONFIG.groups_enabled, true);
  nh_.param<double>("max_robot_speed", CONFIG.max_robot_speed, 1.5);
  nh_.param<double>("pedsim_update_rate", CONFIG.updateRate, 25.0);
  nh_.param<double>("simulation_factor", CONFIG.simulationFactor, 1.0);
  nh_.param<bool>("/use_wall_mode", CONFIG.use_wall_mode, true);

  int op_mode = 1;
  nh_.param<int>("robot_mode", op_mode, 1);
  CONFIG.robot_mode = static_cast<RobotMode>(op_mode);

  double spawn_period;
  nh_.param<double>("spawn_period", spawn_period, 5.0);
  nh_.param<std::string>("frame_id", frame_id_, "odom");
  nh_.param<std::string>("robot_base_frame_id", robot_base_frame_id_,
                         "base_footprint");

  // spawn robot
  Agent *a = new Agent("myrobot");
  a->id = 0;
  Ped::Tagent::staticid = 1; // reset id so regular agents start at 1
  a->setType(Ped::Tagent::AgentType::ROBOT);
  SCENE.addAgent(a);
  SCENE.robot = a;
  robot_ = a;

  paused_ = false;
  last_sim_time = ros::Time::now();
  spawn_timer_ =
      nh_.createTimer(ros::Duration(spawn_period), &Simulator::spawnCallback, this);

  return true;
}

void Simulator::runSimulation()
{
  ros::WallRate r(CONFIG.updateRate);
  while (ros::ok())
  {
    if (!paused_)
    {
      // updateRobotPositionFromTF();
      // ros::Time now = ros::Time::now();
      // ros::Duration diff = now - last_sim_time;
      // last_sim_time = now;
      // SCENE.setTimeStepSize(diff.toSec() / 5.0); // slow down the simulation
      SCENE.moveAllAgents();

      for (auto agent : SCENE.getAgents())
      {
        agent->recordVelocity();
      }

      publishAgents();
      // publishGroups();
      // publishRobotPosition();
      publishObstacles();
      publishWaypoints();
    }
    ros::spinOnce();
    r.sleep();
  }
}

void Simulator::reconfigureCB(pedsim_simulator::PedsimSimulatorConfig &config,
                              uint32_t level)
{
  CONFIG.updateRate = config.update_rate;
  CONFIG.simulationFactor = config.simulation_factor;

  // update force scaling factors
  CONFIG.setObstacleForce(config.force_obstacle);
  CONFIG.setObstacleSigma(config.sigma_obstacle);
  CONFIG.setSocialForce(config.force_social);
  CONFIG.setGroupGazeForce(config.force_group_gaze);
  CONFIG.setGroupCoherenceForce(config.force_group_coherence);
  CONFIG.setGroupRepulsionForce(config.force_group_repulsion);
  CONFIG.setRandomForce(config.force_random);
  CONFIG.setAlongWallForce(config.force_wall);

  // puase or unpause the simulation
  if (paused_ != config.paused)
  {
    paused_ = config.paused;
  }

  ROS_INFO_STREAM("Updated sim with live config: Rate=" << CONFIG.updateRate
                                                        << " incoming rate="
                                                        << config.update_rate);
}

bool Simulator::onPauseSimulation(std_srvs::Empty::Request &request,
                                  std_srvs::Empty::Response &response)
{
  paused_ = true;
  return true;
}

bool Simulator::onUnpauseSimulation(std_srvs::Empty::Request &request,
                                    std_srvs::Empty::Response &response)
{
  paused_ = false;
  return true;
}

void Simulator::spawnCallback(const ros::TimerEvent &event)
{
  ROS_DEBUG_STREAM("Spawning new agents.");

  for (const auto &sa : SCENE.getSpawnAreas())
  {
    AgentCluster *agentCluster = new AgentCluster(sa->x, sa->y, sa->n);
    agentCluster->setDistribution(sa->dx, sa->dy);
    agentCluster->setType(static_cast<Ped::Tagent::AgentType>(0));

    for (const auto &wp_name : sa->waypoints)
    {
      agentCluster->addWaypoint(SCENE.getWaypointByName(wp_name));
    }

    SCENE.addAgentCluster(agentCluster);
  }
}

void Simulator::updateRobotPositionFromTF()
{
  if (!robot_)
    return;

  if (CONFIG.robot_mode == RobotMode::TELEOPERATION ||
      CONFIG.robot_mode == RobotMode::CONTROLLED)
  {
    robot_->setTeleop(true);
    robot_->setVmax(2 * CONFIG.max_robot_speed);

    // Get robot position via TF
    tf::StampedTransform tfTransform;
    try
    {
      transform_listener_->lookupTransform(frame_id_, robot_base_frame_id_,
                                           ros::Time(0), tfTransform);
    }
    catch (tf::TransformException &e)
    {
      ROS_WARN_STREAM_THROTTLE(
          5.0,
          "TF lookup from " << robot_base_frame_id_ << " to " << frame_id_
                            << " failed. Reason: " << e.what());
      return;
    }

    const double x = tfTransform.getOrigin().x();
    const double y = tfTransform.getOrigin().y();
    const double dx = x - last_robot_pose_.getOrigin().x(),
                 dy = y - last_robot_pose_.getOrigin().y();
    const double dt =
        tfTransform.stamp_.toSec() - last_robot_pose_.stamp_.toSec();
    double vx = dx / dt, vy = dy / dt;

    if (!std::isfinite(vx))
      vx = 0;
    if (!std::isfinite(vy))
      vy = 0;

    ROS_DEBUG_STREAM("rx, ry: " << robot_->getx() << ", " << robot_->gety() << " vs: " << x << ", " << y);

    robot_->setX(x);
    robot_->setY(y);
    robot_->setvx(vx);
    robot_->setvy(vy);

    ROS_DEBUG_STREAM("Robot speed: " << std::hypot(vx, vy) << " dt: " << dt);

    last_robot_pose_ = tfTransform;
  }
}

void Simulator::publishRobotPosition()
{
  if (robot_ == nullptr)
    return;

  nav_msgs::Odometry robot_location;
  robot_location.header = createMsgHeader();
  robot_location.child_frame_id = robot_base_frame_id_;

  robot_location.pose.pose.position.x = robot_->getx();
  robot_location.pose.pose.position.y = robot_->gety();
  if (hypot(robot_->getvx(), robot_->getvy()) < 0.05)
  {
    robot_location.pose.pose.orientation = last_robot_orientation_;
  }
  else
  {
    robot_location.pose.pose.orientation =
        poseFrom2DVelocity(robot_->getvx(), robot_->getvy());
    last_robot_orientation_ = robot_location.pose.pose.orientation;
  }

  robot_location.twist.twist.linear.x = robot_->getvx();
  robot_location.twist.twist.linear.y = robot_->getvy();

  pub_robot_position_.publish(robot_location);
}

void Simulator::publishAgents()
{
  if (SCENE.getAgents().size() < 1)
  {
    return;
  }

  pedsim_msgs::AgentStates all_status;
  all_status.header = createMsgHeader();

  auto VecToMsg = [](const Ped::Tvector &v)
  {
    geometry_msgs::Vector3 gv;
    gv.x = v.x;
    gv.y = v.y;
    gv.z = v.z;
    return gv;
  };

  for (const Agent *a : SCENE.getAgents())
  {
    // Skip robot.
    if (a->getType() == Ped::Tagent::ROBOT)
    {
      continue;
    }

    pedsim_msgs::AgentState state;
    state.header = createMsgHeader();

    state.id = a->getId();
    state.type = SCENE.agent_types[a->getType()];
    state.pose.position.x = a->getx();
    state.pose.position.y = a->gety();
    state.pose.position.z = a->getz();
    auto theta = std::atan2(a->getvy(), a->getvx());
    state.pose.orientation = pedsim::angleToQuaternion(theta);

    state.twist.linear.x = a->getvx();
    state.twist.linear.y = a->getvy();
    state.twist.linear.z = a->getvz();

    AgentStateMachine::AgentState sc = a->getStateMachine()->getCurrentState();
    state.social_state = agentStateToActivity(sc);

    // Forces.
    pedsim_msgs::AgentForce agent_forces;
    agent_forces.desired_force = VecToMsg(a->getDesiredDirection() * a->forceFactorDesired);
    agent_forces.obstacle_force = VecToMsg(a->getObstacleForce() * a->forceFactorObstacle);
    agent_forces.social_force = VecToMsg(a->getSocialForce() * a->forceFactorSocial);
    agent_forces.keep_distance_force = VecToMsg(a->getKeepDistanceForce());
    agent_forces.robot_force = VecToMsg(a->getRobotForce() * a->forceFactorRobot);
    // agent_forces.group_coherence_force = a->getSocialForce();
    // agent_forces.group_gaze_force = a->getSocialForce();
    // agent_forces.group_repulsion_force = a->getSocialForce();
    // agent_forces.random_force = a->getSocialForce();

    state.forces = agent_forces;

    state.talking_to_id = a->talkingToId;
    state.listening_to_id = a->listeningToId;

    state.acceleration = VecToMsg(a->getAcceleration());
    if (a->currentDestination == nullptr)
    {
      state.destination = VecToMsg(a->getPosition());
    }
    else
    {
      state.destination = VecToMsg(a->getCurrentDestination()->getPosition());
    }
    state.direction = a->facingDirection;

    all_status.agent_states.push_back(state);
    // ROS_WARN("publish agent states %d,%lf, typeID,%d",state.id,state.twist.linear.x,state.type);
  }

  pub_agent_states_.publish(all_status);
}

void Simulator::publishGroups()
{
  if (!CONFIG.groups_enabled)
  {
    ROS_DEBUG_STREAM("Groups are disabled, no group data published: flag="
                     << CONFIG.groups_enabled);
    return;
  }

  if (SCENE.getGroups().size() < 1)
  {
    return;
  }

  pedsim_msgs::AgentGroups sim_groups;
  sim_groups.header = createMsgHeader();

  for (const auto &ped_group : SCENE.getGroups())
  {
    if (ped_group->memberCount() <= 1)
      continue;

    pedsim_msgs::AgentGroup group;
    group.group_id = ped_group->getId();
    group.age = 10;
    const Ped::Tvector com = ped_group->getCenterOfMass();
    group.center_of_mass.position.x = com.x;
    group.center_of_mass.position.y = com.y;

    for (const auto &member : ped_group->getMembers())
    {
      group.members.emplace_back(member->getId());
    }
    sim_groups.groups.emplace_back(group);
  }
  pub_agent_groups_.publish(sim_groups);
}

void Simulator::publishObstacles()
{
  pedsim_msgs::LineObstacles sim_obstacles;
  sim_obstacles.header = createMsgHeader();
  for (const auto &obstacle : SCENE.getObstacles())
  {
    pedsim_msgs::LineObstacle line_obstacle;
    line_obstacle.start.x = obstacle->getax();
    line_obstacle.start.y = obstacle->getay();
    line_obstacle.start.z = 0.0;
    line_obstacle.end.x = obstacle->getbx();
    line_obstacle.end.y = obstacle->getby();
    line_obstacle.end.z = 0.0;
    sim_obstacles.obstacles.push_back(line_obstacle);
  }
  pub_obstacles_.publish(sim_obstacles);
}

void Simulator::publishWaypoints()
{
  pedsim_msgs::Waypoints sim_waypoints;
  sim_waypoints.header = createMsgHeader();
  for (const auto &waypoint : SCENE.getWaypoints())
  {
    pedsim_msgs::Waypoint wp;
    wp.name = waypoint->getName().toStdString();
    wp.type = waypoint->getType();
    wp.behavior = waypoint->getBehavior();
    wp.radius = waypoint->getRadius();
    wp.interaction_radius = waypoint->interactionRadius;
    wp.position.x = waypoint->getPosition().x;
    wp.position.y = waypoint->getPosition().y;
    sim_waypoints.waypoints.push_back(wp);
  }
  pub_waypoints_.publish(sim_waypoints);
}

std::string Simulator::agentStateToActivity(
    const AgentStateMachine::AgentState &state) const
{
  std::string activity = AgentStateMachine::stateToName(state).toStdString();
  return activity;
}

std_msgs::Header Simulator::createMsgHeader() const
{
  std_msgs::Header msg_header;
  msg_header.stamp = ros::Time::now();
  msg_header.frame_id = frame_id_;
  return msg_header;
}

void Simulator::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{
  robot_->setX(odom->pose.pose.position.x);
  robot_->setY(odom->pose.pose.position.y);
  robot_->setvx(odom->twist.twist.linear.x);
  robot_->setvy(odom->twist.twist.linear.y);
}
