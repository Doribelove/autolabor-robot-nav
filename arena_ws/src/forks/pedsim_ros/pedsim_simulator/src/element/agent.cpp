/**
* Copyright 2014 Social Robotics Lab, University of Freiburg
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

#include <pedsim_simulator/agentstatemachine.h>
#include <pedsim_simulator/config.h>
#include <pedsim_simulator/element/agent.h>
#include <pedsim_simulator/element/waypoint.h>
#include <pedsim_simulator/force/force.h>
#include <pedsim_simulator/scene.h>
#include <pedsim_simulator/waypointplanner/waypointplanner.h>
#include <pedsim_simulator/rng.h>
#include <ros/ros.h>

Agent::Agent() {
  Ped::Tagent::setType(Ped::Tagent::ADULT);
  Ped::Tagent::setForceFactorObstacle(CONFIG.forceObstacle);
  forceSigmaObstacle = CONFIG.sigmaObstacle;
  Ped::Tagent::setForceFactorSocial(CONFIG.forceSocial);

  initialPosX = 0.0;
  initialPosY = 0.0;

  waypointMode = WaypointMode::LOOP;
  currentDestination = nullptr;
  destinationIndex = 0;
  previousDestinationIndex = 0;
  nextDestinationIndex = 0;
  lastInteractedWithWaypointId = -1;
  lastInteractedWithWaypoint = nullptr;
  isInteracting = false;

  maxTalkingDistance = 1.5;
  talkingToId = -1;
  talkingToAgent = nullptr;
  listeningToId = -1;
  listeningToAgent = nullptr;
  maxServicingRadius = 10.0;
  servicingAgent = nullptr;
  servicingWaypoint = nullptr;
  currentServiceRobot = nullptr;
  followWaypoint = nullptr;

  facingDirection = 0.0;

  angleTarget = 0.0;
  timeStepSize = 0.02;

  stateMachine = new AgentStateMachine(this);

  chattingProbability = 0.01;
  tellStoryProbability = 0.01;
  groupTalkingProbability = 0.01;
  talkingAndWalkingProbability = 0.01;
  switchRunningWalkingProbability = 0.1;
  requestingServiceProbability = 0.1;
  requestingGuideProbability = 0.1;
  requestingFollowerProbability = 0.1;

  lastStartTalkingCheck = ros::Time::now();
  lastTellStoryCheck = ros::Time::now();
  lastGroupTalkingCheck = ros::Time::now();
  lastStartTalkingAndWalkingCheck = ros::Time::now();
  lastSwitchRunningWalkingCheck = ros::Time::now();
  lastRequestingServiceCheck = ros::Time::now();
  lastRequestingGuideCheck = ros::Time::now();
  lastRequestingFollowerCheck = ros::Time::now();

  stateWorkingBaseTime = 30.0;
  stateLiftingForksBaseTime = 3.0;
  stateLoadingBaseTime = 3.0;
  stateLoweringForksBaseTime = 3.0;
  stateTalkingBaseTime = 10.0;
  stateTellStoryBaseTime = 20.0;
  stateGroupTalkingBaseTime = 20.0;
  stateTalkingAndWalkingBaseTime = 9.0;
  stateRequestingServiceBaseTime = 30.0;
  stateReceivingServiceBaseTime = 30.0;

  lastIsStuckCheck = ros::Time::now();
  lastRecordedVelocityTime = ros::Time::now();
  recordedVelocitiesIndex = 0;
  velocitiesRecorded = 0;

  subGoal = nullptr;
  arenaGoal = nullptr;
  hasRequestedFollower = false;

  group = nullptr;
  waypointplanner = nullptr;
  disableForce("KeepDistance");
  disableForce("Robot");

  lastVarySpeed = ros::Time::now();

  startUpMode = StartUpMode::DEFAULT;
  waitTime = 0;
  waitTimer = ros::Time::now();
  triggerZoneRadius = 0;
}

Agent::Agent(std::string name) : Agent() {
  agentName = name;
}

Agent::Agent(const Agent&) : ScenarioElement() {}

Agent::~Agent() {
  // clean up
  foreach (Force* currentForce, forces) { delete currentForce; }
}

/// Calculates the desired force. Same as in lib, but adds graphical
/// representation
Ped::Tvector Agent::desiredForce() {
  Ped::Tvector force;
  if (!disabledForces.contains("Desired")) force = Tagent::desiredForce();

  // inform users
  emit desiredForceChanged(force.x, force.y);

  return force;
}

/// Calculates the social force. Same as in lib, but adds graphical
/// representation
Ped::Tvector Agent::socialForce() const {
  Ped::Tvector force;
  if (!disabledForces.contains("Social")) force = Tagent::socialForce();

  // inform users
  emit socialForceChanged(force.x, force.y);

  return force;
}

double Agent::obstacleForceFunction(double distanceIn) {
  if (distanceIn <= 0) {
    // distance < 0 means a collision so return a big force
    return 10.0;
  }

  return 1.0 / distanceIn;
}

/// Calculates the obstacle force. Same as in lib, but adds graphical
/// representation
Ped::Tvector Agent::obstacleForce() {
  Ped::Tvector force;
  if (!disabledForces.contains("Obstacle")) {
    if (CONFIG.use_wall_mode) {
      force = Ped::Tagent::obstacleForce();
    } else {
      Ped::Twaypoint closest_obstacle;
      if (!SCENE.getClosestObstacle(p, &closest_obstacle)) {
        return Ped::Tvector(0.0, 0.0);
      }

      Ped::Tvector diff = p - closest_obstacle.getPosition();
      double force_magnitude = obstacleForceFunction(diff.length() - agentRadius - closest_obstacle.modelRadius);
      Ped::Tvector force_direction = diff.normalized();
      force = force_direction * force_magnitude;
    }
  }

  // inform users
  emit obstacleForceChanged(force.x, force.y);

  return force;
}

Ped::Tvector Agent::keepDistanceForce() {
  Ped::Tvector force;
  if (!disabledForces.contains("KeepDistance")) force = Tagent::keepDistanceForce();

  return force;
}

Ped::Tvector Agent::robotForce() {
  Ped::Tvector force;

  if (!disabledForces.contains("Robot")) {
    if (SCENE.robot != nullptr) {
      auto diff = p - SCENE.robot->getPosition();
      auto dist = diff.length();
      if (dist < 4.0) {
        double amount = 10.0;
        if (dist > 0.0) {
          amount = 1.0 / dist;
        }
        force = amount * diff.normalized();
      }
    }
  }

  return force;
}

Ped::Tvector Agent::myForce(Ped::Tvector desired) const {
  // run additional forces
  Ped::Tvector forceValue;
  foreach (Force* force, forces) {
    // skip disabled forces
    if (disabledForces.contains(force->getName())) {
      // update graphical representation
      emit additionalForceChanged(force->getName(), 0, 0);
      continue;
    }

    // add force to the total force
    Ped::Tvector currentForce = force->getForce(desired);
    // → sanity checks
    if (!currentForce.isValid()) {
      ROS_DEBUG("Invalid Force: %s", force->getName().toStdString().c_str());
      currentForce = Ped::Tvector();
    }
    forceValue += currentForce;

    // update graphical representation
    emit additionalForceChanged(force->getName(), currentForce.x,
                                currentForce.y);
  }

  // inform users
  emit myForceChanged(forceValue.x, forceValue.y);

  return forceValue;
}

Waypoint* Agent::getCurrentDestination() const {
  return currentDestination;
}

void Agent::reset() {
  // reset position
  setPosition(initialPosX, initialPosY);

  // reset destination
  previousDestinationIndex = 0;
  destinationIndex = 0;
  nextDestinationIndex = 0;

  // reset state
  if (startUpMode == Agent::StartUpMode::WAITTIMER) {
    waitTimer = ros::Time::now();
    stateMachine->activateState(AgentStateMachine::AgentState::StateWaitForTimer);
  } else if (startUpMode == Agent::StartUpMode::TRIGGERZONE) {
    stateMachine->activateState(AgentStateMachine::AgentState::StateWaitForTrigger);
  } else {
    stateMachine->activateState(AgentStateMachine::AgentState::StateNone);
  }
}

Waypoint* Agent::getPreviousDestination() {
  return destinations[previousDestinationIndex];
}

Ped::Twaypoint* Agent::updateDestination() {
  // if (id == 1) ROS_WARN("updating destination, current index: %d", destinationIndex);
  
  // assign new destination
  if (!destinations.isEmpty()) {
    previousDestinationIndex = destinationIndex;
    destinationIndex = nextDestinationIndex;
    currentDestination = destinations[destinationIndex];

    if (waypointMode == WaypointMode::RANDOM) {
      // choose random destination
      while (nextDestinationIndex == destinationIndex && destinations.length() > 1) {
        nextDestinationIndex = rand() % destinations.count();
      }
    } else {
      // cycle through destinations
      nextDestinationIndex = (nextDestinationIndex + 1) % destinations.count();
    }
  }

  return currentDestination;
}

void Agent::updateState() {
  // check state
  stateMachine->doStateTransition();
}

// update direction the agent is facing based on the state
void Agent::updateDirection() {
  switch (stateMachine->getCurrentState()) {
    case AgentStateMachine::AgentState::StateWalking:
      if (v.length() > 0.001) {
        facingDirection = v.polarAngle().toRadian(Ped::Tangle::PositiveOnlyRange);
      }
      break;
    case AgentStateMachine::AgentState::StateListening:
      facingDirection = (keepDistanceTo - p).polarAngle().toRadian(Ped::Tangle::PositiveOnlyRange);
      break;
    case AgentStateMachine::AgentState::StateGroupTalking:
      facingDirection = (keepDistanceTo - p).polarAngle().toRadian(Ped::Tangle::PositiveOnlyRange);
      break;
    case AgentStateMachine::AgentState::StateLiftingForks:
      assert(lastInteractedWithWaypoint != nullptr);
      facingDirection = lastInteractedWithWaypoint->staticObstacleAngle;
      break;
    case AgentStateMachine::AgentState::StateLoading:
      assert(lastInteractedWithWaypoint != nullptr);
      facingDirection = lastInteractedWithWaypoint->staticObstacleAngle;
      break;
    case AgentStateMachine::AgentState::StateLoweringForks:
      assert(lastInteractedWithWaypoint != nullptr);
      facingDirection = lastInteractedWithWaypoint->staticObstacleAngle;
      break;
    case AgentStateMachine::AgentState::StateReachedShelf:
      // do nothing
      break;
    case AgentStateMachine::AgentState::StateBackUp:
      // do nothing
      break;
    case AgentStateMachine::AgentState::StateTalking:
      facingDirection = (talkingToAgent->getPosition() - p).polarAngle().toRadian(Ped::Tangle::PositiveOnlyRange);
      break;
    case AgentStateMachine::AgentState::StateReceivingService:
      if (currentServiceRobot != nullptr) {
        facingDirection = (currentServiceRobot->getPosition() - p).polarAngle().toRadian(Ped::Tangle::PositiveOnlyRange);
      }
      break;
    default:
      if (v.length() > 0.001) {
        facingDirection = v.polarAngle().toRadian(Ped::Tangle::PositiveOnlyRange);
      }
      break;
  }
}

// in: angle in radians
// out: angle in radians between 0 and 2*PI
double Agent::normalizeAngle(double angle_in) {
  double angle = angle_in;
  while (angle < 0) {
    angle += 2 * M_PI;
  }

  while (angle > 2 * M_PI) {
    angle -= 2 * M_PI;
  }

  return angle;
}

double Agent::rotate(double current_angle, double target_angle, double time_step, double angular_v) {
  double current_angle_normalized = normalizeAngle(current_angle);
  double target_angle_normalized = normalizeAngle(target_angle);
  double angle = time_step * angular_v;
  double angle_diff = normalizeAngle(target_angle_normalized - current_angle_normalized);

  if (angle_diff > M_PI) {
    angle *= -1;
  }

  return current_angle_normalized + angle;
}

bool Agent::completedMoveList() {
  auto last_move = moveList.back();
  return ros::Time::now() > last_move.timestamp;
}

void Agent::moveByMoveList() {
  // TODO this can be sped up by projecting the current time onto the range between start time and end time to find the right move index
  auto now = ros::Time::now();
  double min_time_diff = INFINITY;
  AgentPoseStamped new_pose;
  // find pose in move list that is closest to ros::Time::now()
  for (auto pose : moveList) {
    double time_diff = abs((now - pose.timestamp).toSec());
    if (time_diff < min_time_diff) {
      min_time_diff = time_diff;
      new_pose = pose;
    }
  }
  p = new_pose.pos;
  facingDirection = new_pose.theta;
}

std::vector<AgentPoseStamped> Agent::createMoveListStateReachedShelf() {
  std::vector<AgentPoseStamped> moves;
  double linear_v = 0.5;
  double angular_v = 0.5;
  double temp_direction = facingDirection;
  Ped::Tvector temp_pos = p;
  ros::Time temp_time = ros::Time::now() + ros::Duration(1.0);

  // do rotation
  // while not reached target angle
  while (abs(fmod(temp_direction, 2 * M_PI) - angleTarget) > 0.1) {
    AgentPoseStamped pose = AgentPoseStamped(temp_time, temp_pos, temp_direction);
    moves.push_back(pose);

    temp_direction = rotate(temp_direction, angleTarget, timeStepSize, angular_v);
    temp_time += ros::Duration(timeStepSize);
  }

  // do short move forward
  Ped::Tvector target_pos = temp_pos + Ped::Tvector::fromPolar(Ped::Tangle::fromRadian(temp_direction), 1.0);  // 1.0m forward in the current direction
  double original_diff = (target_pos - temp_pos).length();
  while ((temp_pos - target_pos).length() > 0.1) {
    if ((temp_pos - target_pos).length() > original_diff + 1.0) {
      ROS_ERROR("overshot target");
      break;
    }
    AgentPoseStamped pose = AgentPoseStamped(temp_time, temp_pos, temp_direction);
    moves.push_back(pose);

    // move linearly in direction with linear_v
    temp_pos += Ped::Tvector::fromPolar(Ped::Tangle::fromRadian(temp_direction), 1.0) * linear_v * timeStepSize;
    temp_time += ros::Duration(timeStepSize);
  }

  return moves;
}

std::vector<AgentPoseStamped> Agent::createMoveListStateBackUp() {
  std::vector<AgentPoseStamped> moves;
  double linear_v = 0.5;
  double angular_v = 0.5;
  double temp_direction = facingDirection;
  Ped::Tvector temp_pos = p;
  ros::Time temp_time = ros::Time::now() + ros::Duration(1.0);

  // move backwards
  Ped::Tvector target_pos = temp_pos + Ped::Tvector::fromPolar(Ped::Tangle::fromRadian(temp_direction + M_PI), 1.0);  // 1.0m backwards in the current direction
  double original_diff = (target_pos - temp_pos).length();
  while ((temp_pos - target_pos).length() > 0.1) {
    if ((temp_pos - target_pos).length() > original_diff + 1.0) {
      ROS_ERROR("overshot target");
      break;
    }

    AgentPoseStamped pose = AgentPoseStamped(temp_time, temp_pos, temp_direction);
    moves.push_back(pose);

    // move linearly in direction with linear_v
    temp_pos += Ped::Tvector::fromPolar(Ped::Tangle::fromRadian(temp_direction + M_PI), 1.0) * linear_v * timeStepSize;
    temp_time += ros::Duration(timeStepSize);
  }

  // turn to next destination
  // auto next_destination = destinations[nextDestinationIndex];
  auto next_destination_direction = currentDestination->getPosition() - temp_pos;
  double angle_target = next_destination_direction.polarAngle().toRadian(Ped::Tangle::AngleRange::PositiveOnlyRange);
  // while not reached target angle
  while (abs(fmod(temp_direction, 2 * M_PI) - angle_target) > 0.1) {
    AgentPoseStamped pose = AgentPoseStamped(temp_time, temp_pos, temp_direction);
    moves.push_back(pose);

    temp_direction = rotate(temp_direction, angle_target, timeStepSize, angular_v);
    temp_time += ros::Duration(timeStepSize);
  }

  return moves;
}

std::vector<AgentPoseStamped> Agent::createMoveList(AgentStateMachine::AgentState state) {
  std::vector<AgentPoseStamped> moves;
  switch (state) {
  case AgentStateMachine::AgentState::StateReachedShelf:
    moves = createMoveListStateReachedShelf();
    break;
  case AgentStateMachine::AgentState::StateBackUp:
    moves = createMoveListStateBackUp();
    break;
  default:
    break;
  }
  return moves;
}

void Agent::move(double h) {
  if (getType() == Ped::Tagent::ROBOT) {
    if (CONFIG.robot_mode == RobotMode::TELEOPERATION) {
      // NOTE: Moving is now done by setting x, y position directly in
      // simulator.cpp
      // Robot's vx, vy will still be set for the social force model to work
      // properly wrt. other agents.

      // FIXME: This is a very hacky way of making the robot "move" (=update
      // position in hash tree) without actually moving it
      const double vx = getvx();
      const double vy = getvy();

      setvx(0);
      setvy(0);
      Ped::Tagent::move(h);
      setvx(vx);
      setvy(vy);
    } else if (CONFIG.robot_mode == RobotMode::CONTROLLED) {
      if (SCENE.getTime() >= CONFIG.robot_wait_time) {
        Ped::Tagent::move(h);
      }
    } else if (CONFIG.robot_mode == RobotMode::SOCIAL_DRIVE) {
      Ped::Tagent::setForceFactorSocial(CONFIG.forceSocial * 0.7);
      Ped::Tagent::setForceFactorObstacle(35);
      Ped::Tagent::setForceFactorDesired(4.2);

      Ped::Tagent::setVmax(1.6);
      Ped::Tagent::SetRadius(0.4);
      Ped::Tagent::move(h);
    }
  } else {
    // special cases for some states
    auto state = stateMachine->getCurrentState();
    if (state == AgentStateMachine::AgentState::StateListeningAndWalking) {
      // simulate movement by always placing the agent next to listeningToAgent
      Ped::Tvector neighbor_v = listeningToAgent->getVelocity();
      double angle = 0.5 * M_PI;
      Ped::Tvector neighbor_v_rotated = neighbor_v.x * Ped::Tvector(cos(angle), sin(angle)) + neighbor_v.y * Ped::Tvector(-1 * sin(angle), cos(angle));
      neighbor_v_rotated.normalize();
      // set new position every tick instead of "normal" movement
      p = listeningToAgent->getPosition() + (keepDistanceForceDistanceDefault * neighbor_v_rotated);
      // copy v from neighbor
      v = neighbor_v;
    } else if (state == AgentStateMachine::AgentState::StateReachedShelf){
      moveByMoveList();
    } else if (state == AgentStateMachine::AgentState::StateBackUp){
      moveByMoveList();
    } else {
      // normal movement
      Ped::Tagent::move(h);
    }
    updateDirection();
  }

  // inform users
  emit positionChanged(getx(), gety());
  emit velocityChanged(getvx(), getvy());
  emit accelerationChanged(getax(), getay());
}

const QList<Waypoint*>& Agent::getWaypoints() const { return destinations; }

bool Agent::setWaypoints(const QList<Waypoint*>& waypointsIn) {
  destinations = waypointsIn;
  return true;
}

bool Agent::addWaypoint(Waypoint* waypointIn) {
  destinations.append(waypointIn);
  return true;
}

bool Agent::removeWaypoint(Waypoint* waypointIn) {
  const int removeCount = destinations.removeAll(waypointIn);

  return (removeCount > 0);
}

bool Agent::needNewDestination() const {
  if (waypointplanner == nullptr)
    return (!destinations.isEmpty());
  else {
    // ask waypoint planner
    return waypointplanner->hasCompletedDestination();
  }
}

bool Agent::hasCompletedDestination() const {
  if (waypointplanner == nullptr)
  {
    return false;
  }
  return waypointplanner->hasCompletedDestination();
}

Ped::Twaypoint* Agent::getCurrentWaypoint() const {
  // sanity checks
  if (waypointplanner == nullptr) return nullptr;

  // ask waypoint planner
  return waypointplanner->getCurrentWaypoint();
}

bool Agent::isInGroup() const { return (group != nullptr); }

AgentGroup* Agent::getGroup() const { return group; }

void Agent::setGroup(AgentGroup* groupIn) { group = groupIn; }

bool Agent::addForce(Force* forceIn) {
  forces.append(forceIn);

  // inform users
  emit forceAdded(forceIn->getName());

  // report success
  return true;
}

bool Agent::removeForce(Force* forceIn) {
  int removeCount = forces.removeAll(forceIn);

  // inform users
  emit forceRemoved(forceIn->getName());

  // report success if a Behavior has been removed
  return (removeCount >= 1);
}

AgentStateMachine* Agent::getStateMachine() const { return stateMachine; }

WaypointPlanner* Agent::getWaypointPlanner() const { return waypointplanner; }

void Agent::setWaypointPlanner(WaypointPlanner* plannerIn) {
  waypointplanner = plannerIn;
}

QList<const Agent*> Agent::getNeighbors() const {
  // upcast neighbors
  QList<const Agent*> output;
  for (const Ped::Tagent* neighbor : neighbors) {
    const Agent* upNeighbor = dynamic_cast<const Agent*>(neighbor);
    // neighbor->getPosition();
    if (upNeighbor != nullptr) output.append(upNeighbor);
  }

  return output;
}

std::vector<Agent*> Agent::getAgentsInRange(double distance) {
  std::vector<Agent*> agents;
  double distance_squared = distance * distance;
  for (auto agent : SCENE.getAgents()) {
    // skip self
    if (agent->getId() == id) {
      continue;
    }

    auto diff = p - agent->getPosition();
    if (diff.lengthSquared() < distance_squared) {
      agents.push_back(agent);
    }
  }

  return agents;
}

QList<const Agent*> Agent::getPotentialListeners(double distance) {
  QList<const Agent*> agents_with_state;
  auto agents = getAgentsInRange(distance);
  for (auto agent : agents) {
    if (
      agent->getStateMachine()->getCurrentState() == AgentStateMachine::AgentState::StateWalking ||
      agent->getStateMachine()->getCurrentState() == AgentStateMachine::AgentState::StateRunning
    ) {
      agents_with_state.append(agent);
    }
  }
  return agents_with_state;
}

Waypoint* Agent::getInteractiveObstacleInRange(int type) {
  auto waypoints = SCENE.getWaypoints();
  for (auto waypoint : waypoints.values()) {
    if (waypoint->getType() == type) {
      auto diff = waypoint->getPosition() - p;
      if (diff.lengthSquared() < pow(waypoint->interactionRadius, 2)) {
        return waypoint;
      }
    }
  }
  return nullptr;
}

bool Agent::someoneTalkingToMe() {
  auto neighbor_list = getAgentsInRange(maxTalkingDistance);
  for (const Agent* neighbor: neighbor_list) {
    if (neighbor == nullptr) {
      ROS_WARN("neighbor is nullptr. skip it.");
      continue;
    }
    
    if (
      neighbor->getStateMachine()->getCurrentState() == AgentStateMachine::AgentState::StateTellStory ||
      (
        neighbor->getStateMachine()->getCurrentState() == AgentStateMachine::AgentState::StateTalking &&
        neighbor->talkingToId == id
      )
    ) {
      // neighbor is telling a story or specifically talking to me
      listeningToId = neighbor->getId();
      listeningToAgent = SCENE.getAgent(neighbor->getId());
      keepDistanceTo = listeningToAgent->getPosition();
      keepDistanceForceDistance = keepDistanceForceDistanceDefault;
      return true;
    } else if (
      neighbor->getStateMachine()->getCurrentState() == AgentStateMachine::AgentState::StateGroupTalking
    ) {
      // neighbor started a group talk
      listeningToId = neighbor->getId();
      listeningToAgent = SCENE.getAgent(neighbor->getId());
      // copy talking center from neighbor
      keepDistanceTo = neighbor->keepDistanceTo;
      keepDistanceForceDistance = keepDistanceForceDistanceDefault;
      return true;
    } else if (
      neighbor->getStateMachine()->getCurrentState() == AgentStateMachine::AgentState::StateTalkingAndWalking && 
      neighbor->talkingToId == id
    ) {
      // neighbor talks to me while walking
      listeningToId = neighbor->getId();
      listeningToAgent = SCENE.getAgent(neighbor->getId());
      return true;
    }
  }
  return false;
}

bool Agent::isListeningToIndividual() {
  if (listeningToAgent != nullptr) {
    if (listeningToAgent->getStateMachine()->getCurrentState() == AgentStateMachine::AgentState::StateTalking) {
      return true;
    }
  }

  return false;
}

bool Agent::tellStory() {
  ros::Time now = ros::Time::now();
  // only do the probability check again after some time has passed
  if ((now - lastTellStoryCheck).toSec() > 0.5) {
    // reset timer
    lastTellStoryCheck = ros::Time::now();

    auto potentialChatters = getAgentsInRange(maxTalkingDistance);
    // only tell story if there are multiple people around
    if (potentialChatters.size() > 2) {
      // don't tell a story if someone else already is
      for (auto chatter: potentialChatters) {
        if (chatter->getStateMachine()->getCurrentState() == AgentStateMachine::AgentState::StateTellStory) {
          return false;
        }
      }

      // start telling a story with a given probability
      uniform_real_distribution<double> Distribution(0, 1);
      double roll = Distribution(RNG());
      if (roll < tellStoryProbability) {
        return true;
      }
    }
  }

  return false;
}


bool Agent::startGroupTalking() {
  ros::Time now = ros::Time::now();
  // only do the probability check again after some time has passed
  if ((now - lastGroupTalkingCheck).toSec() > 0.5) {
    // reset timer
    lastGroupTalkingCheck = ros::Time::now();

    QList<const Agent*> potentialChatters = getPotentialListeners(maxTalkingDistance);
    // only group talk if there are multiple people around
    if (potentialChatters.length() > 2) {
      // don't start a group talk if someone else already is
      for (const Agent* chatter: potentialChatters) {
        if (chatter->getStateMachine()->getCurrentState() == AgentStateMachine::AgentState::StateGroupTalking) {
          return false;
        }
      }

      // start group talk with a given probability
      uniform_real_distribution<double> Distribution(0, 1);
      double roll = Distribution(RNG());
      if (roll < groupTalkingProbability) {
        // use my own current position as center of group
        keepDistanceTo = p;
        return true;
      }
    }
  }

  return false;
}


bool Agent::startTalking(){
  // only do the probability check again after some time has passed
  ros::Time now = ros::Time::now();
  if ((now - lastStartTalkingCheck).toSec() > 0.5) {
    // reset timer
    lastStartTalkingCheck = ros::Time::now();

    // start talking sometimes when there is someone near
    QList<const Agent*> potentialChatters = getPotentialListeners(maxTalkingDistance);
    if (!potentialChatters.isEmpty()) {
      // roll a dice
      uniform_real_distribution<double> Distribution(0, 1);
      double roll = Distribution(RNG());
      if (roll < chattingProbability) {
        // start chatting to a random person in range
        int idx = std::rand() % potentialChatters.length();
        talkingToId = potentialChatters[idx]->getId();
        talkingToAgent = potentialChatters[idx];
        return true;
      }
    }
  }

  return false;
}


bool Agent::startTalkingAndWalking(){
  // only do the probability check again after some time has passed
  ros::Time now = ros::Time::now();
  if ((now - lastStartTalkingAndWalkingCheck).toSec() > 0.5) {
    // reset timer
    lastStartTalkingAndWalkingCheck = ros::Time::now();

    // start talking sometimes when there is someone near
    QList<const Agent*> potentialChatters = getPotentialListeners(maxTalkingDistance);
    if (!potentialChatters.isEmpty()) {
      // roll a dice
      uniform_real_distribution<double> Distribution(0, 1);
      double roll = Distribution(RNG());
      if (roll < talkingAndWalkingProbability) {
        // start chatting to a random person in range
        int idx = std::rand() % potentialChatters.length();
        this->talkingToId = potentialChatters[idx]->getId();
        return true;
      }
    }
  }

  return false;
}

bool Agent::startRequestingService() {
  if (SCENE.serviceRobotExists) {
    // only do the probability check again after some time has passed
    ros::Time now = ros::Time::now();
    if ((now - lastRequestingServiceCheck).toSec() > 0.5) {
      // reset timer
      lastRequestingServiceCheck = ros::Time::now();

      // roll a die
      uniform_real_distribution<double> Distribution(0, 1);
      double roll = Distribution(RNG());
      if (roll < requestingServiceProbability) {
        return true;
      }
    }
  }

  return false;
}

bool Agent::startRequestingGuide() {
  // only do the probability check again after some time has passed
  ros::Time now = ros::Time::now();
  if ((now - lastRequestingGuideCheck).toSec() > 0.5) {
    // reset timer
    lastRequestingGuideCheck = ros::Time::now();

    // only enter the state if no other guide interaction is going on
    if (!SCENE.guideActive) {
      // roll a die
      uniform_real_distribution<double> Distribution(0, 1);
      double roll = Distribution(RNG());
      if (roll < requestingGuideProbability) {
        return true;
      }
    }
  }

  return false;
}

bool Agent::startRequestingFollower() {
  // only do the probability check again after some time has passed
  ros::Time now = ros::Time::now();
  if ((now - lastRequestingFollowerCheck).toSec() > 0.5) {
    // reset timer
    lastRequestingFollowerCheck = ros::Time::now();

    // only enter the state if no other follower interaction is going on
    if (!SCENE.followerActive && !hasRequestedFollower) {
      // roll a die
      uniform_real_distribution<double> Distribution(0, 1);
      double roll = Distribution(RNG());
      if (roll < requestingFollowerProbability) {
        return true;
      }
    }
  }

  return false;
}

bool Agent::switchRunningWalking(){
  // only do the probability check again after some time has passed
  ros::Time now = ros::Time::now();
  if ((now - lastSwitchRunningWalkingCheck).toSec() > 0.5) {
    // reset timer
    lastSwitchRunningWalkingCheck = ros::Time::now();

    // roll a dice
    uniform_real_distribution<double> Distribution(0, 1);
    double roll = Distribution(RNG());
    if (roll < switchRunningWalkingProbability) {
      // switch to running
      return true;
    }
  }

  return false;
}

bool Agent::finishedRotation() {
  return abs(fmod(facingDirection, 2 * M_PI) - angleTarget) < 0.1;
}

bool Agent::serviceRobotIsNear() {
  for (auto agent : getAgentsInRange(1.0)) {
    if (agent->getType() == Ped::Tagent::AgentType::SERVICEROBOT) {
      currentServiceRobot = agent;
      return true;
    }
  }
  return false;
}

bool Agent::guideRobotIsNear() {
  if (SCENE.robot != nullptr) {
    auto robotpos = SCENE.robot->getPosition();
    auto diff = robotpos - p;
    if (diff.length() < 4.0) {
      return true;
    }
  }
  return false;
}

bool Agent::someoneIsRequestingService() {
  for (auto agent : getAgentsInRange(maxServicingRadius)) {
    if (agent->getStateMachine()->getCurrentState() == AgentStateMachine::AgentState::StateRequestingService) {
      servicingAgent = agent;
      if (servicingWaypoint != nullptr) {
        // remove old waypoint
        SCENE.removeWaypoint(servicingWaypoint);
      }
      servicingWaypoint = new AreaWaypoint("service_destination", servicingAgent->getPosition(), 1.0);
      SCENE.addWaypoint(servicingWaypoint);
      getWaypointPlanner()->setDestination(servicingWaypoint);
      currentDestination = servicingWaypoint;
      return true;
    }
  }
  return false;
}

void Agent::recordVelocity() {
  // record velocities at a fixed time interval
  ros::Time now = ros::Time::now();
  if ((now - lastRecordedVelocityTime).toSec() > 1.5) {
    // reset timer
    lastRecordedVelocityTime = ros::Time::now();

    recordedVelocitiesIndex = (recordedVelocitiesIndex + 1) % numRecordedVelocities;
    recordedVelocities[recordedVelocitiesIndex] = v.lengthSquared();
    velocitiesRecorded++;
  }
}

bool Agent::isStuck() {
  ros::Time now = ros::Time::now();
  if ((now - lastIsStuckCheck).toSec() > 1.5) {
    // reset timer
    lastIsStuckCheck = ros::Time::now();

    if (velocitiesRecorded > numRecordedVelocities) {  // make sure array is filled
      // get mean velocity
      double v_sum = 0;
      for (int i = 0; i < numRecordedVelocities; i++) {
        v_sum += recordedVelocities[i];
      }
      double v_mean = v_sum / numRecordedVelocities;
      if (v_mean < 0.3) {
        return true;
      }
    }
  }
  return false;
}

void Agent::updateSubGoal() {
  if (subGoal != nullptr) {
    SCENE.removeWaypoint(subGoal);
    subGoal = nullptr;
  }

  if (arenaGoal == nullptr) {
    ROS_WARN("Can't update subgoal. Agent::arenaGoal is nullptr.");
    return;
  }

  // create new subgoal, some distance from the arena goal away
  uniform_real_distribution<double> distribution_angle(0, 2 * M_PI);
  uniform_real_distribution<double> distribution_distance(3, 5);
  double random_angle;
  double random_distance;
  Ped::Tvector random_point;
  do {
    random_angle = distribution_angle(RNG());
    random_distance = distribution_distance(RNG());
    random_point = arenaGoal->getPosition() + Ped::Tvector::fromPolar(Ped::Tangle::fromRadian(random_angle), random_distance);
  } while (SCENE.isOccupied(random_point));
  
  subGoal = new AreaWaypoint("subgoal", random_point, 0.3);
  SCENE.addWaypoint(subGoal);
}

void Agent::updateArenaGoal() {
  if (arenaGoal != nullptr) {
    SCENE.removeWaypoint(arenaGoal);
    arenaGoal = nullptr;
  }

  if (SCENE.arenaGoal == nullptr) {
    ROS_WARN("Can't update Agent::arenaGoal. Scene::arenaGoal is nullptr.");
    return;
  }

  arenaGoal = new AreaWaypoint("arenagoal", *SCENE.arenaGoal, 0.3);
  SCENE.addWaypoint(arenaGoal);
}

void Agent::disableForce(const QString& forceNameIn) {
  // disable force by adding it to the list of disabled forces
  disabledForces.append(forceNameIn);
}

void Agent::enableForce(const QString& forceNameIn) {
  int idx = disabledForces.indexOf(forceNameIn);
  if (idx >= 0) {
    disabledForces.removeAt(idx);
  }
}

void Agent::enableAllForces() {
  // remove all forces from disabled list
  disabledForces.clear();
}

void Agent::disableAllForces() {
  disableForce("Obstacle");
  disableForce("Desired");
  disableForce("Social");
  disableForce("KeepDistance");
  disableForce("Robot");
}

void Agent::resumeMovement() {
  enableAllForces();
  disableForce("KeepDistance");  // disable KeepDistance because it's not part of normal movement
}

void Agent::stopMovement() {
  disableAllForces();
  // set v and a to zero
  setv(Ped::Tvector());
  seta(Ped::Tvector());
}

void Agent::adjustKeepDistanceForceDistance() {
  // This method is called while in StateListening
  // Adjust listening distance based on how many agents are listening together with this one

  // get number of agents that are listening to the same id as I do
  auto agents = SCENE.getAgents();
  int count = 0;
  int check_for_id = -1;

  if (stateMachine->getCurrentState() == AgentStateMachine::AgentState::StateGroupTalking) {
    check_for_id = id;
  } else {
    check_for_id = listeningToId;
  }

  for (auto agent : agents) {
    if (agent->listeningToId == check_for_id) {
      count++;
    }
  }

  double distance_between_listening_agents = 1.5;
  double min_keep_distance_force_distance = 0.3;
  keepDistanceForceDistance = count * distance_between_listening_agents / (2 * M_PI);
  
  if (keepDistanceForceDistance < min_keep_distance_force_distance) {
    keepDistanceForceDistance = min_keep_distance_force_distance;
  }
}

void Agent::setPosition(double xIn, double yIn) {
  // call super class' method
  Ped::Tagent::setPosition(xIn, yIn);

  // inform users
  emit positionChanged(xIn, yIn);
}

void Agent::setX(double xIn) { setPosition(xIn, gety()); }

void Agent::setY(double yIn) { setPosition(getx(), yIn); }

void Agent::setType(Ped::Tagent::AgentType typeIn) {
  // call super class' method
  Ped::Tagent::setType(typeIn);

  // inform users
  emit typeChanged(typeIn);
}

Ped::Tvector Agent::getDesiredDirection() const { return desiredforce; }

Ped::Tvector Agent::getWalkingDirection() const { return v; }

Ped::Tvector Agent::getSocialForce() const { return socialforce; }

Ped::Tvector Agent::getObstacleForce() const { return obstacleforce; }

Ped::Tvector Agent::getMyForce() const { return myforce; }

Ped::Tvector Agent::getKeepDistanceForce() const { return keepdistanceforce; }

Ped::Tvector Agent::getRobotForce() const { return robotforce; }

QPointF Agent::getVisiblePosition() const { return QPointF(getx(), gety()); }

void Agent::setVisiblePosition(const QPointF& positionIn) {
  // check and apply new position
  if (positionIn != getVisiblePosition())
    setPosition(positionIn.x(), positionIn.y());
}

QString Agent::toString() const {
  return tr("Agent %1 (@%2,%3)").arg(getId()).arg(getx()).arg(gety());
}

void Agent::varySpeed() {
  ros::Time now = ros::Time::now();
  if ((now - lastVarySpeed).toSec() > 0.5) {
    // reset timer and don't vary again for 5 seconds
    lastVarySpeed = ros::Time::now() + ros::Duration(5.0);

    uniform_real_distribution<double> distribution_activate(0, 1);
    double roll = distribution_activate(RNG());
    if (roll < 0.5) {  // 50% chance to change velocity
      if (vmax < vmaxDefault + 0.01) {
        // increase speed
        setVmax(vmaxDefault * 2.0);
      } else {
        // reset speed
        setVmax(vmaxDefault);
      }
    }
  }
}

bool Agent::waitTimeExpired() {
  ros::Time now = ros::Time::now();
  if ((now - waitTimer).toSec() > waitTime) {
    return true;
  }
  return false;
}

bool Agent::robotInTriggerZone() {
  if (SCENE.robot != nullptr) {
    auto diff = p - SCENE.robot->getPosition();
    if (diff.length() < triggerZoneRadius) {
      return true;
    }
  }
  return false;
}
