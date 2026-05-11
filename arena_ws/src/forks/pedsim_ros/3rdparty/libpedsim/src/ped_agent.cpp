//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 20012 by Christian Gloor
// Modified by Ronja Gueldenring
//

#include "ped_agent.h"
#include "ped_obstacle.h"
#include "ped_scene.h"
#include "ped_waypoint.h"
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <random>

using namespace std;

int Ped::Tagent::staticid = 1;
default_random_engine generator;

/// Default Constructor
Ped::Tagent::Tagent() {
  id = staticid++;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  v.x = 0;
  v.y = 0;
  v.z = 0;
  type = ADULT;
  scene = nullptr;
  teleop = false;

  // assign random maximal speed in m/s
  normal_distribution<double> distribution(0.6, 0.2);
  vmax = distribution(generator);
  vmaxDefault = vmax;
  forceFactorDesired = 1.0;
  forceFactorSocial = 2.1;
  forceFactorObstacle = 10.0;
  forceSigmaObstacle = 0.8;
  forceSigmaRobot = 0.3*vmax/0.4;

  agentRadius = 0.35;
  relaxationTime = 0.5;
  robotPosDiffScalingFactor = 5;
  obstacleForceRange = 2.0;

  keepDistanceForceDistanceDefault = 0.8;
  keepDistanceForceDistance = keepDistanceForceDistanceDefault;
  keepDistanceTo = Tvector(0.0, 0.0);

  obstacleforce = Ped::Tvector(0.0, 0.0);
  
  ROS_INFO("created agent with id: %d", id);
}

/// Destructor
Ped::Tagent::~Tagent() {}

/// Assigns a Tscene to the agent. Tagent uses this to iterate over all
/// obstacles and other agents in a scene.
/// The scene will invoke this function when Tscene::addAgent() is called.
/// \warning Bad things will happen if the agent is not assigned to a scene. But
/// usually, Tscene takes care of that.
/// \param   *s A valid Tscene initialized earlier.
void Ped::Tagent::assignScene(Ped::Tscene* sceneIn) { scene = sceneIn; }

void Ped::Tagent::removeAgentFromNeighbors(const Ped::Tagent* agentIn) {
  // search agent in neighbors, and remove him
  set<const Ped::Tagent*>::iterator foundNeighbor = neighbors.find(agentIn);
  if (foundNeighbor != neighbors.end()) neighbors.erase(foundNeighbor);
}

/// Sets the maximum velocity of an agent (vmax). Even if pushed by other
/// agents, it will not move faster than this.
/// \param   pvmax The maximum velocity. In scene units per timestep, multiplied
/// by the simulation's precision h.
void Ped::Tagent::setVmax(double pvmax) { vmax = pvmax; }

/// Defines how much the position difference between this agent
/// and a robot is scaled: the bigger the number is, the smaller
/// the position based force contribution will be.
/// \param   scalingFactor should be positive.
void Ped::Tagent::setRobotPosDiffScalingFactor(double scalingFactor) {
  if (scalingFactor > 0) {
    robotPosDiffScalingFactor = scalingFactor;
  }
}

/// Sets the agent's position. This, and other getters returning coordinates,
/// will eventually changed to returning a
/// Tvector.
/// \param   px Position x
/// \param   py Position y
/// \param   pz Position z
void Ped::Tagent::setPosition(double px, double py, double pz) {
  p.x = px;
  p.y = py;
  p.z = pz;
}

/// Sets the factor by which the desired force is multiplied. Values between 0
/// and about 10 do make sense.
/// \param   f The factor
void Ped::Tagent::setForceFactorDesired(double f) { forceFactorDesired = f; }

/// Sets the factor by which the social force is multiplied. Values between 0
/// and about 10 do make sense.
/// \param   f The factor
void Ped::Tagent::setForceFactorSocial(double f) { forceFactorSocial = f; }

/// Sets the factor by which the obstacle force is multiplied. Values between 0
/// and about 10 do make sense.
/// \param   f The factor
void Ped::Tagent::setForceFactorObstacle(double f) { forceFactorObstacle = f; }

double Ped::Tagent::keepDistanceForceFunction(double distance) {
  return -10 * (distance - keepDistanceForceDistance);
}

// force to keep a specific distance
Ped::Tvector Ped::Tagent::keepDistanceForce() {
  Tvector diff = p - keepDistanceTo;
  Tvector direction = diff.normalized();
  double magnitude = keepDistanceForceFunction(diff.length());
  Tvector force = direction * magnitude;
  return force;
}

/// Calculates the force between this agent and the next assigned waypoint.
/// If the waypoint has been reached, the next waypoint in the list will be
/// selected.
/// \return  Tvector: the calculated force
Ped::Tvector Ped::Tagent::desiredForce() {
  // get destination
  Twaypoint* waypoint = getCurrentWaypoint();

  // if there is no destination, don't move
  if (waypoint == nullptr) {
    desiredDirection = Ped::Tvector();
    Tvector antiMove = -v / relaxationTime;
    return antiMove;
  }

  // compute force
  Tvector force = waypoint->getForce(*this, &desiredDirection);

  return force;
}

/// Calculates the social force between this agent and all the other agents
/// belonging to the same scene.
/// It iterates over all agents inside the scene, has therefore the complexity
/// O(N^2). A better
/// agent storing structure in Tscene would fix this. But for small (less than
/// 10000 agents) scenarios, this is just
/// fine.
/// \return  Tvector: the calculated force
Ped::Tvector Ped::Tagent::socialForce() const {
  // define relative importance of position vs velocity vector
  // (set according to Moussaid-Helbing 2009)
  const double lambdaImportance = 2.0;

  // define speed interaction
  // (set according to Moussaid-Helbing 2009)
  const double gamma = 0.35;

  // define speed interaction
  // (set according to Moussaid-Helbing 2009)
  const double n = 2;

  // define angular interaction
  // (set according to Moussaid-Helbing 2009)
  const double n_prime = 3;

  Tvector force(0.0, 0.0);
  for (const Ped::Tagent* other : neighbors) {
    // don't compute social force to yourself
    if (other->id == id) continue;

    // compute difference between both agents' positions
    Tvector diff = other->p - p;

    // if both agents have the exact same position disable social force to avoid division by zero
    if (diff.lengthSquared() <= 0.001) {
      return Tvector(0.0, 0.0);
    }

    if(other->getType() == ROBOT) diff /= robotPosDiffScalingFactor;

    Tvector diffDirection = diff.normalized();
    int quadrant_before = diff.getQuadrant();
    // shorten the diff vector by the model radiuses
    diff -= diffDirection * agentRadius;
    diff -= diffDirection * other->agentRadius;
    int quadrant_after = diff.getQuadrant();
    if (quadrant_before != quadrant_after) {
      // vector has changed direction i.e. models are already overlapping
      // make diff length close to zero
      diff = diffDirection * 0.01;
    }

    // compute difference between both agents' velocity vectors
    // Note: the agent-other-order changed here
    Tvector velDiff = v - other->v;

    // compute interaction direction t_ij
    Tvector interactionVector = lambdaImportance * velDiff + diffDirection;
    double interactionLength = interactionVector.length();
    assert(interactionLength > 0.0);
    Tvector interactionDirection = interactionVector / interactionLength;


    // The robots influence is computed separetly in Ped::Tagent::robotForce()
    if(other->getType() == ROBOT){
      continue;
    }else{
      // compute angle theta (between interaction and position difference vector)
      Ped::Tangle theta = interactionDirection.angleTo(diffDirection);
      // compute model parameter B = gamma * ||D||
      double B = gamma * interactionLength;

      double thetaRad = theta.toRadian();
      double forceVelocityAmount =
          -exp(-diff.length() / B -
              (n_prime * B * thetaRad) * (n_prime * B * thetaRad));
      double forceAngleAmount =
          -theta.sign() *
          exp(-diff.length() / B - (n * B * thetaRad) * (n * B * thetaRad));

      Tvector forceVelocity = forceVelocityAmount * interactionDirection;
      Tvector forceAngle =
          forceAngleAmount * interactionDirection.leftNormalVector();
      force += forceVelocity + forceAngle;

    }

  }

  return force;
}

Ped::Tvector Ped::Tagent::robotForce(){
  Tvector force;
  return force;
}

// // Added by Ronja Gueldenring
// // Robot influences agents behaviour according the robot force
// Ped::Tvector Ped::Tagent::robotForce(){
//   double vel = sqrt(pow(this->getvx(),2) + pow(this->getvy(),2));
//   if (vel > 0.1){
//     still_time = 0.0;
//   }

//   Tvector force;
//   for (const Ped::Tagent* other : neighbors) {
//     if(other->getType() == ROBOT){
//       // pedestrian is influenced robot force depending on the distance to the robot.
//       Tvector diff = other->p - p;
//       Tvector diffDirection = diff.normalized();
//       double distanceSquared = diff.lengthSquared();
//       double distance = sqrt(distanceSquared) - (agentRadius + 0.7);
//       double forceAmount = -1.0 * exp(-distance / forceSigmaRobot);
//       Tvector robot_force = forceAmount * diff.normalized();
//       force += robot_force;
//       break;
//     }
//   }
//   return force;
// }

/// Calculates the force between this agent and the nearest obstacle in this
/// scene.
/// Iterates over all obstacles == O(N).
/// \return  Tvector: the calculated force
Ped::Tvector Ped::Tagent::obstacleForce() {
  // obstacle which is closest only
  Ped::Tvector minDiff;
  double minDistanceSquared = INFINITY;

  for (const Tobstacle* obstacle : scene->obstacles) {
    Ped::Tvector closestPoint = obstacle->closestPoint(p);
    Ped::Tvector diff = p - closestPoint;
    double distanceSquared = diff.lengthSquared();  // use squared distance to
    // avoid computing square
    // root
    if (distanceSquared < minDistanceSquared) {
      minDistanceSquared = distanceSquared;
      minDiff = diff;
    }
  }

  double distance = sqrt(minDistanceSquared) - agentRadius;
  // double forceAmount = exp(-distance / forceSigmaObstacle);
  double forceAmount = 10.0;
  if (distance > 0.0) {
    forceAmount = 1.0 / distance;
  }
  return forceAmount * minDiff.normalized();
}

/// myForce() is a method that returns an "empty" force (all components set to
/// 0).
/// This method can be overridden in order to define own forces.
/// It is called in move() in addition to the other default forces.
/// \return  Tvector: the calculated force
/// \param   e is a vector defining the direction in which the agent wants to
/// walk to.
Ped::Tvector Ped::Tagent::myForce(Ped::Tvector e) {
  return Ped::Tvector(0.0, 0.0);
}

void Ped::Tagent::computeForces() {
  // update neighbors 
  // NOTE - have a config value for the neighbor range
  const double neighborhoodRange = 10.0;
  neighbors = scene->getNeighbors(p.x, p.y, neighborhoodRange);  

  // update forces
  desiredforce = desiredForce();
  if (forceFactorSocial > 0) socialforce = socialForce();
  if (forceFactorObstacle > 0) obstacleforce = obstacleForce();
  robotforce = robotForce();
  keepdistanceforce = keepDistanceForce();
  myforce = myForce(desiredDirection);
}

/// Does the agent dynamics stuff. Calls the methods to calculate the individual
/// forces, adds them
/// to get the total force affecting the agent. This will then be translated
/// into a velocity difference,
/// which is applied to the agents velocity, and then to its position.
/// \param   stepSizeIn This tells the simulation how far the agent should
/// proceed
void Ped::Tagent::move(double stepSizeIn) {
  still_time += stepSizeIn;

  // sum of all forces --> acceleration
  a = forceFactorDesired * desiredforce +
      forceFactorSocial * socialforce +
      forceFactorObstacle * obstacleforce +
      myforce +
      keepdistanceforce +
      forceFactorRobot * robotforce;
    // if (id == 1) {
    //   ROS_INFO("desiredforce: %lf, %lf, %lf", desiredforce.x, desiredforce.y, desiredforce.z);
    //   ROS_INFO("socialforce: %lf, %lf, %lf", socialforce.x, socialforce.y, socialforce.z);
    //   ROS_INFO("obstacleforce: %lf, %lf, %lf", obstacleforce.x, obstacleforce.y, obstacleforce.z);
    //   ROS_INFO("myforce: %lf, %lf, %lf", myforce.x, myforce.y, myforce.z);
    //   ROS_INFO("keepdistanceforce: %lf, %lf, %lf", keepdistanceforce.x, keepdistanceforce.y, keepdistanceforce.z);
    // }
    // ROS_INFO("stepSizeln%lf",stepSizeIn);

  // Added by Ronja Gueldenring
  // add robot force, so that pedestrians avoid robot
  // if (this->getType() == ADULT_AVOID_ROBOT || this->getType() == ADULT_AVOID_ROBOT_REACTION_TIME){
      // a = a + forceFactorSocial * robotforce;
  // }
  
  // calculate the new velocity
  if (getTeleop() == false) {
    v = v + stepSizeIn * a;
    // ROS_WARN("update velocity %lf,%lf", v.x,v.y);
  }

  // don't exceed maximal speed
  double speed = v.length();
  if (speed > vmax) v = v.normalized() * vmax;

  // internal position update = actual move
  p += stepSizeIn * v;

  // notice scene of movement
  scene->moveAgent(this);
}
