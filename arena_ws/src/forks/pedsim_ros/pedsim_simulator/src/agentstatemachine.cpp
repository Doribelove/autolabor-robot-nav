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
#include <pedsim_simulator/element/agentgroup.h>
#include <pedsim_simulator/element/attractionarea.h>
#include <pedsim_simulator/element/waitingqueue.h>
#include <pedsim_simulator/rng.h>
#include <pedsim_simulator/scene.h>
#include <pedsim_simulator/waypointplanner/groupwaypointplanner.h>
#include <pedsim_simulator/waypointplanner/individualwaypointplanner.h>
#include <pedsim_simulator/waypointplanner/queueingplanner.h>
#include <pedsim_simulator/waypointplanner/shoppingplanner.h>

#include <ros/ros.h>

AgentStateMachine::AgentStateMachine(Agent* agentIn) {
  // initialize values
  agent = agentIn;
  individualPlanner = nullptr;
  queueingPlanner = nullptr;
  groupWaypointPlanner = nullptr;
  shoppingPlanner = nullptr;
  groupAttraction = nullptr;
  shallLoseAttraction = false;
  currentEpisode = 0;
  // initialize state machine
  state = StateNone;
}

AgentStateMachine::~AgentStateMachine() {
  // clean up
  delete individualPlanner;
  delete queueingPlanner;
  delete groupWaypointPlanner;
  delete shoppingPlanner;
}

void AgentStateMachine::loseAttraction() {
  // set mark to lose attraction
  shallLoseAttraction = true;
}

void AgentStateMachine::doStateTransition() {
  // determine new state
  if (agent->getType() == Ped::Tagent::AgentType::VEHICLE)
  {
    // ## Forklift behavior ##

    // → operate on waypoints/destinations
    if (state == StateNone) {
      Ped::Twaypoint* destination = agent->updateDestination();
      if (destination == nullptr) {
        activateState(StateWaiting);
        return;
      } else {
        activateState(StateDriving);
        return;
      }
    }

    if (state == StateDriving && agent->isStuck()) {
      agent->updateDestination();
      // don't check again for some time
      agent->lastIsStuckCheck = agent->lastIsStuckCheck + ros::Duration(10.0);
      activateState(StateDriving);
      return;
    }

    if (state == StateWaitForTimer) {
      if (agent->waitTimeExpired()) {
        activateState(StateDriving);
        return;
      }
      return;
    }

    if (state == StateWaitForTrigger) {
      if (agent->robotInTriggerZone()) {
        activateState(StateDriving);
        return;
      }
      return;
    }

    // → update destination on arrival
    if (agent->hasCompletedDestination()) {
      if (agent->getCurrentDestination()->isInteractive()) {
        agent->updateDestination();
        activateState(StateReachedShelf);
      } else if (state == StateWaitForTrigger) {
        // don't do anything
      } else if (state == StateWaitForTimer) {
        // don't do anything
      } else {
        agent->updateDestination();
        activateState(StateDriving);
      }
      return;
    }


    // → check if interactive obstacle is in range
    if (state == StateDriving && agent->isInteracting == false)
    {
      auto destination = agent->getInteractiveObstacleInRange(Ped::Twaypoint::WaypointType::Shelf);
      if (destination != nullptr && agent->lastInteractedWithWaypointId != destination->getId())
      {
        agent->lastInteractedWithWaypointId = destination->getId();
        agent->lastInteractedWithWaypoint = destination;
        agent->isInteracting = true;
        agent->getWaypointPlanner()->setDestination(destination);
        agent->currentDestination = destination;
      }
    }


    if (state == StateReachedShelf)
    {
      if (agent->completedMoveList()) {
        activateState(StateLiftingForks);
      }
      return;
    }


    // → lift forks
    if (state == StateLiftingForks)
    {
      ros::WallDuration timePassed = ros::WallTime::now() - startTimestamp;
      if (timePassed.toSec() > stateMaxDuration)
      {
        activateState(StateLoading);
      }
      return;
    }


    // → load stuff
    if (state == StateLoading)
    {
      ros::WallDuration timePassed = ros::WallTime::now() - startTimestamp;
      if (timePassed.toSec() > stateMaxDuration)
      {
        activateState(StateLoweringForks);
      }
      return;
    }


    // → lower forks
    if (state == StateLoweringForks)
    {
      ros::WallDuration timePassed = ros::WallTime::now() - startTimestamp;
      if (timePassed.toSec() > stateMaxDuration)
      {
        activateState(StateBackUp);
      }
      return;
    }

    // drive backwards and turn to next destination
    if (state == StateBackUp)
    {
      if (agent->completedMoveList()) {
        activateState(StateDriving);
      }
      return;
    }

  }
  else if (agent->getType() == Ped::Tagent::AgentType::SERVICEROBOT)
  {
    // ## Service Robot behavior ##

    if (state == StateNone) {
      Ped::Twaypoint* destination = agent->updateDestination();
      if (destination == nullptr) {
        activateState(StateWaiting);
        return;
      } else {
        activateState(StateDriving);
        return;
      }
    }

    if (state == StateWaitForTimer) {
      if (agent->waitTimeExpired()) {
        activateState(StateDriving);
        return;
      }
      return;
    }

    if (state == StateWaitForTrigger) {
      if (agent->robotInTriggerZone()) {
        activateState(StateDriving);
        return;
      }
      return;
    }

    if (state == StateDriving) {
      // update destination on arrival
      if (agent->hasCompletedDestination()) {
        agent->updateDestination();
        activateState(StateDriving);
        return;
      }

      // check if someone is requesting service
      if (agent->someoneIsRequestingService()) {
        activateState(StateDrivingToInteraction);
        return;
      }
    }

    if (state == StateDrivingToInteraction) {
      // update position of destination
      agent->servicingWaypoint->setPosition(agent->servicingAgent->getPosition());
      // check if agent reached other agent calling for service (which is reflected by the state)
      if (agent->servicingAgent->getStateMachine()->getCurrentState() == StateReceivingService) {
        activateState(StateProvidingService);
        return;
      }
      // stop if other agent is not waiting anymore
      if (agent->servicingAgent->getStateMachine()->getCurrentState() != StateRequestingService) {
        activateState(StateDriving);
        return;
      }
      return;
    }

    if (state == StateProvidingService) {
      // provide service for as long as the receiving agent is in StateReceivingService
      if (agent->servicingAgent->getStateMachine()->getCurrentState() != StateReceivingService) {
        activateState(StateDriving);
        return;
      }
      return;
    }

  }
  else
  {
    // ## normal pedestrian behavior ##

    // → randomly get attracted by attractions
    // if ((state != StateShopping) && (state != StateQueueing)) {
    //   double distance = INFINITY;
    //   AttractionArea* attraction = nullptr;
    //   bool hasGroupAttraction = checkGroupForAttractions(&attraction);
    //   if (hasGroupAttraction) {
    //     // inherit groups' attraction
    //     groupAttraction = attraction;

    //     normalState = state;
    //     activateState(StateShopping);
    //     return;
    //   } else {
    //     // TODO: attraction must be visible!
    //     attraction = SCENE.getClosestAttraction(agent->getPosition(), &distance);

    //     if (attraction != nullptr) {
    //       // check whether agent is attracted
    //       // NOTE: The Cumulative Geometric Distribution determines the
    //       //       number of Bernoulli trials needed to get one success.
    //       //       → CDF(X) = 1-(1-p)^k   with k = the number of trials
    //       double baseProbability = 0.02;
    //       double maxAttractionDist = 7;
    //       // → probability dependents on strength, distance,
    //       //   and whether another group member are attracted
    //       double probability = baseProbability * attraction->getStrength() *
    //                             ((distance < maxAttractionDist)
    //                                 ? (1 - (distance / maxAttractionDist))
    //                                 : 0) *
    //                             CONFIG.getTimeStepSize();
    //       std::bernoulli_distribution isAttracted(probability);

    //       if (isAttracted(RNG())) {
    //         normalState = state;
    //         activateState(StateShopping);
    //         return;
    //       }
    //     }
    //   }
    // }


    // // → randomly lose attraction
    // if (state == StateShopping) {
    //   // check whether agent loses attraction
    //   // TODO: make this dependent from the distance to CoM
    //   double probability = 0.03;
    //   std::bernoulli_distribution isAttracted(probability *
    //                                           CONFIG.getTimeStepSize());

    //   if (shallLoseAttraction || isAttracted(RNG())) {
    //     // reactivate previous state
    //     activateState(normalState);

    //     // alreade picked a new state, so nothing to do
    //     return;
    //   }
    // }


    // → operate on waypoints/destinations
    if (state == StateNone) {
      Ped::Twaypoint* destination = agent->updateDestination();
      if (destination == nullptr) {
        activateState(StateWaiting);
        return;
      } else {
        activateState(StateWalking);
        return;
      }
    }

    if (state == StateWaitForTimer) {
      if (agent->waitTimeExpired()) {
        activateState(StateWalking);
        return;
      }
      return;
    }

    if (state == StateWaitForTrigger) {
      if (agent->robotInTriggerZone()) {
        activateState(StateWalking);
        return;
      }
      return;
    }

    // → update destination on arrival
    if (agent->hasCompletedDestination()) {
      if (state == StateGuideToGoal) {
        AreaWaypoint* wp = dynamic_cast<AreaWaypoint*>(agent->currentDestination);

        if (wp == agent->subGoal) {
          // agent has reached subgoal
          agent->currentDestination = agent->arenaGoal;
          if (individualPlanner == nullptr) {
            individualPlanner = new IndividualWaypointPlanner();
          }
          individualPlanner->setAgent(agent);
          individualPlanner->setDestination(agent->currentDestination);
          agent->setWaypointPlanner(individualPlanner);
          return;
        } else {
          // agent has reached arenagoal
          activateState(StateClearingGoal);
          return;
        }
      } else if (state == StateWaitForTrigger) {
        // don't do anything
      } else if (state == StateWaitForTimer) {
        // don't do anything
      } else {
        agent->updateDestination();
        activateState(StateWalking);
        return;
      }
    }

    if ((state == StateWalking || state == StateRunning) && agent->isStuck()) {
      agent->updateDestination();
      // don't check again for some time
      agent->lastIsStuckCheck = agent->lastIsStuckCheck + ros::Duration(10.0);
      activateState(StateWalking);
      return;
    }

    // ## do some checks wether to interrupt walking

    // → switch to running sometimes
    if (state == StateWalking && agent->switchRunningWalking()) {
      activateState(StateRunning);
      return;
    }

    // → switch to running sometimes
    if (state == StateRunning && agent->switchRunningWalking()) {
      activateState(StateWalking);
      return;
    }

    // → start telling a story sometimes
    if (state == StateWalking && agent->tellStory()) {
      activateState(StateTellStory);
      return;
    }


    // → start group talking sometimes
    if (state == StateWalking && agent->startGroupTalking()) {
      activateState(StateGroupTalking);
      return;
    }


    // → check wether someone is talking to me
    if ((state == StateWalking || state == StateRunning) && agent->someoneTalkingToMe()) {
      if (agent->listeningToAgent->getStateMachine()->getCurrentState() == AgentStateMachine::AgentState::StateTalkingAndWalking) {
        activateState(StateListeningAndWalking);
      } else {
        activateState(StateListening);
      }
      return;
    }


    // → start talking to someone sometimes
    if ((state == StateWalking) && agent->startTalking()) {
      activateState(StateTalking);
      return;
    }


    // → start talking to someone while walking sometimes
    if ((state == StateWalking) && agent->startTalkingAndWalking()) {
      activateState(StateTalkingAndWalking);
      return;
    }

    // → request service sometimes
    if ((state == StateWalking) && agent->startRequestingService()) {
      activateState(StateRequestingService);
      return;
    }

    // → requesting service for some time or until a service robot is near
    if (state == StateRequestingService) {
      ros::WallDuration timePassed = ros::WallTime::now() - startTimestamp;
      if (timePassed.toSec() > stateMaxDuration) {
        activateState(StateWalking);
        return;
      }
      if (agent->serviceRobotIsNear()) {
        activateState(StateReceivingService);
        return;
      }
      return;
    }
    
    // → receiving service for some time
    if (state == StateReceivingService) {
      ros::WallDuration timePassed = ros::WallTime::now() - startTimestamp;
      if (timePassed.toSec() > stateMaxDuration) {
        activateState(StateWalking);
        return;
      }
    }

    // → request guide sometimes
    if ((state == StateWalking) && agent->startRequestingGuide()) {
      activateState(StateRequestingGuide);
      return;
    }

    if (state == StateRequestingGuide) {
      if (agent->guideRobotIsNear()) {
        activateState(StateFollowingGuide);
        return;
      }
      return;
    }

    if (state == StateFollowingGuide) {
      if (SCENE.episode != currentEpisode) {
        // end state after episode has ended
        activateState(StateWalking);
        return;
      }
      agent->varySpeed();
      agent->followWaypoint->setPosition(SCENE.robot->getPosition());
      agent->keepDistanceTo = SCENE.robot->getPosition();
      return;
    }

    // → talk and walk for some time
    if (state == StateTalkingAndWalking) {
      ros::WallDuration timePassed = ros::WallTime::now() - startTimestamp;
      if (timePassed.toSec() > stateMaxDuration)
      {
        activateState(StateWalking);
      }
      return;
    }


    // → tell story for some time
    if (state == StateTellStory) {
      ros::WallDuration timePassed = ros::WallTime::now() - startTimestamp;
      if (timePassed.toSec() > stateMaxDuration)
      {
        activateState(StateWalking);
      }
      return;
    }


    // → talk as a group for some time
    if (state == StateGroupTalking) {
      ros::WallDuration timePassed = ros::WallTime::now() - startTimestamp;
      if (timePassed.toSec() > stateMaxDuration)
      {
        activateState(StateWalking);
      }
      agent->adjustKeepDistanceForceDistance();
      return;
    }


    // → listening while walking
    if (state == StateListeningAndWalking) {
      // check if I am still being talked to
      if (agent->listeningToAgent != nullptr) {
        if (
          agent->listeningToAgent->getStateMachine()->getCurrentState() == AgentStateMachine::AgentState::StateTalkingAndWalking
        ) {
          return;
        }
      }

      activateState(StateWalking);
      return;
    }

    // → listening
    if (state == StateListening) {
      // check if I am still being talked to
      if (agent->listeningToAgent != nullptr) {
        if (
          agent->listeningToAgent->getStateMachine()->getCurrentState() == AgentStateMachine::AgentState::StateTellStory ||
          agent->listeningToAgent->getStateMachine()->getCurrentState() == AgentStateMachine::AgentState::StateGroupTalking ||
          agent->listeningToAgent->talkingToId == agent->getId()
        ) {
          agent->adjustKeepDistanceForceDistance();
          return;
        }
      }

      activateState(StateWalking);
      return;
    }


    // → talk for some time
    if (state == StateTalking) {
      ros::WallDuration diff = ros::WallTime::now() - startTimestamp;
      if (diff.toSec() > 6.20) {
        activateState(StateWalking);
        return;
      }
    }

    // → request follower sometimes
    if ((state == StateWalking) && agent->startRequestingFollower()) {
      activateState(StateRequestingFollower);
      return;
    }

    if (state == StateRequestingFollower) {
      if (agent->guideRobotIsNear()) {
        activateState(StateGuideToGoal);
        return;
      }
      return;
    }

    if (state == StateGuideToGoal) {
      return;
    }

    if (state == StateClearingGoal) {
      return;
    }
  }
}


void AgentStateMachine::activateState(AgentState stateIn) {
  // if (agent->id == 1) ROS_INFO("Agent %d type %d activating state '%s' (time: %f)", agent->getId(), agent->getType(), stateToName(stateIn).toStdString().c_str(), SCENE.getTime());

  // de-activate old state
  deactivateState(state);

  // re-activate all forces
  agent->enableAllForces();

  // set state
  state = stateIn;

  Waypoint* destination =
      dynamic_cast<Waypoint*>(agent->getCurrentDestination());

  switch (state) {
    case StateNone:
      agent->setWaypointPlanner(nullptr);
      agent->disableAllForces();
      break;
    case StateWaiting:
      agent->setWaypointPlanner(nullptr);
      agent->disableAllForces();
      break;
    case StateWaitForTrigger:
      agent->setVmax(0);
      break;
    case StateWaitForTimer:
      agent->setVmax(0);
      break;
    case StateWalking:
      if (individualPlanner == nullptr)
        individualPlanner = new IndividualWaypointPlanner();
      individualPlanner->setAgent(agent);
      individualPlanner->setDestination(destination);
      agent->setWaypointPlanner(individualPlanner);
      agent->resumeMovement();
      agent->setVmax(agent->vmaxDefault);
      agent->disableForce("KeepDistance");
      agent->disableForce("Robot");
      break;
    case StateDriving:
      if (individualPlanner == nullptr)
        individualPlanner = new IndividualWaypointPlanner();
      individualPlanner->setAgent(agent);
      individualPlanner->setDestination(destination);
      agent->setWaypointPlanner(individualPlanner);
      agent->resumeMovement();
      agent->setVmax(agent->vmaxDefault);
      agent->disableForce("KeepDistance");
      agent->disableForce("Robot");
      agent->isInteracting = false;
      break;
    case StateRunning:
      if (individualPlanner == nullptr)
        individualPlanner = new IndividualWaypointPlanner();
      individualPlanner->setAgent(agent);
      individualPlanner->setDestination(destination);
      agent->setWaypointPlanner(individualPlanner);
      agent->resumeMovement();
      agent->setVmax(agent->vmaxDefault * 2.0);
      agent->disableForce("KeepDistance");
      agent->disableForce("Robot");
      break;
    case StateQueueing:
      if (queueingPlanner == nullptr)
        queueingPlanner = new QueueingWaypointPlanner();
      queueingPlanner->setAgent(agent);
      queueingPlanner->setDestination(destination);
      agent->setWaypointPlanner(queueingPlanner);
      break;
    case StateGroupWalking:
      if (groupWaypointPlanner == nullptr)
        groupWaypointPlanner = new GroupWaypointPlanner();
      groupWaypointPlanner->setDestination(destination);
      groupWaypointPlanner->setGroup(agent->getGroup());
      agent->setWaypointPlanner(groupWaypointPlanner);
      break;
    case StateShopping:
      {
        shallLoseAttraction = false;
      if (shoppingPlanner == nullptr) shoppingPlanner = new ShoppingPlanner();
      AttractionArea* attraction =
          SCENE.getClosestAttraction(agent->getPosition());
      shoppingPlanner->setAgent(agent);
      shoppingPlanner->setAttraction(attraction);
      agent->setWaypointPlanner(shoppingPlanner);
      agent->disableForce("GroupCoherence");
      agent->disableForce("GroupGaze");

      // keep other agents informed about the attraction
      AgentGroup* group = agent->getGroup();
      if (group != nullptr) {
        for (Agent* member : group->getMembers()) {
          if (member == agent) continue;

          AgentStateMachine* memberStateMachine = member->getStateMachine();
          connect(shoppingPlanner, SIGNAL(lostAttraction()), memberStateMachine,
                  SLOT(loseAttraction()));
        }
      }
      }
      break;
    case StateTalking:
      startTimestamp = ros::WallTime::now();
      stateMaxDuration = getRandomDuration(agent->stateTalkingBaseTime);
      agent->setWaypointPlanner(nullptr);
      agent->stopMovement();
      break;
    case StateTalkingAndWalking:
      startTimestamp = ros::WallTime::now();
      stateMaxDuration = getRandomDuration(agent->stateTalkingAndWalkingBaseTime);
      if (individualPlanner == nullptr){
        individualPlanner = new IndividualWaypointPlanner();
      }
      individualPlanner->setAgent(agent);
      individualPlanner->setDestination(destination);
      agent->setWaypointPlanner(individualPlanner);
      agent->resumeMovement();
      agent->setVmax(agent->vmaxDefault * 0.3);  // walk slower when talking
      agent->disableForce("Social");
      agent->disableForce("KeepDistance");
      agent->disableForce("Robot");
      break;
    case StateListeningAndWalking:
      agent->setWaypointPlanner(nullptr);
      agent->disableAllForces();
      break;
    case StateWorking:
      startTimestamp = ros::WallTime::now();
      stateMaxDuration = getRandomDuration(agent->stateWorkingBaseTime);
      agent->setWaypointPlanner(nullptr);
      agent->stopMovement();
      break;
    case StateLiftingForks:
      startTimestamp = ros::WallTime::now();
      stateMaxDuration = getRandomDuration(agent->stateLiftingForksBaseTime);
      agent->setWaypointPlanner(nullptr);
      agent->stopMovement();
      break;
    case StateLoading:
      startTimestamp = ros::WallTime::now();
      stateMaxDuration = getRandomDuration(agent->stateLoadingBaseTime);
      agent->setWaypointPlanner(nullptr);
      agent->stopMovement();
      break;
    case StateLoweringForks:
      startTimestamp = ros::WallTime::now();
      stateMaxDuration = getRandomDuration(agent->stateLoweringForksBaseTime);
      agent->setWaypointPlanner(nullptr);
      agent->stopMovement();
      break;
    case StateTellStory:
      startTimestamp = ros::WallTime::now();
      stateMaxDuration = getRandomDuration(agent->stateTellStoryBaseTime);
      agent->setWaypointPlanner(nullptr);
      agent->stopMovement();
      break;
    case StateGroupTalking:
      startTimestamp = ros::WallTime::now();
      stateMaxDuration = getRandomDuration(agent->stateGroupTalkingBaseTime);
      agent->setWaypointPlanner(nullptr);
      agent->enableForce("KeepDistance");
      agent->disableForce("Robot");
      agent->setForceFactorSocial(15.0);
      break;
    case StateListening:
      agent->setWaypointPlanner(nullptr);
      if (agent->isListeningToIndividual()) {
        agent->stopMovement();
      } else {
        agent->enableForce("KeepDistance");
        agent->disableForce("Robot");
        agent->setForceFactorSocial(15.0);
      }
      break;
    case StateReachedShelf:
      assert(agent->lastInteractedWithWaypoint != nullptr);
      agent->angleTarget = agent->lastInteractedWithWaypoint->staticObstacleAngle;
      agent->moveList = agent->createMoveList(StateReachedShelf);
      agent->setWaypointPlanner(nullptr);
      agent->disableAllForces();
      break;
    case StateBackUp:
      agent->moveList = agent->createMoveList(StateBackUp);
      agent->setWaypointPlanner(nullptr);
      agent->disableAllForces();
      break;
    case StateRequestingService:
      startTimestamp = ros::WallTime::now();
      stateMaxDuration = getRandomDuration(agent->stateRequestingServiceBaseTime);
      if (individualPlanner == nullptr) {
        individualPlanner = new IndividualWaypointPlanner();
      }
      individualPlanner->setAgent(agent);
      individualPlanner->setDestination(destination);
      agent->setWaypointPlanner(individualPlanner);
      agent->resumeMovement();
      agent->setVmax(agent->vmaxDefault * 0.2);
      agent->disableForce("KeepDistance");
      agent->disableForce("Robot");
      break;
    case StateReceivingService:
      startTimestamp = ros::WallTime::now();
      stateMaxDuration = getRandomDuration(agent->stateReceivingServiceBaseTime);
      // don't stop moving completely so the pedsimMovement animation can update
      agent->setVmax(agent->vmaxDefault * 0.01);
      agent->disableForce("KeepDistance");
      agent->disableForce("Robot");
      break;
    case StateDrivingToInteraction:
      if (individualPlanner == nullptr) {
        individualPlanner = new IndividualWaypointPlanner();
      }
      individualPlanner->setAgent(agent);
      individualPlanner->setDestination(destination);
      agent->setWaypointPlanner(individualPlanner);
      agent->resumeMovement();
      agent->setVmax(agent->vmaxDefault * 1.5);
      agent->disableForce("KeepDistance");
      agent->disableForce("Robot");
      break;
    case StateProvidingService:
      agent->setWaypointPlanner(nullptr);
      SCENE.removeWaypoint(agent->servicingWaypoint);
      agent->servicingWaypoint = nullptr;
      agent->currentDestination = nullptr;
      agent->stopMovement();
      break;
    case StateRequestingGuide:
      SCENE.guideActive = true;
      agent->setWaypointPlanner(nullptr);
      // don't stop moving completely so the pedsimMovement animation can update
      agent->setVmax(agent->vmaxDefault * 0.01);
      currentEpisode = SCENE.episode;
      agent->disableForce("KeepDistance");
      agent->disableForce("Robot");
      break;
    case StateFollowingGuide:
      if (individualPlanner == nullptr) {
        individualPlanner = new IndividualWaypointPlanner();
      }
      individualPlanner->setAgent(agent);
      // remove old waypoint
      if (agent->followWaypoint != nullptr) {
        SCENE.removeWaypoint(agent->followWaypoint);
      }
      agent->followWaypoint = new AreaWaypoint("follow_destination", SCENE.robot->getPosition(), 0.0);  // make radius zero so destination will never be reached
      SCENE.addWaypoint(agent->followWaypoint);
      agent->currentDestination = agent->followWaypoint;
      individualPlanner->setDestination(agent->followWaypoint);
      agent->setWaypointPlanner(individualPlanner);
      agent->resumeMovement();
      agent->enableForce("KeepDistance");
      agent->keepDistanceTo = SCENE.robot->getPosition();
      agent->keepDistanceForceDistance = 3.1;
      agent->setVmax(agent->vmaxDefault * 2.0);
      agent->disableForce("Robot");
      break;
    case StateRequestingFollower:
      SCENE.followerActive = true;
      agent->setWaypointPlanner(nullptr);
      // don't stop moving completely so the pedsimMovement animation can update
      agent->setVmax(agent->vmaxDefault * 0.01);
      agent->disableForce("KeepDistance");
      agent->disableForce("Robot");
      agent->hasRequestedFollower = true;
      break;
    case StateGuideToGoal:      
      if (SCENE.arenaGoal == nullptr) {
        ROS_WARN("Can't guide to goal. Scene::arenaGoal is nullptr. Is /goal published?");
        activateState(StateWalking);
        return;
      }
      agent->updateArenaGoal();
      agent->updateSubGoal();
      if (individualPlanner == nullptr)
        individualPlanner = new IndividualWaypointPlanner();
      individualPlanner->setAgent(agent);
      assert(agent->subGoal != nullptr);
      individualPlanner->setDestination(agent->subGoal);
      agent->currentDestination = agent->subGoal;
      agent->setWaypointPlanner(individualPlanner);
      agent->resumeMovement();
      agent->setVmax(agent->vmaxDefault * 2.0);
      agent->disableForce("KeepDistance");
      agent->disableForce("Robot");
      break;
    case StateClearingGoal:
      if (individualPlanner == nullptr)
        individualPlanner = new IndividualWaypointPlanner();
      individualPlanner->setAgent(agent);
      individualPlanner->setDestination(destination);
      agent->setWaypointPlanner(individualPlanner);
      agent->resumeMovement();
      agent->setVmax(agent->vmaxDefault * 2.0);
      agent->disableForce("KeepDistance");
      break;
    default:
      break;
  }

  // inform users
  emit stateChanged(state);
}

void AgentStateMachine::deactivateState(AgentState state) {
  switch (state) {
    case StateTalking:
      // reset talking to id
      agent->talkingToId = -1;
      break;
    case StateTalkingAndWalking:
      // reset talking to id
      agent->talkingToId = -1;
      break;
    case StateListening:
      // reset listening to id
      agent->listeningToId = -1;
      agent->listeningToAgent = nullptr;
      agent->setForceFactorSocial(2.0);
      break;
    case StateListeningAndWalking:
      // reset listening to id
      agent->listeningToId = -1;
      agent->listeningToAgent = nullptr;
      break;
    case StateShopping:
    {
      // inform other group members
      shoppingPlanner->loseAttraction();

      // don't worry about other group members
      AgentGroup* group = agent->getGroup();
      if (group != nullptr) {
        for (Agent* member : group->getMembers()) {
          if (member == agent) continue;

          AgentStateMachine* memberStateMachine = member->getStateMachine();
          disconnect(shoppingPlanner, SIGNAL(lostAttraction()),
                     memberStateMachine, SLOT(loseAttraction()));
        }
      }

      break;
    }
    case StateRequestingGuide:
      agent->setVmax(agent->vmaxDefault);
      break;
    case StateFollowingGuide:
      SCENE.removeWaypoint(agent->followWaypoint);
      agent->followWaypoint = nullptr;
      agent->updateDestination();
      break;
    case StateBackUp:
        agent->isInteracting = false;
      break;
    case StateRequestingFollower:
      agent->setVmax(agent->vmaxDefault);
      break;
    case StateGuideToGoal:
      if (agent->subGoal != nullptr) {
        SCENE.removeWaypoint(agent->subGoal);
        agent->subGoal = nullptr;
      }
      if (agent->arenaGoal != nullptr) {
        SCENE.removeWaypoint(agent->arenaGoal);
        agent->arenaGoal = nullptr;
      }
      agent->updateDestination();
      break;
    case StateWaitForTrigger:
      agent->setVmax(agent->vmaxDefault);
      break;
    case StateWaitForTimer:
      agent->setVmax(agent->vmaxDefault);
      break;
    default:
      break;
  }
}

double AgentStateMachine::getRandomDuration(double baseTime)
{
  uniform_real_distribution<double> Distribution(0.5, 1.5);
  double durationFactor = Distribution(RNG());
  double duration = durationFactor * baseTime;
  return duration;
}

bool AgentStateMachine::checkGroupForAttractions(
    AttractionArea** attractionOut) const {
  AgentGroup* group = agent->getGroup();

  // check whether the agent is even in a group
  if (group == nullptr) {
    if (attractionOut != nullptr) *attractionOut = nullptr;
    return false;
  }

  // check all group members
  for (Agent* member : group->getMembers()) {
    // ignore agent himself
    if (member == agent) continue;

    // check whether the group member uses ShoppingPlanner
    WaypointPlanner* planner = member->getWaypointPlanner();
    ShoppingPlanner* typedPlanner = dynamic_cast<ShoppingPlanner*>(planner);
    if (typedPlanner != nullptr) {
      AttractionArea* attraction = typedPlanner->getAttraction();

      if (attraction != nullptr) {
        attractionOut = &attraction;
        return true;
      }
    }
  }

  // no group member is attracted to something
  if (attractionOut != nullptr) *attractionOut = nullptr;
  return false;
}

QString AgentStateMachine::stateToName(AgentState stateIn) {
  switch (stateIn) {
    case StateNone:
      return "None";
    case StateWaiting:
      return "Waiting";
    case StateWalking:
      return "Walking";
    case StateRunning:
      return "Running";
    case StateDriving:
      return "Driving";
    case StateQueueing:
      return "Queueing";
    case StateGroupWalking:
      return "GroupWalking";
    case StateTalking:
      return "Talking";
    case StateTalkingAndWalking:
      return "TalkingAndWalking";
    case StateListeningAndWalking:
      return "ListeningAndWalking";
    case StateShopping:
      return "Shopping";
    case StateWorking:
      return "Working";
    case StateLiftingForks:
      return "LiftingForks";
    case StateLoading:
      return "Loading";
    case StateLoweringForks:
      return "LoweringForks";
    case StateTellStory:
      return "TellStory";
    case StateGroupTalking:
      return "GroupTalking";
    case StateListening:
      return "Listening";
    case StateReachedShelf:
      return "ReachedShelf";
    case StateBackUp:
      return "BackUp";
    case StateRequestingService:
      return "StateRequestingService";
    case StateReceivingService:
      return "StateReceivingService";
    case StateDrivingToInteraction:
      return "StateDrivingToInteraction";
    case StateProvidingService:
      return "StateProvidingService";
    case StateRequestingGuide:
      return "StateRequestingGuide";
    case StateFollowingGuide:
      return "StateFollowingGuide";
    case StateRequestingFollower:
      return "StateRequestingFollower";
    case StateGuideToGoal:
      return "StateGuideToGoal";
    case StateClearingGoal:
      return "StateClearingGoal";
    case StateWaitForTrigger:
      return "StateWaitForTrigger";
    case StateWaitForTimer:
      return "StateWaitForTimer";
    default:
      return "UnknownState";
  }
}

AgentStateMachine::AgentState AgentStateMachine::getCurrentState() {
  return state;
}
