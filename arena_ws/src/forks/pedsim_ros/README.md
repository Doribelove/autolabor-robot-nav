# Pedestrian Simulator
Pedestrian Simulator (Pedsim) is a 2D pedestrian simulator based on the social force model of [Helbing et. al](http://arxiv.org/pdf/cond-mat/9805244.pdf). This repository is an extension of the original [Pedsim repository](https://github.com/srl-freiburg/pedsim_ros) and also incorporating features from [Ronja Güldenring's fork](https://github.com/RGring/pedsim_ros/tree/rl_features). It's supposed to be used in conjunction with [Arena-Rosnav](https://github.com/ignc-research/arena-rosnav/tree/sim_to_real).

Movement of the agents in the simulation is controlled by these forces:
- desired force (pulls agent towards the current destination)
- social force (pushes agent away from other agents)
- obstacle force (pushes agent away from obstacles)
- robot force (pushes agent away from robot)

Additionally agents can perform different actions that interrupt or modify the movement like talking to each other, stand as a group or take a walk with another agent.
We also implemented the possibility to spawn a forklift agent that has the same basic movement functionality but has different actions e.g. loading/unloading objects.

For Pedsim to work together with Arena-Rosnav it needs to be synced to [Flatland Simulator](https://github.com/ignc-research/flatland/tree/sim_to_real). This is done via a flatland plugin that subscribes to the **simulated_agents** topic published by Pedsim and updates the position of the corresponding flatland models accordingly.

## Examples

| <img width="400" height="250" src="/img/takingawalk.gif"> | <img width="400" height="250" src="/img/grouping.gif"> | 
|:--:|:--:|
| *Human agents walking together* | *Human agents standing as a group* |

| <img width="400" height="250" src="/img/forklift.gif"> | 
|:--:|
| *Forklift agent doing some work* |


# Demo
#### 1. Normal Arena-Rosnav Installation
See [Installation Manual](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/docs/Installation.md)

#### 2. Switch to sim_to_real branch and pull forks
```
cd ~/catkin_ws/src/arena-rosnav
git checkout sim_to_real
rosws update --delete-changed-uris .
```

#### 3. Build and Source
```
cd ~/catkin_ws
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.zsh
```

#### 4. Start Demo
```
workon rosnav
roslaunch arena_bringup pedsim_demo.launch
```
This will start Flatland, Pedsim and a script that will spawn a few agents.


# How to set up Pedsim to work with your own Arena-Rosnav branch
#### 1. Add Pedsim to .rosinstall
Copy this to your .rosinstall file:
```
- git:
    local-name: ../forks/pedsim_ros
    uri: https://github.com/ignc-research/pedsim_ros
    version: master
```
Afterwards run rosws update
```
cd ~/catkin_ws/src/arena-rosnav
rosws update .
```
#### 2. Add Pedsim to a launch file
Add the following to the launch file you want to use:
```
<arg name="simulation_factor" default="1"/>
<arg name="update_rate" default="25.0"/>
<arg name="scene_file" default="$(find simulator_setup)/pedsim_scenarios/empty.xml"/>

<node name="pedsim_simulator" pkg="pedsim_simulator" type="pedsim_simulator" output="screen">
    <param name="simulation_factor" value="$(arg simulation_factor)" type="double"/>
    <param name="update_rate" value="$(arg update_rate)" type="double"/>
    <param name="scene_file" value="$(arg scene_file)" type="string"/>
</node>
```
Pedsim was originally meant to be started using a scenario.xml file. In our version you can spawn agents dynamically but you still need to provide a scenario.xml file. Best practice is to just use an empty scenario. For the above snippet to work you need a file called **empty.xml** in this folder:
```
~/catkin_ws/src/arena-rosnav/simulator_setup/pedsim_scenarios/
```
**empty.xml** should contains this:
```
<?xml version="1.0" encoding="UTF-8"?>
<scenario>
</scenario>
```
After launching, the pedsim_simulator node should be running and advertising its services.

# Notable Pedsim Services
### /pedsim_simulator/spawn_peds
Spawns a number of agents in Pedsim. Additionally it spawns models in Flatland by calling the **/spawn_models** service. Uses an array of Ped messages. Fields that can be filled:
|Type|Name|Description|typical value|
|---|---|---|---|
|int16 | id | agent id(*) | - |
|geometry_msgs/Point | pos | starting position | - |
|string | type | [Agent Type](#agent-types) | "adult" |
|int16 | number_of_peds | number of agents in this cluster | 1 |
|float64 | vmax | max. velocity | 1.4 |
|float64 | chatting_probability | Probability entering StateTalking | 0.01 |
|float64 | tell_story_probability | Probability entering StateTellStory | 0.01 |
|float64 | group_talking_probability | Probability entering StateGroupTalking | 0.01 |
|float64 | talking_and_walking_probability | Probability entering StateTalkingAndWalking | 0.01 |
|float64 | requesting_service_probability | Probability entering StateRequestingService | 0.01 |
|float64 | max_talking_distance | how close agents need to be, to be able to talk to them | 1.5 |
|float64 | max_servicing_radius | radius in which the service robot will provide service | 10.0 |
|float64 | talking_base_time | time agent spends being in StateTalking (***) | 10.0 |
|float64 | tell_story_base_time | time agent spends being in StateTellStory (***) | 10.0 |
|float64 | group_talking_base_time | time agent spends being in StateGroupTalking (***) | 10.0 |
|float64 | talking_and_walking_base_time | time agent spends being in StateTalkingAndWalking (***) | 10.0 |
|float64 | receiving_service_base_time | time agent spends being in StateReceivingService (***) | 30.0 |
|float64 | requesting_service_base_time | time agent spends being in StateRequestingService (***) | 30.0 |
|float64 | force_factor_desired | desired force will be multiplied by this factor | 1.0 |
|float64 | force_factor_obstacle | obstacle force will be multiplied by this factor | 2.0 |
|float64 | force_factor_social | social force will be multiplied by this factor | 2.0 |
|geometry_msgs/Point[] | waypoints | destinations for the agent | - |
|int16 | waypoint_mode | can be loop (0) or random (1) | 1 |
|string | yaml_file | path to flatland model file (**) | - |

(*) not used currently; agents automatically increment their id, starting at 1

(**) models are found in arena-rosnav/simulator_setup/dynamic_obstacles/

(***) this value will be multiplied by a random factor (0.5 to 1.5) every time the state gets activated

### /pedsim_simulator/reset_all_peds
Resets all agents to their starting position and resets their current destination. Uses std_srvs/Trigger Message.

### /pedsim_simulator/remove_all_peds
Removes all agents from Pedsim and also removes their corresponding models from Flatland. Uses std_srvs/SetBool Message.

### /pedsim_simulator/respawn_peds
Calls **remove_all_peds** first and then **spawn_peds** with the given array of Ped messages.

### /pedsim_simulator/spawn_interactive_obstacles
Spawns a number of interactive obstacles. Agents will interact with these when they get close. Additionally it spawns models in Flatland by calling the **/spawn_models** service. Uses an array of InteractiveObstacle messages. Fields that can be filled:
|Type|Name|Description|typical value|
|---|---|---|---|
| string | type | obstacle type, only "shelf" available currently | "shelf" |
| geometry_msgs/Pose | pose | position and angle | - |
| float64 | interaction_radius | if the appropriate agent enters this radius, it will interact | 4.0 |
|string | yaml_file | path to flatland model file (*) | - |

(*) models are found in arena-rosnav/simulator_setup/dynamic_obstacles/

### /pedsim_simulator/remove_all_interactive_obstacles
Removes all interactive obstacles from Pedsim and also removes their corresponding models from Flatland. Uses std_srvs/Trigger Message.

### /pedsim_simulator/respawn_interactive_obstacles
Calls **remove_all_interactive_obstacles** first and then **spawn_interactive_obstacles** with the given array of InteractiveObstacle messages.

# Notable Pedsim Topics
The most important topic is **/pedsim_simulator/simulated_agents**. It publishes an array of AgentState messages. The most important fields are:
|Type|Name|Description|
|---|---|---|
|string| type | [Agent Type](#agent-types) |
|string| social_state | [Agent State](#agent-states) |
|geometry_msgs/Pose | pose | position |
|geometry_msgs/Twist | twist | velocity |
|float64| direction | angle the agent is facing |

Note: **pose** contains information about orientation, just like **direction** does. **direction** is meant to be the angle, the agent is facing on a semantic level. Due to how Pedsim works, the orientation in **pose** can change very rapidly based on small movements.

# Agent Types
Agents currently can be one of four types which affects their behavior
|ID|Name|
|---|---|
|0|adult|
|1|child|
|2|elder|
|3|vehicle|
|4|servicerobot|

# Agent States
The agents behavior is controlled by a statemachine with the following states depending on agent type:
### Human agents:
#### StateWaiting
Standing still.
#### StateWalking
Walking to the next destination.
#### StateRunning
Like StateWalking but increased movement speed.
#### StateTalking
Talking to another agent, standing still.
#### StateTellStory
When entering this state the agent will stand still. Listeners will gather around in a circular pattern.
#### StateGroupTalking
When entering this state the agent triggers a group talk with the agent itself and other agents in range gathering in a cirular pattern.
#### StateListening
This state will be triggered whenever one of these is true:
- another agent is talking to this agent directly (StateTalking)
- another agent in close proximity is telling a story (StateTellStory)
- another agent in close proximity has initiated a group talk (StateGroupTalking)
#### StateTalkingAndWalking
Talk to another agent while walking to the next destination. This means that the listening agent will be walking directly next to this agent.
#### StateListeningAndWalking
This state will be triggered when another agent entered StateTalkingAndWalking with this agent as the recipient. This agent will walk directly next to the talking agent.
#### StateRequestingService
When entering this state the agent will slow down and signal to service robots that it's requesting service. If no service robot reached this agent in a given time it switches back to StateWalking.
#### StateReceivingService
This state will be activated once a service robot has reached this agent. Switch back to StateWalking after the configured time has passed.

### Vehicle (forklift) agents:
#### StateDriving
Driving to the next destination.
#### StateReachedShelf
Reached the destination (shelf) and turn towards it.
#### StateLiftingForks
Standing still.
#### StateLoading
Standing still.
#### StateLoweringForks
Standing still.

### Service Robot agents:
#### StateDriving
Driving to the next destination.
#### StateDrivingToInteraction
Activated once there is an agent in state StateRequestingService in a circular area around the service robot. Radius of the area can be configured by the parameter **max_servicing_radius** in pedsim_msgs/Ped. The Service robot will now drive towards the requesting agent.
#### StateProvidingService
Activated once the service robot has reached the requesting agent. Will stay in this state as long as the other agent is in StateReceivingService.
