## Integration into your project
The arena-evaluation repository can likely be added to your repository. 

It relies on the following topics being published by the robot-simulator:
- `/scan`: The scan-topic of the laser scanner
- `/odom`: The odometry information of the robot
- `/cmd_vel`: The robots command-velocity
- `/scenario_reset`: Provides the number of repeats of the current scenario.

It relies on the following parameters being set by the robot simulator :
- `local_planner`: The name of the local planner e.g. *dwa* being used
- `model`: The name of the robot model e.g. *burger* being used
- `waypoint_generator`: Whether the waypoint generator is activated [True, False]
- `record_only_planner`: Whether only the planner should be recorded [True, False]
- `scenario_file`: The name of the scenario file being used

To integrate this evaluation class into your project, clone it into your catkin workspace (or put it into your *.rosinstall* file). 
Then, inside your catkin workspace do:

```bash
rosws update && catkin_make
```

After successfully making, you can use the evaluation class by including the following part into the roslaunch file from which you will start your simulation:

```xml
  <!-- data recorder -->
  <group if="$(eval arg('use_recorder') == true)">
    <node pkg="arena-evaluation" name="data_recorder_node" type="data_recorder_node.py" />
    <param name="local_planner" value="$(arg local_planner)"/>
    <param name="waypoint_generator" value="$(arg waypoint_generator)"/>
    <param name="record_only_planner" value="$(arg record_only_planner)"/>
    <param name="scenario_file" value="$(arg scenario_file)"/>
    <param name="model" value="$(arg model)"/>
  </group>

```

> ⚠️ This documentation is not complete. Including the this code into your repository will likely require additional customization