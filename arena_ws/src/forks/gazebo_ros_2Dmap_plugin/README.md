# gazebo_ros_2Dmap_plugin
This is a fork of [gazebo_ros_2Dmap_plugin](marinaKollmitz/gazebo_ros_2Dmap_plugin) adapted to be used in conjunction with [arena-rosnav-3D](https://github.com/ignc-research/arena-rosnav-3D).

This version adds an additional service to the original repo, namely to create a "layered" obstacle map of any exisiting Gazebo world. User can specify the height interval, which should be considered during the exploration, as well as the step size of exploration simulation, resulting in a single 2D occupancy map, covering a specific height range of the 3D world. 

## arena-rosnav-3D
We are using this plugin in our simulation environment [arena-rosnav-3D](https://github.com/ignc-research/arena-rosnav-3D) to seamlessly create a 2d map used by the robot for navigation, as well as our manager nodes for random goal / position generation. On top of that we are using the extra "layered" map functionality to generate a separate occupancy map used solely by our obstacle manager and pedsim simulator, making sure that both of those are supplied with more accurate information. Based on your use case your results may vary, but in our experience specifying additional, "layered" map for pedsim agents results in a more realistic behaviour.

## Usage 
If you are coming from **arena-rosnav-3D**, then this plugin should already be in your catkin_ws/src/forks folder, if not then clone this repo into that directory and follow these steps

- Make sure that you have the following package installed
```
sudo apt-get install ros-noetic-octomap
```

- Check out the plugin in your `catkin_ws` and build it with `catkin_make`.
To include the plugin, add the following line in between the `<world> </world>` tags of your Gazebo world file:

```
<plugin name='gazebo_occupancy_map' filename='libgazebo_2Dmap_plugin.so'>
    <map_resolution>0.1</map_resolution> <!-- in meters, optional, default 0.1 -->
    <map_height>0.3</map_height>         <!-- in meters, optional, default 0.3 -->
    <check_below>true</check_below>      <!-- Check for obstacles below map height, default false -->
    <lower_bound>0.3</lower_bound>       <!-- Lower bound  in meters at which obstacles below map height will be looked at, default 0.3 -->
    <step>0.1</step>                     <!-- in meters, defines the heights at which occupancy check will be made, giving a range of [map_height, lower_bound[, default 0.1 -->
    <map_size_x>10</map_size_x>          <!-- in meters, optional, default 10 -->
    <map_size_y>10</map_size_y>          <!-- in meters, optional, default 10 -->
</plugin>
```
#### Notes on the parameters
- **check_below**: set to true, if you want the plugin to start checking for obstacles at **map_height** till the **lower_bound** is reached
- **lower_bound**: defines the lowest height at which the obstacles exploration may be performed, depending on the **step_size** the exact value may not be used, instead a value within (**lower_bound**,**lower_bound + **step_size**) will be used
- **step_size**: in meters, determines the next height at which, the exploration will be executed. The first exploration happens at height **map_height**, next at at **map_height** - **step_size**, and so on.


To generate the map, call the `/gazebo_2Dmap_plugin/generate_map` ros service:

```
rosservice call /gazebo_2Dmap_plugin/generate_map
```

The generated map is published on the `/map2d` ros topic. 

You can use the `map_saver` node from the `map_server` package inside ros navigation to save your generated map to a .pgm and .yaml file:

```
rosrun map_server map_saver -f <mapname> /map:=/map2d
```
The last map generated with the ```/gazebo_2Dmap_plugin/generate_map``` call is saved.

## Hints

* To identify the connected free space the robot would discover during mapping, the plugin performs a wavefront exploration along the occupancy grid starting from the origin of the gazebo world coordinate system. Please ensure that the corresponding cell is in the continuous free space. 
* The plugin will map all objects in the world, including the robot. Remove all unwanted  objects before creating the map. 
