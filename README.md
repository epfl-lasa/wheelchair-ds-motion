# wheelchair-ds-motion
DS-based motion planning (including obstacle avoidance) for the quickie-salsa wheelchair simulated in Gazebo.

## Instructions
**Step 1** Bring-up Gazebo Wheelchair Simulator and RViz for DS Visualization  
```
$ roslaunch wheelchair_ds_motion ds_simulation.launch
```
To change the 'world' configurations to include obstacles change the following variables:
```xml
	<arg name="no_obstacle" default="true"/>
	<arg name="one_obstacle" default="false"/>
	<arg name="multi_obstacles" default="false"/>
	```

**Step 2** To run a simple linear DS with obstacle avoidance:
```
$ rosrun wheelchair_ds_motion simple_velocity_controller.py 8 0 1
```
- parameters: <x-position of attractor> <y-position of attractor> <number of obstacles>  

Without obstacle avoidance, simply set the last parameter to 0.
  
