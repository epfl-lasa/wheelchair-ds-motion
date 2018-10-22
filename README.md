# wheelchair-ds-motion
DS-based motion planning for the quickie-salsa wheelchair simulated in Gazebo, as shown below:

<p align="center">
<img src="https://github.com/epfl-lasa/wheelchair-ds-motion/blob/master/figs/Road1-scenario.png" width="750"></>

### Dependencies
To run this package you must install the following dependencies:
- [quickie-salsa-m2](https://github.com/sinamr66/quickie_salsa_m2) ``checkout 'nadia' branch`` | Control Interface for Quicki-Salsa Wheelchair in Gazebo
- [ds-motion-generator](https://github.com/epfl-lasa/ds_motion_generator.git) ``checkout 'nadia' branch`` | DS motion generation nodes
- [lpvDS-lib](https://github.com/nbfigueroa/lpvDS-lib) | lpv-DS class used by ds-motion-generator, should be installed automatically if using ``wstool`` with the [ds-motion-generator](https://github.com/epfl-lasa/ds_motion_generator.git) package.


### Instructions
**Step 1** Bring-up Gazebo Wheelchair Simulator of the Road World and RViz for DS Visualization  
```
$ roslaunch wheelchair_ds_motion ds_simulation.launch world:=_road
```

**Step 2** To run a non-linear DS (lpv formulation) with streamline visualization in rviz:
- Load the DS model
	```
	$ roslaunch wheelchair_ds_motion run_nonlinearDS_controller.launch 
	```
	The attractor and type of DS must are set in "DS_name" parameter, there are currently 2 options:
	```xml
	<arg name="DS_name" value="2D-W-Nav"/>
	<arg name="DS_name" value="2D-U-Nav"/>
	```
	which points to the ``.yml`` file in the [ds-motion-generator](https://github.com/epfl-lasa/ds_motion_generator) package.

- To control the wheelchair with the loaded DS, run the following command:
	```
	$ rosrun wheelchair_ds_motion nonlinearDS_controller.py
	```

**Optional** To run a simple linear DS with a pre-defined attractor:
```
$ roslaunch wheelchair_ds_motion run_linearDS_controller.launch 
```
To define the attractor and if obstacles should be present or not, modify the following line:
```xml
<arg name="ctrl_command"   default="10 0 1" />
```
- parameters: ``<x-position of attractor> <y-position of attractor> <number of obstacles> ``

Without obstacle avoidance, simply set the last parameter to 0.
