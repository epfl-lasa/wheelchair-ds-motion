# wheelchair-ds-motion
DS-based motion planning for the quickie-salsa wheelchair simulated in Gazebo, as shown below:

<p align="center">
<img src="https://github.com/epfl-lasa/wheelchair-ds-motion/blob/master/figs/Road1-scenario.png" width="750"></>

### Dependencies
To run this package you must install the following dependencies:
- [quickie-salsa-m2](https://github.com/sinamr66/quickie_salsa_m2) ``checkout 'nadia' branch`` | Control Interface for Quickie-Salsa Wheelchair in Gazebo
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

To learn your own lpv-DS models, download and follow the instructions in the [ds-opt](https://github.com/nbfigueroa/ds-opt) package. 

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

**References**   
> [1] Figueroa, N. and Billard, A. (2018) "A Physically-Consistent Bayesian Non-Parametric Mixture Model for Dynamical System Learning". Conference on Robot Learning (CoRL) - 2018 Edition. To Appear. 

**Contact**: [Nadia Figueroa](http://lasa.epfl.ch/people/member.php?SCIPER=238387) (nadia.figueroafernandez AT epfl dot ch)

**Acknowledgments**
This work was supported by the EU project [Cogimon](https://cogimon.eu/cognitive-interaction-motion-cogimon) H2020-ICT-23-2014 and [Crowdbot](https://project.inria.fr/crowdbot/) H2020-ICT-25-2016-2017.
