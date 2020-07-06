This project implement the framework for multicontact locomotion planning proposed in the [loco-3d](https://hal.laas.fr/hal-01543060) project. 
This framework decompose the global locomotion problem in several subproblems solved sequentially, as shown in the figure below:  

![Framework proposed by the Loco3D project.](doc/loco3D_framework.png "Framework proposed by the Loco3D project. ")


In details, this package implement the following architecture: 

![Architecture implemented in this package.](doc/diag_mlp.png "Architecture implemented in this package. ")


Where all the connections between the blocks are made with objects from the [multicontact-API](https://github.com/loco-3d/multicontact-api) package. Thanks to this, we can have a modular architecture with different methods for solving each blocks but with an unified API. 

The multicontact-locomotion-planning package doesn't implement any of the solvers/methods for any of the blocks but it contains wrapper for the different blocks of the framework and script to automatically formulate problems, retrieve and store solutions such that the input and output of each wrapper is an object from the [multicontact-API](https://github.com/loco-3d/multicontact-api) package. 
Then it implement the connection between each block, along with a lot of helpers for visualization or exports. 

This wrappers are python script that take a user input or an object from the  [multicontact-API](https://github.com/loco-3d/multicontact-api) package, generate a generic problem from the input and call the API of a specific solver. Then it retrieve the solution from the solver and correctly store it in a [multicontact-API](https://github.com/loco-3d/multicontact-api) object. 

The goal of this framework is to be modular, allowing the user to select a method for each of the subproblem or add a wrapper for any new method from the state-of-the-art solving one of this subproblem and connect it seamlessly to the rest of the framework. 

Slides of a presentation of this package: https://cloud.laas.fr/index.php/s/CFkJbxrC9SYQ34i

Basic tutorials slides are availables: https://cloud.laas.fr/index.php/s/Y4xeJtY1gn3tFN8

# Installation procedure :

This package rely on a lot of optionnal packages, see the section 'Available methods' for more information on the installation of this optional packages.

## Install depencies from binary repositories :

1. Add robotpkg to your apt configuration: http://robotpkg.openrobots.org/debian.html and robotpkg/wip: http://robotpkg.openrobots.org/robotpkg-wip.html
2. `sudo apt update && sudo apt install robotpkg-py3\*-pinocchio robotpkg-py3\*-multicontact-api robotpkg-py3\*-hpp-rbprm-corba robotpkg-py3\*-qt4-hpp-gepetto-viewer`

## Install depencies from sources : 

Follow the instruction from https://github.com/loco-3d/multicontact-api

## Installation procedure : 
Once the depencies are correctly installed, clone the repository :
``` 
git clone --recursive https://github.com/loco-3d/multicontact-locomotion-planning.git
``` 
And install it:

``` 
cd multicontact-locomotion-planning
mkdir build ; cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DPYTHON_EXECUTABLE=$(which python3) .. ; make install
``` 

# Usage

## Basic usage:

The main class of this package is https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/python/loco_planner.py .This script call all the required solver to compute the complete framework as shown in the figure above. You have to call this script with an additional argument `DEMO_NAME`.
A main script is also available, that take care of running the rquired background process (gepetto-viewer and hpp-rbprm corbaserver) and stopping them after the end of the script. To use it simply run: 

```python
python -im mlp DEMO_NAME
```

The value of `DEMO_NAME` can be the name of any file inside the folder python/mlp/demo_configs/ (without the extension), or the absolute path to any other valid scenario file in your PYTHONPATH. 


### Understanding configuration files:

The user should never modify the main classes (loco_planner.py) but only the various configuration files. When launching the main script, it load three configuration files in the following order:

* mlp/config.py : this is the main configuration file, it can be edited to change:
  * The default method used to solve each subproblem https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/python/mlp/config.py#L23-L29
  * The various path for all the external files used https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/python/mlp/config.py#L36-L48
  * Enabling or disabling the various export https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/python/mlp/config.py#L51-L62
  * Enabling or disabling the display of several items https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/python/mlp/config.py#L65-L80
  * Various default setting for each specific methods
* mlp/demo_configs/common_*.py : This file contains all the robot-specific settings:
  * The rbprm Robot class related to this robot https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/python/mlp/demo_configs/common_talos.py#L1
  * The mass of the robot https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/python/mlp/demo_configs/common_talos.py#L2
  * Default duration of each contact phases https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/python/mlp/demo_configs/common_talos.py#L5-L11
  * Various gains value and task priority used by the wholeBody script https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/python/mlp/demo_configs/common_talos.py#L17-L36
  * Default setting for the end-effector trajectory generation https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/python/mlp/demo_configs/common_talos.py#L43-L47
* mlp/demo_configs/DEMO_NAME.py : This last file contains all the fine tuning specific to each scenario.
  * Select the robot used https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/python/mlp/demo_configs/talos_circle.py#L2
  * If using rbprm to solver the contact planning, it should specify the folder inside hpp-rbprm-corba/script/scenario that contain the script https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/python/mlp/demo_configs/talos_circle.py#L3
  * Set the name of the environment file, inside the package hpp-environments https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/python/mlp/demo_configs/talos_circle.py#L4
  * Then, it can change the value of any the values defined in one of the previous configuration files
  
#### External configuration file for specific solvers:

Some solvers called as external library by this package may require other configuration files, they are stored in specific folders (eg. momentumopt_configs for momentumopt solver). The file(s) used by this solvers are choosen inside the  mlp/demo_configs/DEMO_NAME.py configuration file (https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/scripts/mlp/demo_configs/talos_circle.py#L1).

## Load motion from files:
*DEPRECATED*

The script https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/scripts/load_motion_from_files.py can load a motion from a `*_COM.cs` and a `*.npz` file. 

* Edit the variables `path`, `npzFile` and `csFile` inside the script
* run `gepetto-gui` in a separate terminal
* run python `load_motion_from_files.py`

**This script currently have hardcoded values for the Talos robot.** This is done to avoid other depencies. One can remove the class `Robot` hardcoded in this script and instead load one of the various `Robot` classes found in HPP-RBPRM. 

# Available methods :

Currently supported method for each subproblem, you need to install the packages linked separetely if you want to use this methods.

## Contact generation : 

* RBPRM (https://github.com/humanoid-path-planner/hpp-rbprm-corba#installation-from-binary-package-repository)
* SL1M (https://gepgitlab.laas.fr/loco-3d/sl1m)
* Manually defined (helper methods are available to easily define a gait or a sequence of effector positions)

## Centroidal trajectory optimization : 

* momentumopt (https://github.com/machines-in-motion/kino_dynamic_opt)
* CROC (https://hal.archives-ouvertes.fr/hal-01726155), included in RBPRM
* 2-PAC (quasi-static) (https://hal.archives-ouvertes.fr/hal-01609055) included in RBPRM

## Wholebody motion generation :

* TSID (https://github.com/stack-of-tasks/tsid)
* CROCODDYL (Work in progress) (https://github.com/loco-3d/crocoddyl)

## Effector trajectories :

* Basic predefined splines
* Limb-RRT (geometric path found by path-planning). Require RBPRM 
* Optimized limb-rrt (Dynamically consistent spline based on the results of limb-rrt)

## Visualization : 

* gepetto-viewer (https://github.com/Gepetto/gepetto-viewer-corba#setup-from-robotpkg-apt-binary-package-repository)

## Export : 

* Blender (with plugin https://github.com/Gepetto/gepetto-viewer-corba/tree/master/blender)
* Gazebo (basic joint-trajectory export)
* OpenHRP (only for HRP-2 robot)
* npz numpy archive (containing all the datas of the wholebody motion) (see https://github.com/MeMory-of-MOtion/docker-loco3d#details-on-the-npz-archive  for details)

# Reactive planning (Work In Progess):

The script loco_planner_reactive.py is a WIP script to allow reactive replanning using MLP. It run the centroidal and wholebody blocks with a time-horizon instead of running them sequentially for the whole motion. It also run them in separate processes in parrallel and display the motion as soon as the first step is computed. 

This script currently allow to:
* Request a stop during the motion (Solve a 0-step capturability problem to bring the robot at a stop). WIP: extend it to a 1-step capturability if the 0-step problem fail
* Request a change in the goal position/orientation of the motion during it's execution
* Add an obstacle in the environment, and re plan the motion if required

To use this script, run:

``` 
python3 -i loco_planner_reactive.py DEMO_NAME
``` 

Here, the `DEMO_NAME` initial and goal position defined by the DEMO file are not used. 
*WIP*: This script currently work only for Talos and only with some of the methods for each blocks.

Once the initialization is done, the following python code can be executed:

```python
ref_height = loco_planner.fullBody.referenceConfig[2]

# Add a goal 2m in front of the robot
loco_planner.move_to_goal([2., 0., ref_height])


# Add an ostacle in the path
loco_planner.add_obstacle([0.01, 0.2, 0.4], [1.2, 0.15, 0.2])


# Change the goal to 1.5m on the left
loco_planner.move_to_goal([1., 1.5, ref_height])


# Add an obstacle out of the way
loco_planner.add_obstacle([0.5, 0.5, 0.5], [0.3, 0.2, 0.25])


# Add an obstacle behind the robot
loco_planner.add_obstacle([0.2, 0.1, 0.4], [1.5, 0.4, 0.2])


# STOP the motion
loco_planner.stop_motion()

```

# Contributing

## Package organization: 

Inside python/mlp/ there is one folder for each of the block shown in the figure of the introduction. In each of the subfolder there is one script for each different method that we can use to solve this block.

## Adding new wrapper:

If you want to use a new method to solve one of the subproblem of our proposed framework, you should add a new script in the coresponding folder.

Basically, this script should implement a wrapper between the multicontact-api structure and the solver that you want to use. It should call the API of the solver to generically formulate a problem from the given input, solve the problem and store the results in a multicontact-api object. 




