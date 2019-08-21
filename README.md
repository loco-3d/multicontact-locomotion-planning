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

Slides of a presentation of this package : https://cloud.laas.fr/index.php/s/3GbEHLld94asr8H

# Installation procedure :

This package rely on a lot of optionnal packages, see the section 'Available methods' for more information on the installation of this optional packages.

## Install depencies from binary repositories :

1. Add robotpkg to your apt configuration: http://robotpkg.openrobots.org/debian.html
2. `sudo apt update && sudo apt install robotpkg-py27-pinocchio robotpkg-py27-multicontact-api`

## Install depencies from sources : 

Follow the instruction from https://github.com/loco-3d/multicontact-api

## Installation procedure : 
Once the depencies are correctly installed, clone the repository :
``` 
git clone https://github.com/loco-3d/multicontact-locomotion-planning.git
``` 
And add the directory `multicontact-locomotion-planning/script`  to your PYTHONPATH.

# Usage

## Basic usage:

### Start background process:

* If you want a visualization of the motion, you have to start gepetto-gui server before launching any script of this package. Run `gepetto-gui` in a separate terminal. After execution of any script, you should kill this process and restart it. 

* Most of the scripts require the hpp-rbprm server.  Run `hpp-rbprm-server` in a separate terminal. After execution of any script, you should kill this process and restart it. 

### Launch the main script:

The main script of this package is https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/scripts/run_mlp.py .This script call all the required solver to compute the complete framework as shown in the figure above. You have to call this script with an additional argument `DEMO_NAME` : 

```python
python run_mlp.py DEMO_NAME
```

The value of `DEMO_NAME` can be the name of any file inside the folder script/mlp/demo_configs/ (without the extension and without the complete path). 


### Understanding configuration files:

The user should never modify the main script (run_mlp.py) but only the various configuration files. When launching the main script, it load three configuration files in the following order:

* mlp/config.py : this is the main configuration file, it can be edited to change:
  * The default method used to solve each subproblem https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/scripts/mlp/config.py#L11-L15
  * The various path for all the external files used https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/scripts/mlp/config.py#L17-L3
  * Enabling or disabling the various export https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/scripts/mlp/config.py#L24-L33
  * Enabling or disabling the display of several items https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/scripts/mlp/config.py#L34-L47
  * Various default setting for each specific methods
* mlp/demo_configs/common_*.py : This file contains all the robot-specific settings:
  * The rbprm Robot class related to this robot https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/scripts/mlp/demo_configs/common_talos.py#L1
  * The mass of the robot https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/scripts/mlp/demo_configs/common_talos.py#L2
  * Default duration of each contact phases https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/scripts/mlp/demo_configs/common_talos.py#L5-L11
  * Various gains value and task priority used by the wholeBody script https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/scripts/mlp/demo_configs/common_talos.py#L17-L36
  * Default setting for the end-effector trajectory generation https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/scripts/mlp/demo_configs/common_talos.py#L43-L47
* mlp/demo_configs/DEMO_NAME.py : This last file contains all the fine tuning specific to each scenario.
  * Select the robot used https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/scripts/mlp/demo_configs/talos_circle.py#L2
  * If using rbprm to solver the contact planning, it should specify the folder inside hpp-rbprm-corba/script/scenario that contain the script https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/scripts/mlp/demo_configs/talos_circle.py#L3
  * Set the name of the environment file, inside the package hpp-environments https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/scripts/mlp/demo_configs/talos_circle.py#L4
  * Then, it can change the value of any the values defined in one of the previous configuration files
  
#### External configuration file for specific solvers:

Some solvers called as external library by this package may require other configuration files, they are stored in specific folders (eg. timeOpt_configs for timeOpt solver). The file(s) used by this solvers are choosen inside the  mlp/demo_configs/DEMO_NAME.py configuration file (https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/scripts/mlp/demo_configs/talos_circle.py#L1).

## Load motion from files:

The script https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/scripts/load_motion_from_files.py can load a motion from a `*_COM.cs` and a `*.npz` file. 

* Edit the variables `path`, `npzFile` and `csFile` inside the script
* run `gepetto-gui` in a separate terminal
* run python `load_motion_from_files.py`

**This script currently have hardcoded values for the Talos robot.** This is done to avoid other depencies. One can remove the class `Robot` hardcoded in this script and instead load one of the various `Robot` classes found in HPP-RBPRM. 

# Available methods :

Currently supported method for each subproblem, you need to install the packages linked separetely if you want to use this methods.

## Contact generation : 

* RBPRM (https://github.com/humanoid-path-planner/hpp-rbprm-corba#installation-from-binary-package-repository)
* Manually defined (helper methods are available to easily define a gait or a sequence of effector positions)

## Centroidal trajectory optimization : 

* timeoptimization (https://git-amd.tuebingen.mpg.de/bponton/timeoptimization and it's wrapper https://github.com/ggory15/timeopt )
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

# Contributing

## Package organization: 

Inside scripts/mlp/ there is one folder for each of the block shown in the figure of the introduction. In each of the subfolder there is one script for each different method that we can use to solve this block. There is also an `__init__.py` file that choose the correct script to import according to the solver choosen in the configuration files. 

## Adding new wrapper:

If you want to use a new method to solve one of the subproblem of our proposed framework, you should add a new script in the coresponding folder. This script should implement the method used in the `__init__.py` file of this subfolder with the same prototype.

Basically, this script should implement a wrapper between the multicontact-api structure and the solver that you want to use. It should call the API of the solver to generically formulate a problem from the given input, solve the problem and store the results in a multicontact-api object. 




