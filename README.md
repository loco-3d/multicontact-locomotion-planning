This project implement the framework for mulicontact locomotion planning proposed in the loco-3d project. 
This framework decompose the global locomotion problem in several subproblems solved sequentially : 

* The contact planning
* Centroidal trajectory optimization
* Wholebody motion generation

It contains wrapper for the different blocks of the framework and script to automatically formulate problems, retrieve and store solutions and connect every blocks together. 
Along with a lot of helpers for visualization or exports. 

The goal of this framework is to be modular, allowing the user to select a method for each of the subproblem or add a wrapper for any new method from the state-of-the-art solving one of this subproblem and connect it seamlessly to the rest of the framework. 

# Installation procedure :

This package rely on a lot of optionnal packages, see the section 'Available methods' for more information on the installation of this optional packages.

## Install depencies from binary repositories :

1. Add robotpkg to your apt configuration: http://robotpkg.openrobots.org/debian.html
2. `sudo apt update && sudo apt install robotpkg-py27-pinocchio robotpkg-py27-multicontact-api`

## Install depencies from sources : 

Follow the instruction from https://github.com/loco-3d/multicontact-api

## Installation procedure : 

Clone the repository
``` 
git clone https://github.com/loco-3d/multicontact-locomotion-planning.git
``` 
And add it to your PYTHONPATH.

# Usage

**TODO**

# Available methods :

Currently supported method for each subproblem, you need to install the packages linked separetely if you want to use this methods.

## Contact generation : 

* RBPRM (https://github.com/humanoid-path-planner/hpp-rbprm-corba#installation-from-binary-package-repository)

## Centroidal trajectory optimization : 

* timeoptimization (https://git-amd.tuebingen.mpg.de/bponton/timeoptimization and it's wrapper https://github.com/ggory15/timeopt )
* CROC (https://hal.archives-ouvertes.fr/hal-01726155), included in RBPRM

## Wholebody motion generation :

* TSID (https://github.com/stack-of-tasks/tsid)

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
* npz numpy archive (containing all the datas of the wholebody motion)
