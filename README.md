This project implement the framework for mulicontact locomotion planning proposed in the loco-3d project. 
This framework decompose the global locomotion problem in several subproblems solved sequentially : 

* The contact planning
* Centroidal trajectory optimization
* Wholebody motion generation

It contains wrapper for the different blocks of the framework and script to automatically formulate problems, retrieve and store solutions and connect every blocks together. 
Along with a lot of helpers for visualization or exports. 

The goal of this framework is to be modular, allowing the user to select a method for each of the subproblem or add a wrapper for any new method from the state-of-the-art solving one of this subproblem and connect it seamlessly to the rest of the framework. 

Currently supported method for each subproblem : 

## Contact generation : 

* RBPRM (https://github.com/humanoid-path-planner/hpp-rbprm)

## Centroidal trajectory optimization : 

* timeoptimization (https://git-amd.tuebingen.mpg.de/bponton/timeoptimization)
* CROC (https://hal.archives-ouvertes.fr/hal-01726155)

## Wholebody motion generation :

* TSID (https://github.com/stack-of-tasks/tsid)

## Visualization : 

* gepetto-viewer (https://github.com/Gepetto/gepetto-viewer)

## Export : 

* Blender (with plugin https://github.com/Gepetto/gepetto-viewer-corba/tree/master/blender)
* Gazebo (basic joint-trajectory export)
* OpenHRP (only for HRP-2 robot)
