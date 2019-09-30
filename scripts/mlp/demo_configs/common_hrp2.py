from hpp.corbaserver.rbprm.hrp2 import Robot
## following info are already accessible once the urdf is loaded, but here we do not load the urdf ...
MASS = 55.88363633
## weight and gains used by TSID


## predef duration of contact phases :
DURATION_INIT = 1. # Time to init the motion
DURATION_FINAL = 1.5 # Time to stop the robot
DURATION_FINAL_SS = 1. # duration of the final phase if it's a single support phase
DURATION_SS =1. # duration of the single support phases
DURATION_DS = 0.2 # duration of the double support phases
DURATION_TS = 0.4 # duration of the triple support phases
DURATION_CONNECT_GOAL = 2. # duration to try to connect the last points in the CoM trajectory with the goal position given to planning
# Hardcoded height change of the COM before the beginning of the motion (value in m and time allowed to make this motion)
# This is used because for some robot, the reference configuration is really close to the kinematic limits of the robot.
COM_SHIFT_Z = -0.05
TIME_SHIFT_COM = 2.


fMin = 1.0                      # minimum normal force
fMax = 1000.                   # maximum normal force
w_com = 1.0           # weight of center of mass task
w_am = 1.
w_posture = 0.1                # weight of joint posture task
w_rootOrientation = 1.       # weight of the root's orientation task
w_forceRef = 1e-6             # weight of force regularization task
w_eff = 1.0                     # weight of the effector motion task
kp_contact = 300.0               # proportional gain of contact constraint
kp_com = 100.                # proportional gain of center of mass task
kp_am = 100.
kp_posture = 1.              # proportional gain of joint posture task
kp_rootOrientation = 500.     # proportional gain of the root's orientation task
kp_Eff = 1000.                   # proportional gain ofthe effectors motion task
level_eff = 0
level_com = 0
level_posture = 1
level_rootOrientation = 1
level_am = 1

IK_dt = 0.005

IK_eff_size = Robot.dict_size.copy()
IK_eff_size[Robot.rfoot] = [0.5,0.8] # diameter of the surface under the flexibilitie
IK_eff_size[Robot.lfoot] = [0.5,0.8]

## Settings for end effectors : 
EFF_T_PREDEF = 0.3
EFF_T_DELAY = 0.05
FEET_MAX_VEL = 0.5
FEET_MAX_ANG_VEL = 1.5
p_max = 0.12

import numpy as np

gain_vector = np.matrix(
    [ 500. , 500.  , #chest
    100.,  100.,#head
    5.,   50.  , 10.,  10.,    10. ,  10. ,10.,  #larm
    5.,   50.  , 10., 10.,    10. ,  10. ,10., #rarm
    10. ,  5.  , 5. , 1. ,  1. ,  10., # lleg  #low gain on axis along y and knee
    10. ,  5.  , 5. , 1. ,  1. ,  10., #rleg    
]).T   # gain vector for postural task :

#gain_vector = np.matrix(np.ones(30)).T

masks_posture = np.matrix(np.ones(30)).T
#masks_posture[-12:] = 0 # legID
#print "mask postural task",masks_posture

IK_REFERENCE_CONFIG = np.matrix(Robot.referenceConfig).T