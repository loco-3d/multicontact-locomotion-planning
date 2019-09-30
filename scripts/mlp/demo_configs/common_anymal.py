from hpp.corbaserver.rbprm.anymal import Robot
MASS = 30.47 # cannot retrieve it from urdf because the file is not parsed here ... 

## weight and gains used by TSID

fMin = 1.0                      # minimum normal force
fMax = 1000.                  # maximum normal force
w_com = 1.0           # weight of center of mass task
w_am = 0.
w_posture = 0.01               # weight of joint posture task
w_rootOrientation = 1       # weight of the root's orientation task
w_forceRef = 1e-3               # weight of force regularization task
w_eff = 1.0                     # weight of the effector motion task
kp_contact = 30.0               # proportional gain of contact constraint
kp_com = 20.                 # proportional gain of center of mass task
kp_am = 20.
kp_posture = 50.              # proportional gain of joint posture task
kp_rootOrientation = 1000.     # proportional gain of the root's orientation task
kp_Eff = 100000.                   # proportional gain ofthe effectors motion task
level_eff = 1
level_com = 0
level_posture = 1
level_rootOrientation = 1
level_am = 1

YAW_ROT_GAIN = 1.

IK_eff_size = Robot.dict_size.copy()

# phases duration
DURATION_INIT = 1. # Time to init the motion
DURATION_FINAL = 1. # Time to stop the robot
DURATION_FINAL_SS = 1.
DURATION_SS =1.2
DURATION_DS = 1.2
DURATION_TS = 0.6
DURATION_QS = 0.1
DURATION_CONNECT_GOAL = 0.

## Settings for end effectors : 
EFF_T_DELAY = 0.
FEET_MAX_VEL = 0.7
FEET_MAX_ANG_VEL = 1.5
p_max = 0.25
EFF_T_PREDEF = 0.

import numpy as np
gain_vector = np.matrix(np.ones(12)).T
masks_posture = np.matrix(np.zeros(12)).T
for i in range(4):
    masks_posture[2+i*3] = 1. # knees

IK_REFERENCE_CONFIG = np.matrix(Robot.referenceConfig).T