from talos_rbprm.talos_fixedUpper import Robot
MASS = 90.27

## predef duration of contact phases :
DURATION_INIT = 1.  # Time to init the motion
DURATION_FINAL = 1.5  # Time to stop the robot
DURATION_FINAL_SS = 1.
DURATION_SS = 1.
DURATION_DS = 0.2
DURATION_TS = 0.4
DURATION_CONNECT_GOAL = 2.
COM_SHIFT_Z = -0.015
TIME_SHIFT_COM = 1.5

## weight and gains used by TSID
fMin = 1.0  # minimum normal force
fMax = 1000.  # maximum normal force
w_com = 1.0  # weight of center of mass task
w_am = 0.1
w_posture = 1e-5  # weight of joint posture task
w_rootOrientation = 1.  # weight of the root's orientation task
w_forceRef = 1e-3  # weight of force regularization task
w_eff = 1.0  # weight of the effector motion task
kp_contact = 30.0  # proportional gain of contact constraint
kp_com = 20.  # proportional gain of center of mass task
kp_am = 20.
kp_posture = 1.  # proportional gain of joint posture task
kp_rootOrientation = 1000.  # proportional gain of the root's orientation task
kp_Eff = 2000.  # proportional gain ofthe effectors motion task
level_eff = 0
level_com = 0
level_posture = 1
level_rootOrientation = 1
level_am = 1

IK_dt = 0.001
IK_eff_size = Robot.dict_size.copy()
#IK_eff_size={Robot.rfoot:[0.1 , 0.06], Robot.lfoot:[0.1 , 0.06],Robot.rhand:[0.1, 0.1],Robot.lhand:[0.1, 0.1]}

## Settings for end effectors :
EFF_T_PREDEF = 0.2
EFF_T_DELAY = 0.05
FEET_MAX_VEL = 0.5
FEET_MAX_ANG_VEL = 1.5
p_max = 0.07

import numpy as np
gain_vector = np.array([
    10.,
    5.,
    5.,
    1.,
    1.,
    10.,  # lleg  #low gain on axis along y and knee
    10.,
    5.,
    5.,
    1.,
    1.,
    10.,  #rleg
])  # gain vector for postural task :

masks_posture = np.ones(12)
#masks_posture[:11] = 0
