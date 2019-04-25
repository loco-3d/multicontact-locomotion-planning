from hpp.corbaserver.rbprm.anymal_contact6D import Robot
MASS = 30.47 # cannot retrieve it from urdf because the file is not parsed here ... 

## weight and gains used by TSID

fMin = 1.0                      # minimum normal force
fMax = 200.                  # maximum normal force
w_com = 1.0                     # weight of center of mass task
w_posture = 1e-3                 # weight of joint posture task
w_am = 1.
w_rootOrientation = 1.0         # weight of the root's orientation task
w_forceRef = 1e-5               # weight of force regularization task
w_eff = 1.0                     # weight of the effector motion task
kp_contact = 10.0               # proportional gain of contact constraint
kp_com = 10.0                # proportional gain of center of mass task
kp_posture = 10.0               # proportional gain of joint posture task
kp_am = 10.
kp_rootOrientation = 50.0     # proportional gain of the root's orientation task
kp_Eff = 100.0                 # proportional gain ofthe effectors motion task
level_eff = 0
level_com = 0
level_posture = 1
level_rootOrientation = 1
level_am = 1

IK_eff_size = Robot.dict_size.copy()

## Settings for end effectors : 
EFF_T_PREDEF = 0.1
EFF_T_DELAY = 0.05
p_max = 0.1

import numpy as np
gain_vector = np.matrix(np.ones(24)).T
masks_posture = np.matrix(np.zeros(24)).T

