from hpp.corbaserver.rbprm.anymal_contact6D import Robot
MASS = 30.47 # cannot retrieve it from urdf because the file is not parsed here ... 

## weight and gains used by TSID

fMin = 1.0                      # minimum normal force
fMax = 1000.0                  # maximum normal force
w_com = 1.0                     # weight of center of mass task
w_posture = 0.75                 # weight of joint posture task
w_rootOrientation = 1.0         # weight of the root's orientation task
w_forceRef = 1e-3               # weight of force regularization task
w_eff = 1.0                     # weight of the effector motion task
kp_contact = 30.0               # proportional gain of contact constraint
kp_com = 3000.0                 # proportional gain of center of mass task
kp_posture = 10.0               # proportional gain of joint posture task
kp_rootOrientation = 500.0      # proportional gain of the root's orientation task
kp_Eff = 3000.0                 # proportional gain ofthe effectors motion task

import numpy as np
gain_vector = np.matrix(np.ones(24)).T
masks_posture = np.matrix(np.zeros(24)).T

