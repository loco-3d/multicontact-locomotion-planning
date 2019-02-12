from hpp.corbaserver.rbprm.talos import Robot
MASS = 90.27

## weight and gains used by TSID

fMin = 5.0                      # minimum normal force
fMax = 1500.0                   # maximum normal force
w_com = 1.0                     # weight of center of mass task
w_posture = 0.75                 # weight of joint posture task
w_rootOrientation = 1.0         # weight of the root's orientation task
w_forceRef = 1e-3               # weight of force regularization task
w_eff = 1.0                     # weight of the effector motion task
kp_contact = 30.0               # proportional gain of contact constraint
kp_com = 3000.0                 # proportional gain of center of mass task
kp_posture = 30.0               # proportional gain of joint posture task
kp_rootOrientation = 500.0      # proportional gain of the root's orientation task
kp_Eff = 3000.0                 # proportional gain ofthe effectors motion task

import numpy as np
gain_vector = np.matrix(
    [   0. ,   0.,   0. ,  #root
     50. , 50. , 5.  , # root rotation
     0.1 ,  0.1  , 0.02 , 0.01 ,  0.02 ,  0.1, # lleg  #low gain on axis along y and knee
    0.1 ,  0.1  , 0.02 , 0.01 ,  0.02 ,  0.1, #rleg
    50. , 50.  , #chest
    1.,   1.  , 1.,  1.,    1. ,  1. , 1. ,  1. , #larm
    1.,   1.  , 1., 1.,    1. ,  1. ,  1. ,  1. , #rarm
   10.,  10.] #head
    ).T   # gain vector for postural task :
gain_vector = np.matrix(np.ones(32)).T

masks_posture = np.matrix(np.ones(32)).T
masks_posture[:11] = 0