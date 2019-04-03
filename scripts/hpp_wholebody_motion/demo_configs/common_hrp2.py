from hpp.corbaserver.rbprm.hrp2 import Robot
MASS = 55.88363633

## weight and gains used by TSID

fMin = 1.0                      # minimum normal force
fMax = 1000.0                   # maximum normal force
w_com = 1.0                     # weight of center of mass task
w_posture = 0.001                 # weight of joint posture task
w_rootOrientation = 1.0         # weight of the root's orientation task
w_forceRef = 1e-3               # weight of force regularization task
w_eff = 1.0                     # weight of the effector motion task
kp_contact = 30.0               # proportional gain of contact constraint
kp_com = 1000.0                 # proportional gain of center of mass task
kp_posture = 50.                # proportional gain of joint posture task
kp_rootOrientation = 1000.     # proportional gain of the root's orientation task
kp_Eff = 1000.                   # proportional gain ofthe effectors motion task


IK_dt = 0.005



import numpy as np

gain_vector = np.matrix(
    [ 500. , 500.  , #chest
    100.,  100.,#head
    10.,   50.  , 10.,  10.,    10. ,  10. ,10.,  #larm
    10.,   50.  , 10., 10.,    10. ,  10. ,10., #rarm
    10. ,  5.  , 5. , 1. ,  1. ,  10., # lleg  #low gain on axis along y and knee
    10. ,  5.  , 5. , 1. ,  1. ,  10., #rleg    
]).T   # gain vector for postural task :

#gain_vector = np.matrix(np.ones(30)).T

masks_posture = np.matrix(np.ones(30)).T
#masks_posture[-12:] = 0 # legID
#print "mask postural task",masks_posture