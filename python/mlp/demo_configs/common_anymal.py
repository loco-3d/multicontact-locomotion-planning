try:
    from anymal_rbprm.anymal import Robot
except ImportError:
    message = "ERROR: Cannot import anymal-rbprm package.\n"
    message += "Did you correctly installed it?\n"
    message +="https://github.com/humanoid-path-planner/anymal-rbprm"
    raise ImportError(message)
MASS = 30.47  # cannot retrieve it from urdf because the file is not parsed here ...

## weight and gains used by TSID
IK_dt = 0.0025  # 400Hz
fMin = 1.0  # minimum normal force
fMax = 200.  # maximum normal force
w_com = 1.0  # weight of center of mass task
w_am = 0.
w_posture = 0.01  # weight of joint posture task
w_rootOrientation = 1.  # weight of the root's orientation task
w_eff = 1.0  # weight of the effector motion task
kp_contact = 50.0  # proportional gain of contact constraint
kp_com = 30.  # proportional gain of center of mass task
kp_am = 20.
kp_posture = 70.  # proportional gain of joint posture task
kp_rootOrientation = 5000.  # proportional gain of the root's orientation task
kp_Eff = 100000.  # proportional gain ofthe effectors motion task
level_eff = 1
level_com = 0
level_posture = 1
level_rootOrientation = 1
level_am = 1

# The weight of the force regularization task of each contact will start at w_forceRef_init when creating a new contact,
# and then linearly reduce to w_forceRef_end over a time period of phase_duration * w_forceRef_time_ratio
w_forceRef_init = 1.
w_forceRef_end = 1e-5
w_forceRef_time_ratio = 0.5

YAW_ROT_GAIN = 1.

IK_eff_size = Robot.dict_size.copy()
PLOT_CIRCLE_RADIUS = 0.01 # radius of the circle used to display the contacts

# phases duration
DURATION_INIT = 1.  # Time to init the motion
DURATION_FINAL = 1.  # Time to stop the robot
DURATION_FINAL_SS = 1.
DURATION_SS = 1.2
DURATION_DS = 1.2
DURATION_TS = 0.6
DURATION_QS = 0.1
DURATION_CONNECT_GOAL = 0.

## Settings for end effectors :
EFF_T_DELAY = 0.
FEET_MAX_VEL = 0.7
FEET_MAX_ANG_VEL = 1.5
p_max = 0.2
EFF_T_PREDEF = 0.

GUIDE_STEP_SIZE = 0.3
SL1M_USE_MIP = True

import numpy as np
gain_vector = np.ones(12)
masks_posture = np.zeros(12)
for i in range(4):
    masks_posture[2 + i * 3] = 1.  # knees

IK_REFERENCE_CONFIG = np.array(Robot.referenceConfig)
