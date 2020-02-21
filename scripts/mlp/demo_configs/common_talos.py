from talos_rbprm.talos import Robot
MASS = 90.27

GUIDE_STEP_SIZE = 1.
GUIDE_MAX_YAW = 0.6

## predef duration of contact phases :
DURATION_INIT = 1.  # Time to init the motion
DURATION_FINAL = 1.  # Time to stop the robot
DURATION_FINAL_SS = 1.  # duration of the final phase if it's a single support phase
DURATION_SS = 1.4  # duration of the single support phases
DURATION_DS = 0.2  # duration of the double support phases
DURATION_TS = 0.4  # duration of the triple support phases
DURATION_CONNECT_GOAL = 2.  # duration to try to connect the last points in the CoM trajectory with the goal position given to planning
# Hardcoded height change of the COM before the beginning of the motion (value in m and time allowed to make this motion)
# This is used because for some robot, the reference configuration is really close to the kinematic limits of the robot.
COM_SHIFT_Z = -0.03
TIME_SHIFT_COM = 2.

## weight and gains used by TSID
fMin = 1.0  # minimum normal force
fMax = 1000.  # maximum normal force
w_com = 1.0  # weight of center of mass task
w_am = 1e-2
w_posture = 0.1  # weight of joint posture task
w_rootOrientation = 1.  # weight of the root's orientation task
w_forceRef = 1e-3  # weight of force regularization task
w_eff = 1.0  # weight of the effector motion task
kp_contact = 30.0  # proportional gain of contact constraint
kp_com = 20.  # proportional gain of center of mass task
kp_am = 20.
kp_posture = 1.  # proportional gain of joint posture task
kp_rootOrientation = 50.  # proportional gain of the root's orientation task
kp_Eff = 50.  # proportional gain ofthe effectors motion task
level_eff = 0
level_com = 0
level_posture = 1
level_rootOrientation = 1
level_am = 1

IK_dt = 0.01
IK_eff_size = Robot.dict_size.copy()
#IK_eff_size={Robot.rfoot:[0.1 , 0.06], Robot.lfoot:[0.1 , 0.06],Robot.rhand:[0.1, 0.1],Robot.lhand:[0.1, 0.1]}

## Settings for end effectors :
EFF_T_PREDEF = 0.2  # duration during which the motion of the end effector is forced to be orthogonal to the contact surface, at the beginning and the end of the phase
EFF_T_DELAY = 0.05  # duration at the beginning and the end of the phase where the effector don't move
FEET_MAX_VEL = 0.5  # maximal linear velocity of the effector, if the current duration of the phase lead to a greater velocity than this setting, the duration of the phase is increased
FEET_MAX_ANG_VEL = 1.5  # maximal angular velocity of the effectors
p_max = 0.1  #setting used to compute the default height of the effector trajectory. end_effector/bezier_predef.py : computePosOffset()

import numpy as np
gain_vector = np.array(  # gain vector for postural task
    [
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
        5000.,
        5000.,  #chest
        500.,
        1000.,
        10.,
        10.,
        10.,
        10.,
        100.,
        50.,  #larm
        50.,
        100.,
        10.,
        10.,
        10.,
        10.,
        100.,
        50.,  #rarm
        100.,
        100.
    ]  #head
)

masks_posture = np.ones(32)
#masks_posture[:11] = 0

# Reference config used by the wholeBody script, may be different than the one used by the planning (default value is the same as planning)
IK_REFERENCE_CONFIG = np.array(Robot.referenceConfig)
