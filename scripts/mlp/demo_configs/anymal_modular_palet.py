TIMEOPT_CONFIG_FILE = "cfg_softConstraints_anymal_kinOrientation.yaml"
from common_anymal import *
SCRIPT_PATH = "sandbox.ANYmal"
ENV_NAME = "ori/modular_palet_low_collision"
DEMO_NAME = "anymal_modular_palet_low"

DURATION_INIT = 2. # Time to init the motion
DURATION_FINAL = 2. # Time to stop the robot
DURATION_FINAL_SS = 1.
DURATION_SS = 2.
DURATION_DS = 2.
DURATION_TS = 0.8
DURATION_QS = 0.5
DURATION_CONNECT_GOAL = 0.
#COM_SHIFT_Z = -0.02
#TIME_SHIFT_COM = 1.
kp_Eff = 100000.                   # proportional gain ofthe effectors motion task

FEET_MAX_VEL = 0.7
FEET_MAX_ANG_VEL = 1.5
p_max = 0.25
EFF_T_PREDEF = 0.
## Override default settings :
YAW_ROT_GAIN = 1.
#w_rootOrientation = 1.       # weight of the root's orientation task
#kp_rootOrientation = 5000.     # proportional gain of the root's orientation task

w_posture = 0.01                # weight of joint posture task
masks_posture = np.matrix(np.zeros(12)).T
for i in range(4):
    masks_posture[2+i*3] = 1. # knees