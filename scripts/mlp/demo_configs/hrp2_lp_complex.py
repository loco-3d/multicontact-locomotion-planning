TIMEOPT_CONFIG_FILE = "cfg_softConstraints_hrp2.yaml"
from common_hrp2 import *
SCRIPT_ABSOLUTE_PATH = "mpcroc.planner_scenarios.lp_complex"
ENV_NAME = "multicontact/bauzil_ramp"



DURATION_INIT = 2. # Time to init the motion
DURATION_SS =1.6
DURATION_DS = 0.3
DURATION_TS = 0.4
DURATION_CONNECT_GOAL = 2.


COM_SHIFT_Z = -0.05
TIME_SHIFT_COM = 2.


FEET_MAX_VEL = 0.4 # maximal linear velocity of the effector, if the current duration of the phase lead to a greater velocity than this setting, the duration of the phase is increased
FEET_MAX_ANG_VEL = 1.5 # maximal angular velocity of the effectors

IK_REFERENCE_CONFIG = np.matrix(Robot.referenceConfig_elbowsUp).T