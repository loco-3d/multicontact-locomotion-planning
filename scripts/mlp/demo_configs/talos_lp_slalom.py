TIMEOPT_CONFIG_FILE = "cfg_softConstraints_talos.yaml"
from common_talos import *
SCRIPT_ABSOLUTE_PATH = "mpcroc.planner_scenarios.talos.lp_slalom_debris"
ENV_NAME = "multicontact/slalom_debris"



DURATION_INIT = 2. # Time to init the motion
DURATION_SS =1.8
DURATION_DS = 0.3
DURATION_TS = 0.4
DURATION_CONNECT_GOAL = 0.

YAW_ROT_GAIN = 0.1
USE_PLANNING_ROOT_ORIENTATION = False # if true, the reference for the root orientation is the one given by the planning (stored in phase.reference_configurations) if false, use the one of q_init for all the motion.
#USE_WP_COST = False # use wp from the contact sequence in the cost function of the centroidal solver


EFF_T_PREDEF = 0.3
p_max = 0.2

IK_REFERENCE_CONFIG = np.matrix(Robot.referenceConfig_elbowsUp).T #Reference config used by the wholeBody script, may be different than the one used by the planning (default value is the same as planning)
COM_SHIFT_Z = -0.04
TIME_SHIFT_COM = 2.

FEET_MAX_VEL = 0.5 # maximal linear velocity of the effector, if the current duration of the phase lead to a greater velocity than this setting, the duration of the phase is increased
FEET_MAX_ANG_VEL = 1.5 # maximal angular velocity of the effectors