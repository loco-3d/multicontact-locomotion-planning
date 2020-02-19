TIMEOPT_CONFIG_FILE = "cfg_softConstraints_talos.yaml"
from .common_talos import *
#SCRIPT_ABSOLUTE_PATH = "mpcroc.planner_scenarios.talos.lp_complex1"
SCRIPT_ABSOLUTE_PATH = "mpcroc.planner_scenarios.talos.rubble_stairs"
ENV_NAME = "multicontact/bauzil_ramp_simplified"

DURATION_INIT = 2.  # Time to init the motion
DURATION_SS = 1.6
DURATION_DS = 0.3
DURATION_TS = 0.4
DURATION_CONNECT_GOAL = 0.

YAW_ROT_GAIN = 0.1
USE_PLANNING_ROOT_ORIENTATION = False  # if true, the reference for the root orientation is the one given by the planning (stored in phase.reference_configurations) if false, use the one of q_init for all the motion.
#USE_WP_COST = False # use wp from the contact sequence in the cost function of the centroidal solver

EFF_T_PREDEF = 0.2
p_max = 0.13

# Reference config used by the wholeBody script, may be different than the one used by the planning (default value is the same as planning)
IK_REFERENCE_CONFIG = np.array(Robot.referenceConfig_elbowsUp)
COM_SHIFT_Z = -0.025
TIME_SHIFT_COM = 2.

FEET_MAX_VEL = 0.4  # maximal linear velocity of the effector, if the current duration of the phase lead to a greater velocity than this setting, the duration of the phase is increased
FEET_MAX_ANG_VEL = 1.  # maximal angular velocity of the effectors
