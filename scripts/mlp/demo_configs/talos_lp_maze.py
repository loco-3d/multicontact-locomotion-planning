TIMEOPT_CONFIG_FILE = "cfg_softConstraints_talos.yaml"
from .common_talos import *
SCRIPT_ABSOLUTE_PATH = "scenarios.sandbox.talos_maze"
ENV_NAME = "multicontact/maze_easy"
contact_generation_method = "lp"
"""
DURATION_INIT = 2. # Time to init the motion
DURATION_SS =1.6
DURATION_DS = 0.3
DURATION_TS = 0.4
DURATION_CONNECT_GOAL = 0.
"""
DURATION_CONNECT_GOAL = 0.

GUIDE_STEP_SIZE = 0.8
GUIDE_MAX_YAW = 1.

YAW_ROT_GAIN = 0.1
USE_PLANNING_ROOT_ORIENTATION = False  # if true, the reference for the root orientation is the one given by the planning (stored in phase.reference_configurations) if false, use the one of q_init for all the motion.
#USE_WP_COST = False # use wp from the contact sequence in the cost function of the centroidal solver

#EFF_T_PREDEF = 0.2
#p_max = 0.08

IK_REFERENCE_CONFIG = np.matrix(
    Robot.referenceConfig_elbowsUp
).T  #Reference config used by the wholeBody script, may be different than the one used by the planning (default value is the same as planning)

#FEET_MAX_VEL = 0.4 # maximal linear velocity of the effector, if the current duration of the phase lead to a greater velocity than this setting, the duration of the phase is increased
#FEET_MAX_ANG_VEL = 1. # maximal angular velocity of the effectors
