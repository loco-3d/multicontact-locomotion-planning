TIMEOPT_CONFIG_FILE = "cfg_softConstraints_talos.yaml"
from common_talos import *
SCRIPT_PATH = "memmo"
ENV_NAME = "multicontact/maze_easy"

DURATION_SS =1.2
DURATION_DS = 0.2

EFF_T_PREDEF = 0.2
EFF_T_DELAY = 0.05
FEET_MAX_VEL = 1.
FEET_MAX_ANG_VEL = 1.5
p_max = 0.07
USE_PLANNING_ROOT_ORIENTATION = True

GUIDE_STEP_SIZE = 0.7

IK_REFERENCE_CONFIG = np.matrix(Robot.referenceConfig_elbowsUp).T #Reference config used by the wholeBody script, may be different than the one used by the planning (default value is the same as planning)

