TIMEOPT_CONFIG_FILE = "cfg_softConstraints_talos.yaml"
from .common_talos import *
SCRIPT_PATH = "demos"
ENV_NAME = "multicontact/floor_bauzil_obstacles"
DEMO_NAME = "talos_navBauzil_obstacles"

kp_Eff = 500.  # proportional gain of the effectors motion task


DURATION_SS = 1.2
DURATION_DS = 0.3

kp_rootOrientation = 50000.  # proportional gain of the root's orientation task

EFF_T_PREDEF = 0.2
EFF_T_DELAY = 0.05
FEET_MAX_VEL = 1.
FEET_MAX_ANG_VEL = 1.5
p_max = 0.07
USE_PLANNING_ROOT_ORIENTATION = True
DURATION_CONNECT_GOAL = 0.

GUIDE_STEP_SIZE = 0.6

# Reference config used by the wholeBody script, may be different than the one used by the planning (default value is the same as planning)
IK_REFERENCE_CONFIG = np.array(Robot.referenceConfig_elbowsUp)
