TIMEOPT_CONFIG_FILE = "cfg_softConstraints_solo.yaml"
from .common_solo import *
SCRIPT_PATH = "demos"
ENV_NAME = "ori/modular_palet_low"

# phases duration
DURATION_INIT = 1.  # Time to init the motion
DURATION_FINAL = 1.  # Time to stop the robot
DURATION_TS = 0.5
DURATION_QS = 0.1
DURATION_CONNECT_GOAL = 2.
USE_PLANNING_ROOT_ORIENTATION = True
GUIDE_MAX_YAW = 0.5 # maximal yaw rotation difference between two discretization step
GUIDE_STEP_SIZE = 0.5

