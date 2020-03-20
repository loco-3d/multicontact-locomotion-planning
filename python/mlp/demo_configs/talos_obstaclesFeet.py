TIMEOPT_CONFIG_FILE = "cfg_softConstraints_talos.yaml"
from .common_talos import *
SCRIPT_PATH = "demos"

DURATION_INIT = 1.5  # Time to init the motion
DURATION_FINAL = 1.5  # Time to stop the robot
DURATION_FINAL_SS = 1.
DURATION_SS = 2.0
DURATION_DS = 0.3
DURATION_TS = 0.4

## Settings for end effectors :
EFF_T_PREDEF = 0.3
p_max = 0.1
