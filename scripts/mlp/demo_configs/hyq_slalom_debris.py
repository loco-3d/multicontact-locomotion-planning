TIMEOPT_CONFIG_FILE = "cfg_softConstraints_hyq.yaml"
from common_hyq import *
SCRIPT_PATH = "demos"

DURATION_INIT = 1.5 # Time to init the motion
DURATION_FINAL = 1.5 # Time to stop the robot
DURATION_FINAL_SS = 1.
DURATION_SS =0.8
DURATION_DS = 0.8
DURATION_TS = 0.8
DURATION_QS = 0.3


## Settings for end effectors : 
EFF_T_PREDEF = 0.1
p_max = 0.2

## Override default settings :
YAW_ROT_GAIN = 0.1
