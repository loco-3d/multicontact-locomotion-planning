TIMEOPT_CONFIG_FILE = "cfg_softConstraints_anymal_kinOrientation.yaml"
from .common_anymal import *
SCRIPT_PATH = "memmo"
ENV_NAME = "multicontact/ground"

DURATION_INIT = 2.  # Time to init the motion
DURATION_FINAL = 2.  # Time to stop the robot
DURATION_FINAL_SS = 1.
DURATION_SS = 2.
DURATION_DS = 2.
DURATION_TS = 0.7
DURATION_QS = 0.5

#COM_SHIFT_Z = -0.02
#TIME_SHIFT_COM = 1.

FEET_MAX_VEL = 0.7
FEET_MAX_ANG_VEL = 1.5

## Override default settings :
YAW_ROT_GAIN = 1.
