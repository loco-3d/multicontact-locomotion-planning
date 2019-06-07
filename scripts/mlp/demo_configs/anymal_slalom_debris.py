TIMEOPT_CONFIG_FILE = "cfg_softConstraints_anymal.yaml"
from common_anymal import *
SCRIPT_PATH = "sandbox.ANYmal"
ENV_NAME = "multicontact/slalom_debris"


DURATION_INIT = 2. # Time to init the motion
DURATION_FINAL = 2. # Time to stop the robot
DURATION_FINAL_SS = 1.
DURATION_SS = 2.
DURATION_DS = 2.
DURATION_TS = 1.5
DURATION_QS = 1.

#COM_SHIFT_Z = -0.02
#TIME_SHIFT_COM = 1.



## Override default settings :
YAW_ROT_GAIN = 0.3
