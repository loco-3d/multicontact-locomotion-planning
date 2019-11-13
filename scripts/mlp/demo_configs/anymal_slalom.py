TIMEOPT_CONFIG_FILE = "cfg_softConstraints_anymal_sidesteps.yaml"
from common_anymal import *
SCRIPT_PATH = "sandbox.ANYmal"
ENV_NAME = "ori/slalom1"
DEMO_NAME = "anymal_slalom1"


DURATION_INIT = 2. # Time to init the motion
DURATION_FINAL = 2. # Time to stop the robot
DURATION_FINAL_SS = 1.
DURATION_SS = 2.
DURATION_DS = 2.
DURATION_TS = 0.3
DURATION_QS = 0.05
DURATION_CONNECT_GOAL = 0.

#COM_SHIFT_Z = -0.02
#TIME_SHIFT_COM = 1.
YAW_ROT_GAIN = 1.

p_max = 0.25


## Override default settings :
