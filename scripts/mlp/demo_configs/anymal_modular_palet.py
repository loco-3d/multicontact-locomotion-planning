TIMEOPT_CONFIG_FILE = "cfg_softConstraints_anymal_kinOrientation.yaml"
from common_anymal import *
SCRIPT_PATH = "sandbox.ANYmal"
ENV_NAME = "ori/modular_palet_low_collision"
DEMO_NAME = "anymal_modular_palet_low"

DURATION_INIT = 2. # Time to init the motion
DURATION_FINAL = 2. # Time to stop the robot
DURATION_FINAL_SS = 1.
DURATION_SS = 2.
DURATION_DS = 2.
DURATION_TS = 0.5
DURATION_QS = 0.1
DURATION_CONNECT_GOAL = 0.
#COM_SHIFT_Z = -0.02
#TIME_SHIFT_COM = 1.

## Override default settings :
#w_rootOrientation = 1.       # weight of the root's orientation task
#kp_rootOrientation = 5000.     # proportional gain of the root's orientation task

