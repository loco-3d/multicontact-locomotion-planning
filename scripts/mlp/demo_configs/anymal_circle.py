TIMEOPT_CONFIG_FILE = "cfg_softConstraints_anymal_sidesteps.yaml"
from .common_anymal import *
SCRIPT_PATH = "memmo"
ENV_NAME = "multicontact/ground"

DURATION_INIT = 2.  # Time to init the motion
DURATION_FINAL = 2.  # Time to stop the robot
DURATION_FINAL_SS = 1.
DURATION_SS = 2.
DURATION_DS = 2.
DURATION_TS = 0.3
DURATION_QS = 0.05
DURATION_CONNECT_GOAL = 0.

#COM_SHIFT_Z = -0.02
#TIME_SHIFT_COM = 1.
#kp_com = 100.                 # proportional gain of center of mass task
#kp_rootOrientation = 5000.     # proportional gain of the root's orientation task
YAW_ROT_GAIN = 1.

p_max = 0.25
