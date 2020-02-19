TIMEOPT_CONFIG_FILE = "cfg_softConstraints_talos_eff9.yaml"
from .common_talos import *
SCRIPT_PATH = "memmo"
#ENV_NAME = "multicontact/ground"

kp_am = 1.
w_am = 0.5

DURATION_INIT = 1.  # Time to init the motion
DURATION_FINAL = 1.  # Time to stop the robot
DURATION_CONNECT_GOAL = 0.
DURATION_SS = 1.4
COM_SHIFT_Z = 0.
TIME_SHIFT_COM = 0.

EFF_T_PREDEF = 0.2
EFF_T_DELAY = 0.05
FEET_MAX_VEL = 100.
FEET_MAX_ANG_VEL = 100.
p_max = 0.07

YAW_ROT_GAIN = 1e-6
end_effector_method = "limbRRToptimized"
