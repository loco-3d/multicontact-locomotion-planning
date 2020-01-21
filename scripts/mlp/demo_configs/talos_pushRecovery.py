TIMEOPT_CONFIG_FILE = "cfg_softConstraints_talos_dt001.yaml"
from .common_talos import *
SCRIPT_PATH = "sandbox"
ENV_NAME = "multicontact/ground"

DURATION_INIT = 0.5  # Time to init the motion
DURATION_FINAL = 2.0  # Time to stop the robot
DURATION_FINAL_SS = 0.8
DURATION_SS = 0.8
DURATION_DS = 0.2
DURATION_TS = 0.4
SOLVER_DT = 0.02  # hardcoded in timeOpt_configs files, must match this one !

## Settings for end effectors :
EFF_T_PREDEF = 0.2
p_max = 0.1

## setting for the IK :
kp_com = 10000.0  # proportional gain of center of mass task
w_com = 1.0  # weight of center of mass task
kp_Eff = 10000.0  # proportional gain ofthe effectors motion task
w_eff = 2.0  # weight of the effector motion task
IK_PRINT_N = 50
