TIMEOPT_CONFIG_FILE = "cfg_softConstraints_talos_rarm.yaml"
from common_talos import *
SCRIPT_PATH = "sandbox.dynamic"
ENV_NAME = "multicontact/table_140_70_73"
DURATION_INIT = 1.5 # Time to init the motion
DURATION_FINAL = 2.9 # Time to stop the robot
DURATION_FINAL_SS = 1.
DURATION_SS =1.5
DURATION_DS = 2.9
DURATION_TS = 2.9

FEET_MAX_VEL = 5
FEET_MAX_ANG_VEL = 5

EFF_CHECK_COLLISION = False


## Settings for end effectors : 
EFF_T_PREDEF = 0.3
p_max = 0.1


"""
centroidal_method = "quasistatic" 
DURATION_INIT = 4. # Time to init the motion
DURATION_FINAL = 4. # Time to stop the robot
DURATION_FINAL_SS = 1.
DURATION_SS =3.
DURATION_DS = 3.
DURATION_TS = 3.
DURATION_CONNECT_GOAL = 2.

COM_SHIFT_Z = -0.03
"""