TIMEOPT_CONFIG_FILE = "cfg_softConstraints_talos.yaml"
from common_talos_fixedUpper import *
SCRIPT_PATH = "demos"
ENV_NAME = "multicontact/ground"
DEMO_NAME = "talos_flatGround"
centroidal_method = "quasistatic" 


DURATION_INIT = 3. # Time to init the motion
DURATION_FINAL = 3. # Time to stop the robot
DURATION_FINAL_SS = 1.
DURATION_SS =3.
DURATION_DS = 3.
DURATION_TS = 0.4
DURATION_CONNECT_GOAL = 1.

EFF_T_PREDEF = 0.3
EFF_T_DELAY = 0.05
FEET_MAX_VEL = 0.5
FEET_MAX_ANG_VEL = 1.5
p_max = 0.1


COM_SHIFT_Z = -0.03
