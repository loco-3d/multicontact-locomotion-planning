TIMEOPT_CONFIG_FILE = "cfg_softConstraints_talos.yaml"
from common_talos import *
SCRIPT_PATH = "demos"
ENV_NAME = "multicontact/ground"

centroidal_method = "quasistatic" 


DURATION_INIT = 2. # Time to init the motion
DURATION_FINAL = 2. # Time to stop the robot
DURATION_FINAL_SS = 1.
DURATION_SS =1.6
DURATION_DS = 2.
DURATION_TS = 0.4
DURATION_CONNECT_GOAL = 1.

COM_SHIFT_Z = -0.03
