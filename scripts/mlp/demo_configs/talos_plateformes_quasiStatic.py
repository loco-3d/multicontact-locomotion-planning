TIMEOPT_CONFIG_FILE = "cfg_softConstraints_talos.yaml"
from common_talos import *
SCRIPT_PATH = "demos"
ENV_NAME = "multicontact/plateforme_surfaces"
DEMO_NAME="talos_plateformes"
centroidal_method = "quasistatic" 
#kp_com = 1.                 # proportional gain of center of mass task
w_com = 1.0           # weight of center of mass task
w_am = 0.
w_posture = 0.1          # weight of joint posture task
w_rootOrientation = 1.       # weight of the root's orientation task
w_forceRef = 1e-5          # weight of force regularization task
w_eff = 1.0                     # weight of the effector motion task
kp_contact = 100.0               # proportional gain of contact constraint
kp_com = 20.                 # proportional gain of center of mass task
kp_am = 20.
kp_posture = 1.              # proportional gain of joint posture task
kp_rootOrientation = 500.     # proportional gain of the root's orientation task
kp_Eff = 2000.                   # proportional gain ofthe effectors motion task
level_eff = 0
level_com = 0
level_posture = 1
level_rootOrientation = 1
level_am = 1

DURATION_INIT = 4. # Time to init the motion
DURATION_FINAL = 4. # Time to stop the robot
DURATION_FINAL_SS = 1.
DURATION_SS =2.5
DURATION_DS = 4.
DURATION_TS = 0.4
DURATION_CONNECT_GOAL = 2.

COM_SHIFT_Z = -0.05

EFF_T_PREDEF = 0.3
p_max = 0.1

