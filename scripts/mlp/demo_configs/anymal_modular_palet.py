TIMEOPT_CONFIG_FILE = "cfg_softConstraints_anymal_kinOrientation.yaml"
from common_anymal import *
SCRIPT_PATH = "sandbox.ANYmal"
#ENV_NAME = "ori/modular_palet_low_collision"
ENV_NAME = "ori/modular_palet_low_obstacles"
DEMO_NAME = "anymal_modular_palet_low"
#ENV_NAME = "ori/modular_palet_flat"
#DEMO_NAME = "anymal_modular_palet_flat"
DURATION_INIT = 2. # Time to init the motion
DURATION_FINAL = 2. # Time to stop the robot
DURATION_FINAL_SS = 1.
DURATION_SS = 2.
DURATION_DS = 2.
DURATION_TS = 0.7
DURATION_QS = 0.1
DURATION_CONNECT_GOAL = 0.
#COM_SHIFT_Z = -0.02
#TIME_SHIFT_COM = 1.
CHECK_DT = IK_dt*10.
#EFF_T_PREDEF = 0.1 # duration during which the motion of the end effector is forced to be orthogonal to the contact surface, at the beginning and the end of the phase
#kp_Eff = 100000.                   # proportional gain ofthe effectors motion task

## Override default settings :
#w_rootOrientation = 1.       # weight of the root's orientation task
#kp_rootOrientation = 5000.     # proportional gain of the root's orientation task

#p_max = 0.18
p_max = 0.25
