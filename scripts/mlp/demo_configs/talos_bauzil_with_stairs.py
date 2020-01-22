TIMEOPT_CONFIG_FILE = "cfg_softConstraints_talos.yaml"
from .common_talos import *
SCRIPT_PATH = "memmo"
ENV_NAME = "multicontact/bauzil_stairs"

DURATION_INIT = 1.5  # Time to init the motion
DURATION_FINAL = 1  # Time to stop the robot
DURATION_FINAL_SS = 1.
DURATION_SS = 1.4
DURATION_DS = 0.3
DURATION_TS = 0.4
DURATION_CONNECT_GOAL = 0.

p_max = 0.13  #setting used to compute the default height of the effector trajectory. end_effector/bezier_predef.py : computePosOffset()

GUIDE_STEP_SIZE = 0.6

# Reference config used by the wholeBody script, may be different than the one used by the planning (default value is the same as planning)
IK_REFERENCE_CONFIG = np.matrix(
    Robot.referenceConfig_elbowsUp
).T
