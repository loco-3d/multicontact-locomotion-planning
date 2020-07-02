TIMEOPT_CONFIG_FILE = "cfg_softConstraints_talos.yaml"
from .common_talos import *
SCRIPT_PATH = "demos"
ENV_NAME = "multicontact/ground"

DURATION_SS = 1.6
DURATION_DS = 0.5

EFF_T_PREDEF = 0.2
EFF_T_DELAY = 0.05
FEET_MAX_VEL = 1.
FEET_MAX_ANG_VEL = 1.5
p_max = 0.07
COM_SHIFT_Z = -0.05

DURATION_CONNECT_GOAL = 0.

USE_PLANNING_ROOT_ORIENTATION = True
GUIDE_STEP_SIZE = 0.5
GUIDE_MAX_YAW = 0.5 # maximal yaw rotation difference between two discretization step
"""
IK_REFERENCE_CONFIG = np.array(
[       0.0,
        0.0,
        1.02127,
        0.0,
        0.0,
        0.0,
        1.,  # Free flyer
        0.0,
        0.0,
        -0.411354,
        0.859395,
        -0.448041,
        -0.001708,  # Left Leg
        0.0,
        0.0,
        -0.411354,
        0.859395,
        -0.448041,
        -0.001708,  # Right Leg
        0.0,
        0.006761,  # Chest
        0.40,
        0.24,
        -0.6,
        -1.45,
        0.0,
        -0.0,
        0.,
        -0.005,  # Left Arm
        -0.4,
        -0.24,
        0.6,
        -1.45,
        0.0,
        0.0,
        0.,
        -0.005,  # Right Arm
        0.,
        0.
    ])
"""
IK_REFERENCE_CONFIG = np.array( # half-sitting but with lower CoM
[0.,   0.,   9.90948142e-01,
        -4.20231975e-07,  -7.12429207e-05,  -4.99415574e-04,
         9.99999873e-01,  -1.69956223e-07,   3.77519575e-06,
        -5.16865149e-01,   1.07581459e+00,  -5.58806952e-01,
        -1.71100592e-03,  -1.69956223e-07,   3.77519501e-06,
        -5.16864772e-01,   1.07581386e+00,  -5.58806604e-01,
        -1.71100591e-03,  -1.11681753e-07,   6.74049522e-03,
         2.58539680e-01,   1.73028928e-01,  -6.54547519e-04,
        -5.17579943e-01,   1.00965079e-04,  -1.67166593e-04,
         1.00080423e-01,  -5.00276891e-03,  -2.59200420e-01,
        -1.72939684e-01,   6.31076645e-04,  -5.17624366e-01,
        -2.75913104e-05,   1.69408413e-04,   1.00077062e-01,
        -5.00258594e-03,  -3.06293412e-05,  -5.77487237e-08]
)
# IK_REFERENCE_CONFIG = np.array(Robot.referenceConfig)
