#DEMO_NAME = "talos_flatGround"
DEMO_NAME = "darpa_hyq"


import importlib
# Import the module
demo_cfg = importlib.import_module('hpp_wholebody_motion.demo_configs.'+DEMO_NAME)
# Determine a list of names to copy to the current name space
names = getattr(demo_cfg, '__all__', [n for n in dir(demo_cfg) if not n.startswith('_')])
# Copy those names into the current name space
g = globals()
for name in names:
    g[name] = getattr(demo_cfg, name)

## PATHS settings : 

import os
PKG_PATH = os.environ['DEVEL_HPP_DIR']+"/src/hpp-wholebody-motion"
OUTPUT_DIR = PKG_PATH+"/res"
CONTACT_SEQUENCE_PATH = OUTPUT_DIR + "/contact_sequences"
TIME_OPT_CONFIG_PATH = PKG_PATH +'/timeOpt_configs'
SAVE_CS = True
SAVE_CS_COM = True

##DISPLAY settings : 
DISPLAY_CS = False # display contact sequence from rbprm
DISPLAY_CS_STONES = True # display stepping stones
DISPLAY_INIT_GUESS_TRAJ = False 
DISPLAY_COM_TRAJ = True
DISPLAY_FEET_TRAJ = True
DISPLAY_WB_MOTION = True
DT_DISPLAY = 0.05 # dt used to display the wb motion

###  Settings for generate_contact_sequence
FORCE_STRAIGHT_LINE = False # DEBUG ONLY should be false

### Settings for locomote script :
MU=0.5
USE_GEOM_INIT_GUESS = True
USE_CROC_INIT_GUESS = False
assert USE_GEOM_INIT_GUESS != USE_CROC_INIT_GUESS , "You must choose exactly one initial guess"
SOLVER_DT = 0.05 # hardcoded in timeOpt_configs files, must match this one ! 

##  Settings for whole body : 
USE_BEZIER_EE = True
WB_VERBOSE = True
WB_STOP_AT_EACH_PHASE = False
IK_dt = 0.001  # controler time step
IK_PRINT_N = 100  # print state of the problem every IK_PRINT_N time steps (if verbose = True)


"""
# only used by muscod scripts

ZMP_RADIUS = 0.01 #m
MU_FOOT = 0.5
MU_HAND = 0.5
FN_FOOT = 1200.
FT_FOOT = MU_FOOT * FN_FOOT
MN_FOOT = 400.
#MT_FOOT = FN_FOOT * ZMP_RADIUS
MT_FOOT = MN_FOOT
FN_HAND = 150.
FT_HAND = MU_HAND * FN_HAND
MN_HAND = 100.
#MT_HAND = FN_HAND * ZMP_RADIUS
MT_HAND = MN_HAND

NUM_NODES_INIT = 2
NUM_NODES_FINAL = 3
NUM_NODES_SS = 3
NUM_NODES_DS = 2
NUM_NODES_TS = 2
"""