#DEMO_NAME = "talos_flatGround"
#DEMO_NAME = "darpa_hyq"
DEMO_NAME = "hyq_slalom_debris"



## PATHS settings : 

import os
PKG_PATH = os.environ['DEVEL_HPP_DIR']+"/src/hpp-wholebody-motion"
OUTPUT_DIR = PKG_PATH+"/res"
CONTACT_SEQUENCE_PATH = OUTPUT_DIR + "/contact_sequences"
TIME_OPT_CONFIG_PATH = PKG_PATH +'/timeOpt_configs'
SAVE_CS = True
SAVE_CS_COM = True

##DISPLAY settings : 
DISPLAY_CS = True # display contact sequence from rbprm
DISPLAY_CS_STONES = True # display stepping stones
DISPLAY_INIT_GUESS_TRAJ = False 
DISPLAY_WP_COST=False
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

USE_WP_COST = True # use wp from the contact sequence in the cost function of timeopt

##  Settings for whole body : 
YAW_ROT_GAIN = 1.
USE_CROC_COM = False
USE_BEZIER_EE = True
WB_VERBOSE = False
WB_STOP_AT_EACH_PHASE = False
IK_dt = 0.001  # controler time step
IK_PRINT_N = 500  # print state of the problem every IK_PRINT_N time steps (if verbose = True)


# import specific settings for the selected demo. This settings may override default ones.
import importlib
# Import the module
demo_cfg = importlib.import_module('hpp_wholebody_motion.demo_configs.'+DEMO_NAME)
# Determine a list of names to copy to the current name space
names = getattr(demo_cfg, '__all__', [n for n in dir(demo_cfg) if not n.startswith('_')])
# Copy those names into the current name space
g = globals()
for name in names:
    g[name] = getattr(demo_cfg, name)