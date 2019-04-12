## PATHS settings : 
import numpy as np
import os
PKG_PATH = os.environ['DEVEL_HPP_DIR']+"/src/hpp-wholebody-motion"
OUTPUT_DIR = PKG_PATH+"/res"
CONTACT_SEQUENCE_PATH = OUTPUT_DIR + "/contact_sequences"
TIME_OPT_CONFIG_PATH = PKG_PATH +'/timeOpt_configs'
LOAD_CS = True
LOAD_CS_COM = False
SAVE_CS = not LOAD_CS and True 
SAVE_CS_COM = not LOAD_CS_COM and True
EXPORT_GAZEBO = False
EXPORT_OPENHRP = True
openHRP_useZMPref = False
EXPORT_PATH = OUTPUT_DIR+"/export"

##DISPLAY settings : 
DISPLAY_CS = False # display contact sequence from rbprm
DISPLAY_CS_STONES = True # display stepping stones
DISPLAY_INIT_GUESS_TRAJ = False 
DISPLAY_WP_COST=True
DISPLAY_COM_TRAJ = True
DISPLAY_FEET_TRAJ = True
DISPLAY_WB_MOTION = False
DT_DISPLAY = 0.05 # dt used to display the wb motion
PLOT = True
DISPLAY_PLOT = True
SAVE_PLOT = True

###  Settings for generate_contact_sequence
FORCE_STRAIGHT_LINE = False # DEBUG ONLY should be false

### Settings for centroidal script :
MU=0.5
GRAVITY = np.matrix([0,0,-9.81]).T
USE_GEOM_INIT_GUESS = True
USE_CROC_INIT_GUESS = False
assert USE_GEOM_INIT_GUESS != USE_CROC_INIT_GUESS , "You must choose exactly one initial guess"
SOLVER_DT = 0.05 # hardcoded in timeOpt_configs files, must match this one ! 

# hardcoded height change between the init (goal) position from planning and the one given to the centroidal solver
COM_SHIFT_Z = 0.0
TIME_SHIFT_COM = 0.0

USE_WP_COST = False # use wp from the contact sequence in the cost function of timeopt

## Settings for end effector :
USE_LIMB_RRT = False
USE_CONSTRAINED_BEZIER = True
USE_BEZIER_EE = True
EFF_CHECK_COLLISION = True
WB_ABORT_WHEN_INVALID = False
WB_RETURN_INVALID = not WB_ABORT_WHEN_INVALID and True

##  Settings for whole body : 
YAW_ROT_GAIN = 1.
USE_CROC_COM = False
WB_VERBOSE = False
WB_STOP_AT_EACH_PHASE = False
IK_dt = 0.001  # controler time step
IK_PRINT_N = 500  # print state of the problem every IK_PRINT_N time steps (if verbose = True)
CHECK_FINAL_MOTION = True
IK_store_centroidal = True
IK_store_effector = True
IK_store_error = True
IK_store_contact_forces = True
# import specific settings for the selected demo. This settings may override default ones.
import importlib
import sys
if len(sys.argv)<2 : 
    print "You must call this script with the name of the config file (one of the file contained in hpp_wholebody_motion.demo_config)"
    print "Available demo files : "
    configs_path = PKG_PATH+"/scripts/hpp_wholebody_motion/demo_configs"
    demos_list = os.listdir(configs_path)
    for f in demos_list:
        if f.endswith(".py") and not f.startswith("__") : 
            print f.rstrip(".py")
    raise IOError("You must call this script with the name of the config file (one of the file contained in hpp_wholebody_motion.demo_config)")
    
import argparse
parser = argparse.ArgumentParser(description = "todo")
parser.add_argument('demo_name',type=str,help="The name of the demo configuration file to load")
args = parser.parse_args()
DEMO_NAME = args.demo_name
print "# Load demo config : ",DEMO_NAME
# Import the module
try :
    demo_cfg = importlib.import_module('hpp_wholebody_motion.demo_configs.'+DEMO_NAME)
except ImportError:
    raise NameError("No demo config file with the given name in hpp_wholebody_motion.demo_config")
# Determine a list of names to copy to the current name space
names = getattr(demo_cfg, '__all__', [n for n in dir(demo_cfg) if not n.startswith('_')])
# Copy those names into the current name space
g = globals()
for name in names:
    g[name] = getattr(demo_cfg, name)