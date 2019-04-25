import numpy as np
import os

contact_generation_method_available = ["none","load", "rbprm"]
centroidal_initGuess_method_available = ["none", "geometric", "croc", "timeopt", "quasistatic"]
centroidal_method_available = ["load", "geometric", "croc", "timeopt", "quasistatic", "muscod"]
wholebody_method_available = ["load", "tsid", "croccodyl"]
end_effector_method_available = ["smoothedFoot", "bezierPredef", "bezierConstrained", "limbRRT", "limbRRToptimized"]

## methods setting : choose which method will be used to solve each subproblem : 
contact_generation_method = "rbprm" 
centroidal_initGuess_method = "geometric" 
centroidal_method = "timeopt" 
wholebody_method = "tsid" 
end_effector_method = "bezierPredef" 

## PATHS settings : 
PKG_PATH = os.environ['DEVEL_HPP_DIR']+"/src/multicontact-locomotion-planning"
OUTPUT_DIR = PKG_PATH+"/res"
CONTACT_SEQUENCE_PATH = OUTPUT_DIR + "/contact_sequences"
TIME_OPT_CONFIG_PATH = PKG_PATH +'/timeOpt_configs'
STATUS_FILENAME = "infos.log"
SAVE_CS = True 
SAVE_CS_COM = True
EXPORT_GAZEBO = False
EXPORT_OPENHRP = False
EXPORT_NPZ = True
EXPORT_BLENDER = False
openHRP_useZMPref = False
EXPORT_PATH = OUTPUT_DIR+"/export"
WRITE_STATUS = True
##DISPLAY settings : 
DISPLAY_CS = False # display contact sequence from rbprm
DISPLAY_CS_STONES = False # display stepping stones
DISPLAY_INIT_GUESS_TRAJ = False 
DISPLAY_WP_COST=False
DISPLAY_COM_TRAJ = False
DISPLAY_FEET_TRAJ = False
DISPLAY_WB_MOTION = False
DT_DISPLAY = 0.05 # dt used to display the wb motion
PLOT = False
DISPLAY_PLOT = PLOT and True
SAVE_PLOT = PLOT and False

###  Settings for generate_contact_sequence
FORCE_STRAIGHT_LINE = False # DEBUG ONLY should be false

### Settings for centroidal script :
MU=0.5
GRAVITY = np.matrix([0,0,-9.81]).T
SOLVER_DT = 0.05 # hardcoded in timeOpt_configs files, must match this one ! 

# hardcoded height change between the init (goal) position from planning and the one given to the centroidal solver
COM_SHIFT_Z = 0.0
TIME_SHIFT_COM = 0.0
USE_WP_COST = True # use wp from the contact sequence in the cost function of timeopt

## Settings for end effector :
EFF_CHECK_COLLISION = True
WB_ABORT_WHEN_INVALID = True # stop wb script when detecting a collision and return the VALID part (before the phase with collision
WB_RETURN_INVALID = not WB_ABORT_WHEN_INVALID and False  # stop wb script when detecting a collision and return  the computed part of motion, incuding the last INVALID phase

##  Settings for whole body : 
YAW_ROT_GAIN = 1.
WB_VERBOSE = 0 # 0,1 or 2
WB_STOP_AT_EACH_PHASE = False # wait for user input between each phase
IK_dt = 0.001  # controler time step
IK_PRINT_N = 500  # print state of the problem every IK_PRINT_N time steps (if verbose = True)
CHECK_FINAL_MOTION = True
IK_store_centroidal = True
IK_store_zmp = True # need store_centroidal
IK_store_effector = True
IK_store_contact_forces = True


# check if method_type choosen are coherent : 
if not(contact_generation_method in contact_generation_method_available):
    raise ValueError("contact generation method must be choosed from : "+str(contact_generation_method_available))
if not(centroidal_initGuess_method in centroidal_initGuess_method_available):
    raise ValueError("centroidal initGuess method must be choosed from : "+str(centroidal_initGuess_method_available))
if not(centroidal_method in centroidal_method_available):
    raise ValueError("centroidal method must be choosed from : "+str(centroidal_method_available))
if not(wholebody_method in wholebody_method_available):
    raise ValueError("wholebody method must be choosed from : "+str(wholebody_method_available))
if not(end_effector_method in end_effector_method_available):
    raise ValueError("end effector method must be choosed from : "+str(end_effector_method_available))
if contact_generation_method == "none" and centroidal_method != "load" :
    raise ValueError("Cannot skip contact_generation phase if centroidal trajectory is not loaded from file")
if centroidal_method == "timeopt" and centroidal_initGuess_method != "geometric":
    raise ValueError("In current implementation of timeopt, the initGuess must be geometric (FIXME)")

# skip useless ethod when loading motion from file: 
if contact_generation_method == "load":
    SAVE_CS = False
if centroidal_method == "load":
    centroidal_initGuess_method = "none"
    SAVE_CS_COM = False
if wholebody_method == "load":
    contact_generation_method = "none"
    centroidal_initGuess_method = "none"
    centroidal_method= "load"
    EXPORT_NPZ = False

# import specific settings for the selected demo. This settings may override default ones.
import importlib
import sys
if len(sys.argv)<2 : 
    print "## WARNING : script called without specifying a demo config file (one of the file contained in mlp.demo_config)"
    print "## Available demo files : "
    configs_path = PKG_PATH+"/scripts/mlp/demo_configs"
    demos_list = os.listdir(configs_path)
    for f in demos_list:
        if f.endswith(".py") and not f.startswith("__") and not f.startswith("common") : 
            print f.rstrip(".py")
    print "## Some data will be missing, scripts may fails. (cfg.Robot will not exist)"
    #raise IOError("You must call this script with the name of the config file (one of the file contained in mlp.demo_config)")
else :  
    import argparse
    parser = argparse.ArgumentParser(description = "todo")
    parser.add_argument('demo_name',type=str,help="The name of the demo configuration file to load")
    args = parser.parse_args()
    DEMO_NAME = args.demo_name
    print "# Load demo config : ",DEMO_NAME
    # Import the module
    try :
        demo_cfg = importlib.import_module('mlp.demo_configs.'+DEMO_NAME)
    except ImportError, e:
        print "Cannot load config file '"+DEMO_NAME+"' from mlp.demo_config, error : "
        print e.message
        raise NameError("Cannot load config file '"+DEMO_NAME+"' from mlp.demo_config")
    # Determine a list of names to copy to the current name space
    names = getattr(demo_cfg, '__all__', [n for n in dir(demo_cfg) if not n.startswith('_')])
    # Copy those names into the current name space
    g = globals()
    for name in names:
        g[name] = getattr(demo_cfg, name)