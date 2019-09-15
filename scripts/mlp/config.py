import numpy as np
import os

contact_generation_method_available = ["none","load", "rbprm","lp"]
centroidal_initGuess_method_available = ["none", "geometric", "croc", "timeopt", "quasistatic"]
centroidal_method_available = ["load", "geometric", "croc", "timeopt", "quasistatic", "muscod"]
wholebody_method_available = ["load", "tsid", "croccodyl"]
end_effector_method_available = ["smoothedFoot", "bezierPredef", "bezierConstrained", "limbRRT", "limbRRToptimized"]

## methods setting : choose which method will be used to solve each subproblem : 
contact_generation_method = "rbprm" 
centroidal_initGuess_method = "geometric" 
centroidal_method = "timeopt" 
wholebody_method = "tsid" 
end_effector_method = "limbRRToptimized" 

## PATHS settings : 
PKG_PATH = "/local/dev/multicontact-locomotion-planning"
OUTPUT_DIR = PKG_PATH+"/res"
CONTACT_SEQUENCE_PATH = OUTPUT_DIR + "/contact_sequences"
TIME_OPT_CONFIG_PATH = PKG_PATH +'/timeOpt_configs'
STATUS_FILENAME = OUTPUT_DIR + "/infos.log"
EXPORT_PATH = OUTPUT_DIR+"/export"
## Export setting
SAVE_CS = True 
SAVE_CS_COM = True
EXPORT_GAZEBO = False
EXPORT_NPZ = True
EXPORT_BLENDER = False
EXPORT_SOT = False
EXPORT_OPENHRP = False
EXPORT_EFF_IN_CS=True
openHRP_useZMPref = False # if true : in the export_openHRP, use the zmp computed by the centroidal solver and the one computed from the wholebody
WRITE_STATUS = True
##DISPLAY settings : 
DISPLAY_CS = False # display contact sequence from rbprm
DISPLAY_CS_STONES = True # display stepping stones
DISPLAY_INIT_GUESS_TRAJ = False 
DISPLAY_WP_COST=True # display waypoints found by the planner and used in the cost function of the centroidal dynamic solver
DISPLAY_COM_TRAJ = True 
DISPLAY_FEET_TRAJ = True # display the feet trajectories used in the final motion
DISPLAY_ALL_FEET_TRAJ = False # display all the trajectory used as reference, even the invalid ones
DISPLAY_WB_MOTION = False # display whole body motion automatically once it's computed
DT_DISPLAY = 0.05 # dt used to display the wb motion (one configuration every dt is sent to the viewer) It have to be greater than IK_dt
PLOT = False # Generate plot for various data
PLOT_CENTROIDAL = False # plot COM trajectory computed by the centroidal dynamic solver, before trying to compute the wholebody motion
DISPLAY_PLOT = PLOT and True # display plot directly
SAVE_PLOT = PLOT and True #save plot as svg in OUTPUT_DIR/plot/demo_name_*

###  Settings for generate_contact_sequence
FORCE_STRAIGHT_LINE = False # DEBUG ONLY should be false

### Settings for centroidal script :
GRAVITY = np.matrix([0,0,-9.81]).T
MU=0.5 # Friction coefficient. hardcoded in timeOpt_configs files, must match this one ! 
SOLVER_DT = 0.05 # hardcoded in timeOpt_configs files, must match this one ! 
# Hardcoded height change of the COM before the beginning of the motion (value in m and time allowed to make this motion)
# This is used because for some robot, the reference configuration is really close to the kinematic limits of the robot. 
COM_SHIFT_Z = 0.0
TIME_SHIFT_COM = 0.0
USE_WP_COST = True # use wp from the contact sequence in the cost function of the centroidal solver

## Settings for end effector :
EFF_CHECK_COLLISION = True # After generating of whole body motion for a phase with an effector motion, check collision and joints limits for this motion and retry if invalid and if choosen method allow it
WB_ABORT_WHEN_INVALID = False # stop wb script when stuck with an invalid motion and return the VALID part (before the phase with collision)
WB_RETURN_INVALID = not WB_ABORT_WHEN_INVALID and True  # stop wb script when stuck with an invalid motion and return  the computed part of motion, incuding the last INVALID phase

##  Settings for whole body : 
YAW_ROT_GAIN = 1. # gain for the orientation task of the root orientation, along the yaw axis (wrt to the other axis of the orientation task)
USE_PLANNING_ROOT_ORIENTATION = True # if true, the reference for the root orientation is the one given by the planning (stored in phase.reference_configurations) if false, use the one of q_init for all the motion.
WB_VERBOSE = 0 # 0,1 or 2 Verbosity level for the output of the wholebody script
WB_STOP_AT_EACH_PHASE = False # wait for user input between each phase
IK_dt = 0.001  # controler time step (in second)
IK_PRINT_N = 500  # print state of the problem every IK_PRINT_N time steps (if verbose >= 1)
CHECK_FINAL_MOTION = True # After computation of the motion, check the complete motion for {self-}collision and joints limits
### The following settings enable the computation of various values stored in the wholeBody_result struct. 
# Enabling them increase the computation time of the wholeBody script
IK_store_centroidal = True # c,dc,ddc,L,dL (of the computed wholebody motion)
IK_store_zmp = True # need store_centroidal
IK_store_effector = True 
IK_store_contact_forces = True


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
    DEMO_NAME = DEMO_NAME.rstrip(".py") # remove extension if specified
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
    contact_generation_method = "load"
    centroidal_initGuess_method = "none"
    SAVE_CS = False    
    SAVE_CS_COM = False
if wholebody_method == "load":
    contact_generation_method = "load"
    centroidal_initGuess_method = "none"
    centroidal_method= "load"
    SAVE_CS = False    
    SAVE_CS_COM = False    
    EXPORT_NPZ = False
