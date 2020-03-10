import numpy as np
import os
from importlib import import_module


class Config:
    """
    Configuration class, used to defined all the different settings used by loco_planner
    """

    # possible choices for each subproblem
    contact_generation_method_available = ["load", "rbprm", "sl1m"]
    centroidal_initGuess_method_available = ["none", "geometric", "croc", "momentumopt", "quasistatic"]
    centroidal_method_available = ["load", "geometric", "croc", "momentumopt", "quasistatic", "muscod"]
    end_effector_initGuess_method_available = ["load","smoothedFoot", "bezierPredef"]
    end_effector_method_available = ["limbRRT", "limbRRToptimized"]
    wholebody_method_available = ["load", "tsid", "croccodyl"]
    simulator_available = ["pinocchioIntegration"]


    def __init__(self):
        ## methods setting : choose which method will be used to solve each subproblem :
        self.contact_generation_method = "rbprm"
        self.centroidal_initGuess_method = "none"
        self.centroidal_method = "momentumopt"
        self.end_effector_initGuess_method = "bezierPredef"
        self.end_effector_method = "limbRRToptimized"
        self.wholebody_method = "tsid"
        self.simulator_method = "pinocchioIntegration"

        ## PATHS settings :
        self.PKG_PATH = os.path.dirname(os.path.realpath(__file__)).rstrip("/python/mlp")
        print ("PKG_PATH = ",self.PKG_PATH)
        self.OUTPUT_DIR = self.PKG_PATH+"/res"
        self.CONTACT_SEQUENCE_PATH = self.OUTPUT_DIR + "/contact_sequences"
        self.TIME_OPT_CONFIG_PATH = self.PKG_PATH + '/momentumopt_configs'
        self.STATUS_FILENAME = self.OUTPUT_DIR + "/infos.log"
        self.EXPORT_PATH = self.OUTPUT_DIR + "/export"
        # if absolute script path is given for rbprm, this path is prepended
        self.RBPRM_SCRIPT_PATH = "hpp.corbaserver.rbprm.scenarios"

        ## Export setting
        self.SAVE_CS = True
        self.SAVE_CS_COM = True
        self.SAVE_CS_REF = True
        self.SAVE_CS_WB = True
        self.EXPORT_GAZEBO = False
        self.EXPORT_NPZ = False
        self.EXPORT_BLENDER = False
        self.EXPORT_SOT = False
        self.EXPORT_OPENHRP = False
        # if true : in the export_openHRP, use the zmp computed by the centroidal solver and the one computed from the wholebody
        self.openHRP_useZMPref = False
        self.WRITE_STATUS = False

        ##DISPLAY settings :
        self.DISPLAY_CS = False  # display contact sequence from rbprm
        self.DISPLAY_CS_STONES = True  # display stepping stones
        self.DISPLAY_INIT_GUESS_TRAJ = False
        # display waypoints found by the planner and used in the cost function of the centroidal dynamic solver
        self.DISPLAY_WP_COST = True
        self.DISPLAY_COM_TRAJ = True
        self.DISPLAY_FEET_TRAJ = True  # display the feet trajectories used in the final motion
        self.DISPLAY_ALL_FEET_TRAJ = True  # display all the trajectory used as reference, even the invalid ones
        self.DISPLAY_WB_MOTION = False  # display whole body motion automatically once it's computed
        # dt used to display the wb motion (one configuration every dt is displayed) It have to be greater than IK_dt
        self.DT_DISPLAY = 0.05
        self.PLOT = False  # Generate plot for various data
        # plot COM trajectory computed by the centroidal dynamic solver, before trying to compute the wholebody motion
        self.PLOT_CENTROIDAL = False
        self.DISPLAY_PLOT = self.PLOT and True  # display plot directly
        self.SAVE_PLOT = self.PLOT and True  #save plot as svg in OUTPUT_DIR/plot/demo_name_*

        ###  Settings for generate_contact_sequence
        self.FORCE_STRAIGHT_LINE = False  # DEBUG ONLY should be false
        self.SL1M_USE_ORIENTATION = True  # sl1m method use the root orientation computed by the guide planning
        # Only matter if SL1M_USE_ORIENTATION=True, if false sl1m method use exactly the orientation from planning,
        # if False, it interpolate the orientation and adapt it depending if the feet is in the inside or outside of the turn
        self.SL1M_USE_INTERPOLATED_ORIENTATION = True

        self.ITER_DYNAMIC_FILTER = 0  # if > 0 the solution of the wholebody method is send back to the centroidal solver

        ### Settings for centroidal script :
        self.GRAVITY = np.array([0, 0, -9.81])
        self.MU = 0.5  # Friction coefficient.
        self.SOLVER_DT = 0.05  # time step used for centroidal methods
        # Hardcoded height change of the COM before the beginning of the motion (value in m and time allowed to make this motion)
        # This is used because for some robot, the reference configuration is really close to the kinematic limits of the robot.
        self.COM_SHIFT_Z = 0.0
        self.TIME_SHIFT_COM = 0.0
        self.USE_WP_COST = True  # use wp from the contact sequence in the cost function of the centroidal solver

        ## Settings for end effector :
        self.EFF_CHECK_COLLISION = True  # After generating of whole body motion for a phase with an effector motion, check collision and joints limits for this motion and retry if invalid and if choosen method allow it
        self.CHECK_DT = 0.01  # time step (in seconds) at which the (self-)collision and joints limits are tested
        self.WB_ABORT_WHEN_INVALID = False  # stop wb script when stuck with an invalid motion and return the VALID part (before the phase with collision)
        self.WB_RETURN_INVALID = not self.WB_ABORT_WHEN_INVALID and True  # stop wb script when stuck with an invalid motion and return  the computed part of motion, incuding the last INVALID phase

        ##  Settings for whole body :
        self.YAW_ROT_GAIN = 1.  # gain for the orientation task of the root orientation, along the yaw axis (wrt to the other axis of the orientation task)
        self.IK_trackAM = False #If True, the Wb algorithm take the Angular momentum computed by te centroidal block as reference. If False it try to minimize the angular momentum
        self.WB_VERBOSE = 0  # 0,1 or 2 Verbosity level for the output of the wholebody script
        self.WB_STOP_AT_EACH_PHASE = False  # wait for user input between each phase
        self.IK_dt = 0.01  # controler time step (in second)
        self.IK_PRINT_N = 500  # print state of the problem every IK_PRINT_N time steps (if verbose >= 1)
        self.CHECK_FINAL_MOTION = True  # After computation of the motion, check the complete motion for {self-}collision and joints limits
        ### The following settings enable the computation of various values stored in the wholeBody_result struct.
        # Enabling them increase the computation time of the wholeBody script
        self.IK_store_centroidal = True  # c,dc,ddc,L,dL (of the computed wholebody motion)
        self.IK_store_zmp = True
        self.IK_store_effector = True
        self.IK_store_contact_forces = True
        self.IK_store_joints_derivatives = True
        self.IK_store_joints_torque = True

        self.check_methods()

    def load_scenario_config(self, demo_name):
        """
        import specific settings for the selected demo. This settings may override default ones.
        :param demo_name: 
        :return: 
        """

        demo_name = demo_name.rstrip(".py")  # remove extension if specified
        demo_name = demo_name.replace("/",".") # replace / to . for python path
        self.DEMO_NAME = demo_name.split(".")[-1] # only take the last part of the path if a full path is given
        print("# Load demo config : ", demo_name)
        # Import the module
        try:
            demo_cfg = import_module(demo_name)
        except ImportError as e:
            print("Cannot load config file '" + demo_name + "', error : ")
            print(e)
            print("Try to prepend path 'mlp.demo_configs.'")
            demo_name = "mlp.demo_configs." + demo_name
            try:
                demo_cfg = import_module(demo_name)
            except ImportError as e:
                print("Cannot load config file '" + demo_name + "', error : ")
                print(e)
                raise NameError("Cannot load config file '" + demo_name)

        # Determine a list of names to copy to the current name space
        names = getattr(demo_cfg, '__all__', [n for n in dir(demo_cfg) if not n.startswith('_')])
        # Copy those names into the current class
        for name in names:
            setattr(self, name, getattr(demo_cfg, name))
        self.CS_FILENAME = self.CONTACT_SEQUENCE_PATH + "/" + self.DEMO_NAME + ".cs"
        self.COM_FILENAME = self.CONTACT_SEQUENCE_PATH + "/" + self.DEMO_NAME + "_COM.cs"
        self.REF_FILENAME = self.CONTACT_SEQUENCE_PATH + "/" + self.DEMO_NAME + "_REF.cs"
        self.WB_FILENAME = self.CONTACT_SEQUENCE_PATH + "/" + self.DEMO_NAME + "_WB.cs"
        self.check_methods()



    def check_methods(self):
        # check if method_type choosen are coherent :
        if not (self.contact_generation_method in self.contact_generation_method_available):
            raise ValueError("contact generation method must be choosed from : " + str(self.contact_generation_method_available))
        if not (self.centroidal_initGuess_method in self.centroidal_initGuess_method_available):
            raise ValueError("centroidal initGuess method must be choosed from : " +
                             str(self.centroidal_initGuess_method_available))
        if not (self.centroidal_method in self.centroidal_method_available):
            raise ValueError("centroidal method must be choosed from : " + str(self.centroidal_method_available))
        if not (self.wholebody_method in self.wholebody_method_available):
            raise ValueError("wholebody method must be choosed from : " + str(self.wholebody_method_available))
        if not (self.end_effector_method in self.end_effector_method_available):
            raise ValueError("end effector method must be choosed from : " + str(self.end_effector_method_available))
        if not (self.end_effector_initGuess_method in self.end_effector_initGuess_method_available):
            raise ValueError("end effector method must be choosed from : " + str(self.end_effector_initGuess_method_available))
        if not (self.simulator_method in self.simulator_available):
            raise ValueError("Simulator method must be choosed from : " + str(self.simulator_available))


        # skip useless method when loading motion from file:
        if self.contact_generation_method == "load":
            self.SAVE_CS = False
        if self.centroidal_method == "load":
            self.contact_generation_method = "load"
            self.centroidal_initGuess_method = "none"
            self.SAVE_CS = False
            self.SAVE_CS_COM = False
            self.CS_FILENAME = self.COM_FILENAME
            self.ITER_DYNAMIC_FILTER = 0
        if self.end_effector_initGuess_method == "load":
            self.contact_generation_method = "load"
            self.centroidal_initGuess_method = "none"
            self.centroidal_method = "load"
            self.SAVE_CS = False
            self.SAVE_CS_COM = False
            self.SAVE_CS_REF = False
            self.CS_FILENAME = self.REF_FILENAME
            self.COM_FILENAME = self.REF_FILENAME
        if self.wholebody_method == "load":
            self.contact_generation_method = "load"
            self.centroidal_initGuess_method = "none"
            self.centroidal_method = "load"
            self.end_effector_initGuess_method = "load"
            self.SAVE_CS = False
            self.SAVE_CS_COM = False
            self.SAVE_CS_WB = False
            self.EXPORT_NPZ = False
            self.ITER_DYNAMIC_FILTER = 0
            if not os.path.isfile(self.REF_FILENAME):
                self.REF_FILENAME = self.WB_FILENAME
            self.CS_FILENAME = self.REF_FILENAME
            self.COM_FILENAME = self.REF_FILENAME


    def get_contact_generation_method(self):
        module = import_module('contact_sequence.'+self.contact_generation_method)
        method = getattr(module, 'generate_contact_sequence_'+self.contact_generation_method)
        Outputs = getattr(module, 'Outputs')
        return method, Outputs