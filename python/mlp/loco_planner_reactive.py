from mlp import LocoPlanner
from mlp.config import Config
from mlp.viewer.display_tools import disp_wb_pinocchio, initScenePinocchio, initScene, displaySteppingStones, hideSteppingStone
import argparse
import atexit
import subprocess
import time
import os, sys, traceback
import numpy as np
import eigenpy
import curves
from curves import SE3Curve, polynomial
from multicontact_api import ContactSequence, ContactPhase
import mlp.utils.cs_tools as tools
from multiprocessing import Process, Queue, Pipe, Value, Array, Lock
from ctypes import c_ubyte, c_bool
from queue import Empty as queue_empty
import pickle
from mlp.centroidal.n_step_capturability import zeroStepCapturability
import mlp.contact_sequence.sl1m as sl1m
from pinocchio import Quaternion
from hpp.corbaserver.rbprm.utils import ServerManager
import importlib
import logging
logging.basicConfig(format='[%(name)-12s] %(levelname)-8s: %(message)s')
logger = logging.getLogger("reactive-planning")
eigenpy.switchToNumpyArray()

MAX_PICKLE_SIZE = 10000 # maximal size (in byte) of the pickled representation of a contactPhase
DURATION_0_STEP = 3. # duration (in seconds) of the motion to stop when calling the 0 step capturability

def update_root_traj_timings(cs):
    for cp in cs.contactPhases:
        cp.root_t = SE3Curve(cp.root_t(cp.root_t.min()), cp.root_t(cp.root_t.max()), cp.timeInitial, cp.timeFinal)



class LocoPlannerReactive(LocoPlanner):

    def __init__(self, cfg):
        cfg.DISPLAY_CS = False
        cfg.DISPLAY_CS_STONES = True
        cfg.DISPLAY_SL1M_SURFACES = False
        cfg.DISPLAY_INIT_GUESS_TRAJ = False
        cfg.DISPLAY_WP_COST = False
        cfg.DISPLAY_COM_TRAJ = False
        cfg.DISPLAY_FEET_TRAJ = False
        cfg.DISPLAY_ALL_FEET_TRAJ = False
        cfg.DISPLAY_WB_MOTION = False
        cfg.centroidal_initGuess_method = "none" # not supported by this class yet
        cfg.ITER_DYNAMIC_FILTER = 0 # not supported by this class yet
        cfg.CHECK_FINAL_MOTION = False
        cfg.IK_SHOW_PROGRESS = False
        cfg.IK_store_centroidal = False  # c,dc,ddc,L,dL (of the computed wholebody motion)
        cfg.IK_store_zmp = False
        cfg.IK_store_effector = False
        cfg.IK_store_contact_forces = False
        cfg.IK_store_joints_derivatives = True
        cfg.IK_store_joints_torque = False
        cfg.EFF_CHECK_COLLISION = False
        cfg.contact_generation_method = "sl1m"
        cfg.Robot.DEFAULT_COM_HEIGHT += cfg.COM_SHIFT_Z
        cfg.COM_SHIFT_Z = 0.
        cfg.TIME_SHIFT_COM = 0.
        self.previous_connect_goal = cfg.DURATION_CONNECT_GOAL
        cfg.DURATION_CONNECT_GOAL = 0.
        super().__init__(cfg)
        self.cfg.TIMEOPT_CONFIG_FILE = "cfg_softConstraints_talos_lowgoal.yaml"
        self.generate_centroidal, self.CentroidalInputs, self.CentroidalOutputs = self.cfg.get_centroidal_method()
        self.generate_effector_trajectories, self.EffectorInputs, self.EffectorOutputs = \
            self.cfg.get_effector_initguess_method()
        self.generate_wholebody, self.WholebodyInputs, self.WholebodyOutputs = self.cfg.get_wholebody_method()
        # define members that will stores processes and queues
        self.process_centroidal = None
        self.process_wholebody = None
        self.process_viewer = None
        self.process_gepetto_gui = None
        self.pipe_cs_in = None
        self.pipe_cs_out = None
        self.pipe_cs_com_in = None
        self.pipe_cs_com_out = None
        self.queue_qt = None
        self.viewer_lock = Lock()
        self.last_phase_lock = Lock()
        self.last_phase_pickled = Array(c_ubyte, MAX_PICKLE_SIZE) # will contain the last contact phase send to the viewer
        self.last_phase = None
        self.stop_motion_flag = Value(c_bool)
        self.last_phase_flag = Value(c_bool) # true if the last phase have changed
        self.cfg.Robot.minDist = 0.7
        # initialize the guide planner class:
        self.client_hpp = None
        self.guide_planner = self.init_guide_planner()
        # initialize a fullBody rbprm object and a sl1m contact planning class
        self.fullBody, _ = initScene(cfg.Robot, cfg.ENV_NAME, context="fullbody")
        self.contact_planner = self.init_contact_planner()
        # Set up gepetto gui and a pinocchio robotWrapper with display
        self.robot = None
        self.gui = None
        self.init_viewer()


    def init_guide_planner(self):
        """
        Initialize an rbprm.AbstractPathPlanner class
        :return: an instance of the class
        """
        # the following script must produce a
        if hasattr(self.cfg, 'SCRIPT_ABSOLUTE_PATH'):
            scriptName = self.cfg.SCRIPT_ABSOLUTE_PATH
        else:
            scriptName = self.cfg.RBPRM_SCRIPT_PATH + "." + self.cfg.SCRIPT_PATH + '.' + self.cfg.DEMO_NAME
        scriptName += "_path"
        logger.warning("Run Guide script : %s", scriptName)
        module = importlib.import_module(scriptName)
        planner = module.PathPlanner("guide_planning")
        planner.init_problem()
        # greatly increase the number of loops of the random shortcut
        planner.ps.setParameter("PathOptimization/RandomShortcut/NumberOfLoops", 50)
        # force the base orientation to follow the direction of motion along the Z axis
        planner.ps.setParameter("Kinodynamic/forceYawOrientation", True)
        planner.q_init[:2] = [0, 0] # FIXME : defined somewhere ??
        planner.init_viewer(cfg.ENV_NAME)
        planner.init_planner()
        return planner

    def init_contact_planner(self):
        """
        Initialize aa class used to generate sl1m contact sequences
        :return: a sl1m instance
        """
        return None # class sl1m

    def init_viewer(self):
        """
        Build a pinocchio wrapper and a gepetto-gui instance and assign them as class members
        """
        subprocess.run(["killall", "gepetto-gui"])
        self.process_gepetto_gui = subprocess.Popen("gepetto-gui",
                                                    stdout=subprocess.PIPE,
                                                    stderr=subprocess.DEVNULL,
                                                    preexec_fn=os.setpgrp)
        atexit.register(self.process_gepetto_gui.kill)
        time.sleep(3)
        self.robot, self.gui = initScenePinocchio(cfg.Robot.urdfName + cfg.Robot.urdfSuffix,
                                                  cfg.Robot.packageName, cfg.ENV_NAME)
        self.robot.display(self.cfg.IK_REFERENCE_CONFIG)

    def plan_guide(self, root_goal):
        """
        Plan a guide from the current last_phase position to the given root position
        :param root_goal: a list of length 7 (translation + quaternion)
        :return: the Id of the new path
        """
        self.guide_planner.q_goal = self.guide_planner.q_init[::]
        self.guide_planner.q_goal[:7] = root_goal
        last_phase = self.get_last_phase()
        if last_phase:
            print("Last phase not None")
            print("Last phase q_final : ", last_phase.q_final[:7])
            self.guide_planner.q_init[:7] = last_phase.q_final[:7]
            self.guide_planner.q_init[2] = self.guide_planner.rbprmBuilder.ref_height # FIXME
        #TODO: add velocity
        print("Guide init = ", self.guide_planner.q_init)
        print("Guide goal = ", self.guide_planner.q_goal)
        self.guide_planner.ps.resetGoalConfigs()
        self.guide_planner.ps.clearRoadmap()
        self.guide_planner.solve()
        return self.guide_planner.ps.numberPaths() - 1

    def compute_cs_from_guide(self, guide_id):
        """
        Call SL1M to produce a contact sequence following the given root path
        Store the result in self.cs
        :param guide_id: Id of the path stored in self.guide_planner.ps
        :return:
        """
        initial_contacts = None
        last_phase = self.get_last_phase()
        if last_phase:
            q_init = None
            initial_contacts = [last_phase.contactPatch(loco_planner.fullBody.lfoot).placement.translation,
                                last_phase.contactPatch(loco_planner.fullBody.rfoot).placement.translation]
            first_phase = ContactPhase()
            tools.copyContactPlacement(last_phase, first_phase)
        else:
            first_phase = None
            q_init = self.fullBody.getCurrentConfig()
            initial_contacts = sl1m.initial_foot_pose_from_fullbody(self.fullBody, q_init)

        self.guide_planner.pathId = guide_id
        pathId, pb, coms, footpos, allfeetpos, res = sl1m.solve(self.guide_planner,
                                                           cfg.GUIDE_STEP_SIZE,
                                                           cfg.GUIDE_MAX_YAW,
                                                           cfg.MAX_SURFACE_AREA,
                                                           False,
                                                           initial_contacts)
        root_end = self.guide_planner.ps.configAtParam(pathId, self.guide_planner.ps.pathLength(pathId) - 0.001)[0:7]
        print("SL1M, root_end = ", root_end)

        self.cs = sl1m.build_cs_from_sl1m(self.fullBody, self.cfg.IK_REFERENCE_CONFIG, root_end, pb, sl1m.RF,
                                     allfeetpos, cfg.SL1M_USE_ORIENTATION, cfg.SL1M_USE_INTERPOLATED_ORIENTATION,
                                          q_init, first_phase)
        print("## Compute cs from guide done.")
        process_stones = Process(target=self.display_stones_lock)
        process_stones.start()
        atexit.register(process_stones.terminate)

    def display_stones_lock(self):
        self.viewer_lock.acquire()
        print("Display stepping stones ...")
        displaySteppingStones(self.cs, self.gui, "world",
                              self.cfg.Robot)  # FIXME: change world if changed in pinocchio ...
        print("Display done.")
        self.viewer_lock.release()
        return

    def hide_stones_lock(self):
        self.viewer_lock.acquire()
        print("Hide stepping stones ...")
        hideSteppingStone(self.gui)
        print("Hiding done.")
        self.viewer_lock.release()

    def get_last_phase(self):
        if self.last_phase is None or self.last_phase_flag.value:
            self.last_phase_lock.acquire()
            try:
                pic = bytes(self.last_phase_pickled)
                self.last_phase = pickle.loads(pic)
                self.last_phase_flag.value = False
            except:
                pass
            self.last_phase_lock.release()
        return self.last_phase

    def set_last_phase(self, last_phase):
        """
        set pickled last_phase data to last_phase_pickled shared memory
        :param last_phase:
        """
        arr = pickle.dumps(last_phase)
        self.last_phase_lock.acquire()
        if len(arr) >= MAX_PICKLE_SIZE:
            raise ValueError("In copy array: given array is too big, size = " + str(len(arr)))
        for i, el in enumerate(arr):
            self.last_phase_pickled[i] = el
        self.last_phase_flag.value = True
        self.last_phase_lock.release()

    def is_at_stop(self):
        """
        Return True if the last phase have a null CoM and joint velocities
        :return:
        """
        p = self.get_last_phase()
        if p:
            if not np.isclose(p.dc_final, np.zeros(3), atol = 1e-2).all():
                return False
            dq = p.dq_t(p.timeFinal)
            if not np.isclose(dq, np.zeros(dq.shape), atol = 1e-1).all():
                return False
        return True


    def compute_centroidal(self, cs, previous_phase,
                           last_iter=False):  # update the initial state with the data from the previous intermediate state:
        if previous_phase:
            tools.setInitialFromFinalValues(previous_phase, cs.contactPhases[0])

        if not self.CentroidalInputs.checkAndFillRequirements(cs, self.cfg, None):
            raise RuntimeError(
                "The current contact sequence cannot be given as input to the centroidal method selected.")
        cs_full = self.generate_centroidal(self.cfg, cs, None, None)
        # CentroidalOutputs.assertRequirements(cs_full)
        if last_iter:
            return cs_full, None
        else:
            cs_cut = ContactSequence(0)
            for i in range(3):
                cs_cut.append(cs_full.contactPhases[i])
            return cs_cut, cs_cut.contactPhases[1]

    def compute_wholebody(self, robot, cs_com,
                          last_q=None, last_v=None, last_iter=False):
        if not self.EffectorInputs.checkAndFillRequirements(cs_com, self.cfg, self.fullBody):
            raise RuntimeError(
                "The current contact sequence cannot be given as input to the end effector method selected.")
        cs_ref_full = self.generate_effector_trajectories(self.cfg, cs_com, self.fullBody)
        # EffectorOutputs.assertRequirements(cs_ref_full)
        if last_iter:
            cs_ref = cs_ref_full
            last_phase = cs_com.contactPhases[-1]
        else:
            cs_cut = ContactSequence()
            for i in range(2):
                cs_cut.append(cs_ref_full.contactPhases[i])
            cs_ref = cs_cut
            last_phase = cs_com.contactPhases[2]
            tools.setFinalFromInitialValues(last_phase, last_phase)
        if last_q is not None:
            cs_ref.contactPhases[0].q_init = last_q
        if last_v is not None:
            t_init = cs_ref.contactPhases[0].timeInitial
            cs_ref.contactPhases[0].dq_t = polynomial(last_v.reshape(-1, 1), t_init, t_init)

        ### Wholebody
        update_root_traj_timings(cs_ref)
        if not self.WholebodyInputs.checkAndFillRequirements(cs_ref, self.cfg, self.fullBody):
            raise RuntimeError(
                "The current contact sequence cannot be given as input to the wholeBody method selected.")
        cs_wb, robot = self.generate_wholebody(self.cfg, cs_ref, robot=robot)
        # print("-- compute whole body END")
        # WholebodyOutputs.assertRequirements(cs_wb)
        last_phase_wb = cs_wb.contactPhases[-1]
        last_q = last_phase_wb.q_t(cs_wb.contactPhases[-1].timeFinal)
        last_v = last_phase_wb.dq_t(cs_wb.contactPhases[-1].timeFinal)
        tools.deletePhaseCentroidalTrajectories(last_phase)
        last_phase.q_final = last_q
        last_phase.dq_t = polynomial(last_v.reshape(-1, 1), last_phase.timeFinal, last_phase.timeFinal)
        #last_phase.c_final = last_phase_wb.c_final
        #last_phase.dc_final = last_phase_wb.dc_final
        #last_phase.L_final = last_phase_wb.L_final
        return cs_wb, last_q, last_v, last_phase, robot

    def loop_centroidal(self):
        last_centroidal_phase = None
        last_iter = False
        try:
            while not last_iter:
                cs, last_iter = self.pipe_cs_out.recv()
                if last_iter:
                    self.cfg.DURATION_CONNECT_GOAL = self.previous_connect_goal
                    self.cfg.TIMEOPT_CONFIG_FILE = "cfg_softConstraints_talos.yaml"
                print("## Run centroidal")
                cs_com, last_centroidal_phase = self.compute_centroidal(cs, last_centroidal_phase, last_iter)
                if self.stop_motion_flag.value:
                    print("STOP MOTION in centroidal")
                    self.pipe_cs_com_in.close()
                    return
                print("-- Add a cs_com to the queue")
                self.pipe_cs_com_in.send([cs_com, last_iter])
            print("Centroidal last iter received, close the pipe and terminate process.")
        except:
            print("FATAL ERROR in loop centroidal: ")
            traceback.print_exc()
            sys.exit(0)
        self.pipe_cs_com_in.close()

    def loop_wholebody(self):
        last_q = None
        last_v = None
        robot = None
        last_iter = False
        try:
            while not last_iter:
                cs_com, last_iter = self.pipe_cs_com_out.recv()
                print("## Run wholebody")
                cs_wb, last_q, last_v, last_phase, robot = self.compute_wholebody(robot, cs_com, last_q, last_v, last_iter)
                if self.stop_motion_flag.value:
                    print("STOP MOTION in wholebody")
                    self.queue_qt.close()
                    return
                print("-- Add a cs_wb to the queue")
                self.queue_qt.put([cs_wb.concatenateQtrajectories(), last_phase, last_iter])
            print("Wholebody last iter received, close the pipe and terminate process.")
        except:
            print("FATAL ERROR in loop wholebody: ")
            traceback.print_exc()
            sys.exit(0)

    def loop_viewer(self):
        self.viewer_lock.acquire()
        last_iter = False
        try:
            while not last_iter:
                    q_t, last_phase, last_iter = self.queue_qt.get()
                    if last_phase:
                        self.set_last_phase(last_phase)
                    disp_wb_pinocchio(self.robot, q_t, cfg.DT_DISPLAY)
                    if self.stop_motion_flag.value:
                        print("STOP MOTION in viewer")
                        self.viewer_lock.release()
                        return
        except:
            print("FATAL ERROR in loop viewer: ")
            traceback.print_exc()
            sys.exit(0)
        self.viewer_lock.release()



    def stop_process(self):
        self.stop_motion_flag.value = True
        print("STOP MOTION flag sent")
        if self.pipe_cs_in:
            self.pipe_cs_in.close()
        if self.pipe_cs_out:
            self.pipe_cs_out.close()
        if self.pipe_cs_com_in:
            self.pipe_cs_com_in.close()
        if self.pipe_cs_com_out:
            self.pipe_cs_com_out.close()
        if self.queue_qt:
            self.queue_qt.close()
        """
        if self.process_centroidal:
            self.process_centroidal.join()
        if self.process_wholebody:
            self.process_wholebody.join()
        """

    def start_viewer_process(self):
        self.queue_qt = Queue()
        self.stop_motion_flag = Value(c_bool)
        self.stop_motion_flag.value = False
        self.process_viewer = Process(target=self.loop_viewer)
        self.process_viewer.start()
        atexit.register(self.process_viewer.terminate)



    def start_process(self):
        self.pipe_cs_out, self.pipe_cs_in = Pipe(False)
        self.pipe_cs_com_out, self.pipe_cs_com_in = Pipe(False)
        self.last_phase_pickled = Array(c_ubyte, MAX_PICKLE_SIZE)
        self.last_phase = None

        self.start_viewer_process()

        if self.process_centroidal:
            self.process_centroidal.terminate()
        self.process_centroidal = Process(target=self.loop_centroidal)
        self.process_centroidal.start()
        atexit.register(self.process_centroidal.terminate)
        if self.process_wholebody:
            self.process_wholebody.terminate()
        self.process_wholebody = Process(target=self.loop_wholebody)
        self.process_wholebody.start()
        atexit.register(self.process_wholebody.terminate)



    def compute_from_cs(self):
        pid_centroidal = 0
        last_iter_centroidal = False
        print("## Compute from cs,  size = ", self.cs.size())
        last_phase = self.get_last_phase()
        if last_phase:
            tools.setFinalFromInitialValues(last_phase, self.cs.contactPhases[0])
        while pid_centroidal + 5 < self.cs.size():
            #print("## Current pid = ", pid_centroidal)
            if pid_centroidal + 7 >= self.cs.size():
                #print("## Last centroidal iter")
                # last iter, take all the remaining phases
                num_phase = self.cs.size() - pid_centroidal
                last_iter_centroidal = True
            else:
                num_phase = 5
            #print("## Num phase = ", num_phase)
            # Extract the phases [pid_centroidal; pid_centroidal +num_phases] from cs_full
            cs_iter = ContactSequence(0)
            for i in range(pid_centroidal, pid_centroidal + num_phase):
                #print("-- Add phase : ", i)
                cs_iter.append(self.cs.contactPhases[i])
            self.pipe_cs_in.send([cs_iter, last_iter_centroidal])
            pid_centroidal += 2
        self.pipe_cs_in.close()


    def compute_cs_requirements(self):
        tools.computePhasesTimings(self.cs, self.cfg)
        tools.computePhasesCOMValues(self.cs, self.cfg.Robot.DEFAULT_COM_HEIGHT)
        tools.setAllUninitializedContactModel(self.cs, cfg.Robot)
        tools.computeRootTrajFromContacts(self.fullBody, self.cs)
        tools.setAllUninitializedFrictionCoef(self.cs, self.cfg.MU)

    #DEPRECATED
    def run(self):
        self.run_contact_generation()
        self.compute_cs_requirements()

        self.start_process()
        time.sleep(2)
        self.compute_from_cs()

    ## debug helper
    def stop_and_retry(self):
        self.stop_process()
        time.sleep(0.2)
        self.start_process()
        self.compute_from_cs()

    def compute_stopping_cs(self):
        # Compute a Centroidal reference to bring the current phase at a stop with a change of contact:
        phase_stop = ContactPhase(self.get_last_phase())
        tools.setInitialFromFinalValues(phase_stop, phase_stop)
        phase_stop.timeInitial = phase_stop.timeFinal
        phase_stop.duration = DURATION_0_STEP  # FIXME !!
        # try 0-step:
        success, phase = zeroStepCapturability(phase_stop, self.cfg)
        if success:
            cs_ref = ContactSequence(0)
            cs_ref.append(phase)
            # TEST : add another phase to go back in the center of the support polygon
            phase_projected = ContactPhase()
            phase_projected.timeInitial = phase.timeFinal
            phase_projected.duration = DURATION_0_STEP
            tools.copyContactPlacement(phase, phase_projected)
            tools.setInitialFromFinalValues(phase, phase_projected)
            phase_projected.c_final = tools.computeCenterOfSupportPolygonFromPhase(phase_stop, self.fullBody.DEFAULT_COM_HEIGHT)
            #FIXME 'default height'
            tools.connectPhaseTrajToFinalState(phase_projected)
            cs_ref.append(phase_projected)
        else:
            # TODO try 1 step :
            raise RuntimeError("One step capturability not implemented yet !")
        tools.computeRootTrajFromContacts(self.fullBody,cs_ref)
        self.last_phase = cs_ref.contactPhases[-1].copy()
        # define the final root position, translation from the CoM position and rotation from the feet rotation
        q_final = np.zeros(7)
        q_final[:3] = self.last_phase.c_final[::]
        placement_rot_root, _ = tools.rootOrientationFromFeetPlacement(self.cfg.Robot, None, self.last_phase, None)
        quat_root = Quaternion(placement_rot_root.rotation)
        q_final[3:7] = [quat_root.x, quat_root.y, quat_root.z, quat_root.w]
        self.last_phase.q_final = q_final
        self.last_phase_flag.value = False
        return cs_ref

    def run_zero_step_capturability(self):
        cs_ref = self.compute_stopping_cs()
        self.start_viewer_process()
        self.cfg.IK_dt = 0.02
        p = Process(target=self.generate_wholebody, args=(self.cfg, cs_ref, None, None, None, self.queue_qt))
        p.start()

    def stop_motion(self):
        self.stop_process()
        process_stones = Process(target=self.hide_stones_lock)
        process_stones.start()
        atexit.register(process_stones.terminate)
        if not self.is_at_stop():
            print("REQUIRE STOP MOTION: compute 0-step capturability")
            # Try 0 or one step capturability HERE
            self.run_zero_step_capturability()


    def move_to_goal(self, root_goal):
        """
        Plan and execute a motion connecting the current configuration to one with the given root position
        If the robot is in motion, start by computing a safe stop motion.
        :param root_goal: list of size 7: translation and quaternion for the desired root position
        :return:
        """
        self.stop_motion()
        guide_id = self.plan_guide(root_goal)
        print("Guide planning solved, path id = ", guide_id)
        self.compute_cs_from_guide(guide_id)
        self.compute_cs_requirements()
        print("Start process")

        self.start_process()
        time.sleep(2)
        self.compute_from_cs()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run a multicontact-locomotion-planning scenario")
    parser.add_argument('demo_name', type=str, help="The name of the demo configuration file to load. "
                                                    "Must be a valid python file, either inside the PYTHONPATH"
                                                    "or inside the mlp.demo_configs folder. ")
    args = parser.parse_args()
    demo_name = args.demo_name

    cfg = Config()
    cfg.load_scenario_config(demo_name)

    subprocess.run(["killall", "hpp-rbprm-server"])
    process_rbprm = subprocess.Popen("hpp-rbprm-server",
                                                stdout=subprocess.PIPE,
                                                stderr=subprocess.DEVNULL,
                                                preexec_fn=os.setpgrp)
    atexit.register(process_rbprm.terminate)
    time.sleep(3)
    loco_planner = LocoPlannerReactive(cfg)
    #loco_planner.run()



