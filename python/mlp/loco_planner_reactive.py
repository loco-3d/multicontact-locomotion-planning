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
import ndcurves as curves
import math
from ndcurves import SE3Curve, polynomial
from multicontact_api import ContactSequence, ContactPhase
import mlp.utils.cs_tools as tools
from multiprocessing import Process, Queue, Pipe, Value, Array, Lock
from ctypes import c_ubyte, c_bool
from queue import Empty as queue_empty
import pickle
from mlp.centroidal.n_step_capturability import zeroStepCapturability
import mlp.contact_sequence.sl1m as sl1m
from pinocchio import Quaternion
import importlib
import logging
logging.basicConfig(format='[%(name)-12s] %(levelname)-8s: %(message)s')
logger = logging.getLogger("reactive-planning")
logger.setLevel(logging.ERROR)  #DEBUG, INFO or WARNING
eigenpy.switchToNumpyArray()

MAX_PICKLE_SIZE = 10000  # maximal size (in byte) of the pickled representation of a contactPhase
DURATION_0_STEP = 3.  # duration (in seconds) of the motion to stop when calling the 0 step capturability
V_INIT = 0.05  # small velocity added to the init/goal guide config to force smooth orientation change
V_GOAL = 0.1
SCALE_OBSTACLE_COLLISION = 0.1  # value added to the size of the collision obstacles manually added
TIMEOUT_CONNECTIONS = 10.  # the time (in second) after which a process close if it did not receive new data

USE_PYB = True
if USE_PYB:
    from solo_impedance_control.pyb_solo_simulator import pybullet_simulator, SimulatorLoop


def update_root_traj_timings(cs):
    """
    Update all the root_t trajectories of all contact phases of the given CS.
    The new trajectories are linear interpolation from the initial to the final placement,
    with a time definition matching the time definition of the contact phase
    :param cs: The contact sequence to update
    """
    for cp in cs.contactPhases:
        cp.root_t = SE3Curve(cp.root_t(cp.root_t.min()), cp.root_t(cp.root_t.max()), cp.timeInitial, cp.timeFinal)


class LocoPlannerReactive(LocoPlanner):
    def __init__(self, cfg):
        # Disable most of the automatic display that could not be updated automatically when motion is replanned
        cfg.DISPLAY_CS = False
        cfg.DISPLAY_CS_STONES = True
        cfg.DISPLAY_SL1M_SURFACES = False
        cfg.DISPLAY_INIT_GUESS_TRAJ = False
        cfg.DISPLAY_WP_COST = False
        cfg.DISPLAY_COM_TRAJ = False
        cfg.DISPLAY_FEET_TRAJ = False
        cfg.DISPLAY_ALL_FEET_TRAJ = False
        cfg.DISPLAY_WB_MOTION = False
        cfg.IK_SHOW_PROGRESS = False
        cfg.centroidal_initGuess_method = "none"  # not supported by this class yet
        cfg.ITER_DYNAMIC_FILTER = 0  # not supported by this class yet
        cfg.CHECK_FINAL_MOTION = False
        # Disable computation of additionnal data to speed up the wholebody computation time
        cfg.IK_store_centroidal = False  # c,dc,ddc,L,dL (of the computed wholebody motion)
        cfg.IK_store_zmp = False
        cfg.IK_store_effector = False
        cfg.IK_store_contact_forces = False
        cfg.IK_store_joints_derivatives = True
        cfg.IK_store_joints_torque = False
        cfg.EFF_CHECK_COLLISION = False  # WIP: disable limb-rrt
        cfg.contact_generation_method = "sl1m"  # Reactive planning class is specific to SL1M for now
        # Store the specific settings for connecting the initial/goal points as they may be changed
        cfg.Robot.DEFAULT_COM_HEIGHT += cfg.COM_SHIFT_Z
        self.previous_com_shift_z = cfg.COM_SHIFT_Z
        self.previous_time_shift_com = cfg.TIME_SHIFT_COM
        cfg.COM_SHIFT_Z = 0.
        cfg.TIME_SHIFT_COM = 0.
        self.previous_connect_goal = cfg.DURATION_CONNECT_GOAL
        cfg.DURATION_CONNECT_GOAL = 0.
        self.TIMEOPT_CONFIG_FILE = cfg.TIMEOPT_CONFIG_FILE
        super().__init__(cfg)
        # Get the centroidal and wholebody methods selected in the configuraton file
        self.generate_centroidal, self.CentroidalInputs, self.CentroidalOutputs = self.cfg.get_centroidal_method()
        self.generate_effector_trajectories, self.EffectorInputs, self.EffectorOutputs = \
            self.cfg.get_effector_initguess_method()
        self.generate_wholebody, self.WholebodyInputs, self.WholebodyOutputs = self.cfg.get_wholebody_method()
        # define members that will stores processes and queues
        self.process_compute_cs = None
        self.process_centroidal = None
        self.process_wholebody = None
        self.process_viewer = None
        self.process_gepetto_gui = None
        self.pipe_cs_in = None
        self.pipe_cs_out = None
        self.pipe_cs_com_in = None
        self.pipe_cs_com_out = None
        self.queue_qt = None
        self.viewer_lock = Lock()  # lock to access gepetto-gui API
        self.loop_viewer_lock = Lock()  # lock to guarantee that only one loop_viewer is executed at any given time
        self.last_phase_lock = Lock()  # lock to read/write last_phase data
        self.last_phase_pickled = Array(c_ubyte,
                                        MAX_PICKLE_SIZE)  # will contain the last contact phase send to the viewer
        self.last_phase = None
        self.stop_motion_flag = Value(c_bool)  # true if a stop is requested
        self.last_phase_flag = Value(c_bool)  # true if the last phase have changed
        self.cfg.Robot.minDist = 0.7  # See bezier-com-traj doc,
        # minimal distance between the CoM and the contact points along the Z axis
        # initialize the guide planner class:
        self.client_hpp = None
        self.robot = None
        self.gui = None
        self.pyb_sim = None
        self.guide_planner = self.init_guide_planner()
        # initialize a fullBody rbprm object
        self.fullBody, _ = initScene(cfg.Robot, cfg.ENV_NAME, context="fullbody")
        self.fullBody.setCurrentConfig(cfg.IK_REFERENCE_CONFIG.tolist() + [0] * 6)
        self.current_root_goal = []
        self.current_guide_id = 0
        # Set up gepetto gui and a pinocchio robotWrapper with display
        self.init_viewer()
        # Set up a pybullet environment:
        if USE_PYB:
            self.init_pybullet()

    def init_guide_planner(self):
        """
        Initialize an rbprm.AbstractPathPlanner class
        :return: an instance of rbprm.AbstractPathPlanner initialized with the current settings
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
        planner.q_init[:2] = [0, 0]  # FIXME : defined somewhere ??
        planner.init_viewer(cfg.ENV_NAME)
        planner.init_planner()
        return planner

    def init_viewer(self):
        """
        Build a pinocchio wrapper and a gepetto-gui instance and assign them as class members:
        self.robot and self.gui
        Also start (or restart) a gepetto-gui process
        """
        subprocess.run(["killall", "gepetto-gui"])
        self.process_gepetto_gui = subprocess.Popen("gepetto-gui",
                                                    stdout=subprocess.PIPE,
                                                    stderr=subprocess.DEVNULL,
                                                    preexec_fn=os.setpgrp)
        atexit.register(self.process_gepetto_gui.kill)
        time.sleep(3)
        self.robot, self.gui = initScenePinocchio(cfg.Robot.urdfName + cfg.Robot.urdfSuffix, cfg.Robot.packageName,
                                                  cfg.ENV_NAME)
        self.robot.display(self.cfg.IK_REFERENCE_CONFIG)

    def init_pybullet(self):
        self.pyb_sim = pybullet_simulator(dt=self.cfg.IK_dt,
                                          q_init=self.cfg.IK_REFERENCE_CONFIG[7:].reshape(-1, 1),
                                          root_init=[0, 0, 0.1],
                                          env_name=cfg.ENV_NAME + ".urdf",
                                          env_package="hpp_environments",
                                          use_gui=False)

    def execute_motion(self, q_t, dq_t):
        if USE_PYB:
            self.viewer_lock.acquire()
            SimulatorLoop(self.pyb_sim, self.cfg.IK_dt, q_t, dq_t, self.robot.display)
            self.viewer_lock.release()
        else:
            self.viewer_lock.acquire()
            disp_wb_pinocchio(self.robot, q_t, self.cfg.DT_DISPLAY)
            self.viewer_lock.release()



    def plan_guide(self, root_goal):
        """
        Plan a guide from the current last_phase root position to the given root position
        :param root_goal: list of size 3 or 7: translation and quaternion for the desired root position
        If the quaternion part is not specified, the final orientation is not constrained
        Store the Id of the new path in self.current_guide_id
        """
        self.guide_planner.q_goal = self.guide_planner.q_init[::]
        self.guide_planner.q_goal[:3] = root_goal[:3]
        self.guide_planner.q_goal[3:7] = [0, 0, 0, 1]
        self.guide_planner.q_goal[-6:] = [0] * 6
        last_phase = self.get_last_phase()
        if last_phase:
            logger.warning("Last phase not None")
            logger.warning("Last phase q_final : %s", last_phase.q_final[:7])
            self.guide_planner.q_init[:7] = last_phase.q_final[:7]
            self.guide_planner.q_init[2] = self.guide_planner.rbprmBuilder.ref_height  # FIXME
        #add small velocity in order to have smooth change of orientation at the beginning/end
        quat_init = Quaternion(self.guide_planner.q_init[6], self.guide_planner.q_init[3],
                               self.guide_planner.q_init[4], self.guide_planner.q_init[5])
        dir_init = quat_init * np.array([1, 0, 0])
        self.guide_planner.q_init[-6] = dir_init[0] * V_INIT
        self.guide_planner.q_init[-5] = dir_init[1] * V_INIT
        if len(root_goal) >= 7:
            self.guide_planner.q_goal[3:7] = root_goal[3:7]
            quat_goal = Quaternion(self.guide_planner.q_goal[6], self.guide_planner.q_goal[3],
                                   self.guide_planner.q_goal[4], self.guide_planner.q_goal[5])
            dir_goal = quat_goal * np.array([1, 0, 0])
            self.guide_planner.q_goal[-6] = dir_goal[0] * V_GOAL
            self.guide_planner.q_goal[-5] = dir_goal[1] * V_GOAL
        logger.warning("Guide init = %s", self.guide_planner.q_init)
        logger.warning("Guide goal = %s", self.guide_planner.q_goal)
        self.guide_planner.ps.resetGoalConfigs()
        self.guide_planner.ps.clearRoadmap()
        self.current_root_goal = root_goal
        self.guide_planner.solve(True)
        self.current_guide_id = self.guide_planner.ps.numberPaths() - 1

    def compute_cs_from_guide(self):
        """
        Call SL1M to produce a contact sequence following the root path stored in self.current_guide_id
        Store the result in self.cs
        :param guide_id: Id of the path stored in self.guide_planner.ps
        :return:
        """
        initial_contacts = None
        last_phase = self.get_last_phase()


        if self.cfg.SL1M_MAX_STEP > 0:
            # compute the pathlength corresponding to this number of steps:
            max_path_length = self.cfg.SL1M_MAX_STEP * self.cfg.GUIDE_STEP_SIZE
            total_path_length = self.guide_planner.ps.pathLength(self.current_guide_id)
            if total_path_length > max_path_length:
                # split the guide path in several segment of max_path_length length
                guide_ids = []
                t = 0.
                while t < total_path_length:
                    t_next = t + max_path_length
                    if t_next > total_path_length:
                        t_next = total_path_length
                    self.guide_planner.ps.extractPath(self.current_guide_id, t, t_next)
                    guide_ids += [self.guide_planner.ps.numberPaths() - 1]
                    t = t_next
            else:
                guide_ids = [self.current_guide_id]
        else:
            guide_ids = [self.current_guide_id]
        print("#####  compute sl1m from guide id(s) : ", guide_ids)

        self.cs = ContactSequence()
        for guide_id in guide_ids:
            self.guide_planner.pathId = guide_id
            self.guide_planner.q_goal = self.guide_planner.ps.configAtParam(
                guide_id, self.guide_planner.ps.pathLength(guide_id))
            print("### FORLOOP, guide id = ", guide_id)
            # compute initial contacts position, either from last_phase or from the wholebody configuration
            if last_phase:
                q_init = None
                initial_contacts = [last_phase.contactPatch(ee_name).placement.translation
                                    + self.fullBody.dict_offset[ee_name].translation
                                    for ee_name in
                                    [self.fullBody.dict_limb_joint[limb] for limb in self.fullBody.limbs_names]]
                first_phase = ContactPhase()
                tools.copyContactPlacement(last_phase, first_phase)
                tools.setInitialFromFinalValues(last_phase, first_phase)
            else:
                first_phase = None
                q_init = self.fullBody.getCurrentConfig()
                initial_contacts = sl1m.initial_foot_pose_from_fullbody(self.fullBody, q_init)

            pathId, pb, coms, footpos, allfeetpos, res = sl1m.solve(self.guide_planner,
                                                                    self.cfg,
                                                                    False,
                                                                    initial_contacts)
            root_end = self.guide_planner.ps.configAtParam(pathId, self.guide_planner.ps.pathLength(pathId) - 0.001)[0:7]
            logger.info("SL1M, root_end = %s", root_end)

            current_cs = sl1m.build_cs_from_sl1m(self.cfg.SL1M_USE_MIP,
                                                 self.fullBody,
                                                 self.cfg.IK_REFERENCE_CONFIG,
                                                 root_end,
                                                 pb,
                                                 sl1m.sl1m.RF,
                                                 allfeetpos,
                                                 cfg.SL1M_USE_ORIENTATION,
                                                 cfg.SL1M_USE_INTERPOLATED_ORIENTATION,
                                                 q_init,
                                                 first_phase)
            last_phase = current_cs.contactPhases[-1]
            # Merge current_cs in cs
            if self.cs.size() == 0:
                [self.cs.append(phase) for phase in current_cs.contactPhases]
            else:
                # first new phase is the same as last previous phase
                [self.cs.append(phase) for phase in current_cs.contactPhases[1:]]

        logger.warning("## Compute cs from guide done.")
        process_stones = Process(target=self.display_stones_lock)
        process_stones.start()
        atexit.register(process_stones.terminate)

    def display_stones_lock(self):
        """
        Wait for the lock to access to gepetto-gui and then display the stepping stones for the current contact_sequence
        """
        logger.info("Waiting lock to display stepping stones ...")
        self.viewer_lock.acquire()
        logger.info("Display stepping stones ...")
        displaySteppingStones(self.cs, self.gui, "world",
                              self.cfg.Robot)  # FIXME: change world if changed in pinocchio ...
        logger.info("Display done.")
        self.viewer_lock.release()

    def hide_stones_lock(self):
        """
        Wait for the lock to access to gepetto-gui and then remove all the stepping stones from the scene
        """
        logger.info("Waiting lock to hide stepping stones ...")
        self.viewer_lock.acquire()
        logger.info("Hide stepping stones ...")
        hideSteppingStone(self.gui)
        logger.info("Hiding done.")
        self.viewer_lock.release()

    def get_last_phase(self):
        """
        Retrieve the last phase stored in the shared memory
        If self.last_phase exist and the flag last_phase_flag is False, return the phase stored in self.last_phase
        Otherwise, retrieve the data in the shared memory and deserialize it
        :return: the last phase
        """
        if self.last_phase is None or self.last_phase_flag.value:
            self.last_phase_lock.acquire()
            try:
                pic = bytes(self.last_phase_pickled)
                self.last_phase = pickle.loads(pic)
                self.last_phase_flag.value = False
            except:
                #logger.error("Cannot de serialize the last_phase")
                self.last_phase = None
            self.last_phase_lock.release()
        return self.last_phase

    def set_last_phase(self, last_phase):
        """
        set pickled last_phase data to last_phase_pickled shared memory
        :param last_phase:
        """
        logger.info("set_last_phase: pickle serialization ...")
        arr = pickle.dumps(last_phase)
        logger.info("set_last_phase: waiting for lock ...")
        self.last_phase_lock.acquire()
        if len(arr) >= MAX_PICKLE_SIZE:
            raise ValueError("In copy array: given array is too big, size = " + str(len(arr)))
        logger.info("set_last_phase: writing data ... ")
        for i, el in enumerate(arr):
            self.last_phase_pickled[i] = el
        self.last_phase_flag.value = True
        logger.info("Add a last_phase, q_final = %s", last_phase.q_final)
        self.last_phase_lock.release()

    def is_at_stop(self):
        """
        Return True if the last phase have a null CoM and joint velocities
        :return:
        """
        p = self.get_last_phase()
        if p:
            if not np.isclose(p.dc_final, np.zeros(3), atol=1e-2).all():
                return False
            dq = p.dq_t(p.timeFinal)
            if not np.isclose(dq, np.zeros(dq.shape), atol=1e-1).all():
                return False
        return True

    def compute_centroidal(self, cs, previous_phase, last_iter=False):
        """
        Solve the centroidal problem for the given ContactSequence
        :param cs: the ContactSequence used
        :param previous_phase: If provided, copy the final data of this phase as initial data for
         the given ContactSequence
        :param last_iter: If True, return a ContactSequence corresponding to the complete contactSequence given as input
        If False, the result is splitted and only the first 3 phases are returned
        :return: The ContactSequence with centroidal trajectories, and the last phase
        """
        # update the initial state with the data from the previous intermediate state:
        if previous_phase:
            tools.setInitialFromFinalValues(previous_phase, cs.contactPhases[0])
            self.cfg.COM_SHIFT_Z = 0.
            self.cfg.TIME_SHIFT_COM = 0.
        #else:
        #    self.cfg.COM_SHIFT_Z = self.previous_com_shift_z
        #    self.cfg.TIME_SHIFT_COM = self.previous_time_shift_com
        if last_iter:
            # Set settings specific to the last iteration that need to connect exactly to the final goal position
            self.cfg.DURATION_CONNECT_GOAL = self.previous_connect_goal
            self.cfg.TIMEOPT_CONFIG_FILE = self.TIMEOPT_CONFIG_FILE
        else:
            # Set settings for the middle of the sequence: do not need to connect exactly to the goal
            self.cfg.DURATION_CONNECT_GOAL = 0.
            self.cfg.TIMEOPT_CONFIG_FILE = self.TIMEOPT_CONFIG_FILE.rstrip(".yaml") + "_lowgoal.yaml"
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

    def compute_wholebody(self, robot, cs_com, last_q=None, last_v=None, last_iter=False):
        """
        Compute the wholebody motion for the given ContactSequence
        :param robot: a TSID RobotWrapper instance
        :param cs_com: a ContactSequence with centroidal trajectories
        :param last_q: the last wholebody configuration (used as Initial configuration for this iteration)
        :param last_v:  the last joint velocities vector (used as initial joint velocities for this iteration)
        :param last_iter: if True, the complete ContactSequence is used, if False only the first 2 phases are used
        :return: a ContactSequence with wholebody data, the last wholebody configuration, the last joint velocity,
        the last phase, the TSID RobotWrapper used
        """
        if not self.EffectorInputs.checkAndFillRequirements(cs_com, self.cfg, self.fullBody):
            raise RuntimeError(
                "The current contact sequence cannot be given as input to the end effector method selected.")
        # Generate end effector trajectories for the contactSequence
        cs_ref_full = self.generate_effector_trajectories(self.cfg, cs_com, self.fullBody)
        # EffectorOutputs.assertRequirements(cs_ref_full)
        # Split contactSequence if it is not the last iteration
        if last_iter:
            cs_ref = cs_ref_full
            last_phase = cs_com.contactPhases[-1]
        else:
            cs_cut = ContactSequence()
            for i in range(2):
                cs_cut.append(cs_ref_full.contactPhases[i])
            cs_ref = cs_cut
            # last_phase should be a double support phase, it should contains all the contact data:
            last_phase = cs_com.contactPhases[2]
            tools.setFinalFromInitialValues(last_phase, last_phase)
        if last_q is not None:
            cs_ref.contactPhases[0].q_init = last_q
        if last_v is not None:
            t_init = cs_ref.contactPhases[0].timeInitial
            cs_ref.contactPhases[0].dq_t = polynomial(last_v.reshape(-1, 1), t_init, t_init)

        ### Generate the wholebody trajectory:
        update_root_traj_timings(cs_ref)
        if not self.WholebodyInputs.checkAndFillRequirements(cs_ref, self.cfg, self.fullBody):
            raise RuntimeError(
                "The current contact sequence cannot be given as input to the wholeBody method selected.")
        cs_wb, robot = self.generate_wholebody(self.cfg, cs_ref, robot=robot)
        logger.info("-- compute whole body END")
        # WholebodyOutputs.assertRequirements(cs_wb)
        # Retrieve the last phase, q, and v from this outputs:
        last_phase_wb = cs_wb.contactPhases[-1]
        last_q = last_phase_wb.q_t(last_phase_wb.timeFinal)
        last_v = last_phase_wb.dq_t(last_phase_wb.timeFinal)
        tools.deletePhaseCentroidalTrajectories(last_phase)  # Remove unnecessary data to reduce serialized size
        last_phase.q_final = last_q
        last_phase.dq_t = polynomial(last_v.reshape(-1, 1), last_phase.timeFinal, last_phase.timeFinal)
        #last_phase.c_final = last_phase_wb.c_final
        #last_phase.dc_final = last_phase_wb.dc_final
        #last_phase.L_final = last_phase_wb.L_final
        return cs_wb, last_q, last_v, last_phase, robot

    def compute_wholebody_queue(self, cs_ref):
        """
        Call the wholebody motion generation with the given cs_ref and the queue_qt
        and store the last compute phase with self.set_last_phase
        :param cs_ref:
        :return:
        """
        logger.warning("@@ Start compute_wholebody_queue")
        cs_wb, _ = self.generate_wholebody(self.cfg, cs_ref, None, None, None, self.queue_qt)
        last_phase = ContactPhase(cs_wb.contactPhases[-1])
        tools.deletePhaseTrajectories(last_phase)
        tools.deleteEffectorsTrajectories(last_phase)
        last_phase.root_t = cs_ref.contactPhases[-1].root_t
        self.set_last_phase(last_phase)
        logger.warning("@@ End compute_wholebody_queue")

    def loop_centroidal(self):
        """
        Loop waiting for data in pipe_cs, solving the centroidal problem for each new data and send the results
        in pipe_cs_com
        """
        last_centroidal_phase = None
        last_iter = False
        timeout = False
        try:
            while not last_iter and not timeout:
                if self.pipe_cs_out.poll(TIMEOUT_CONNECTIONS):
                    cs, last_iter = self.pipe_cs_out.recv()
                    logger.info("## Run centroidal")
                    cs_com, last_centroidal_phase = self.compute_centroidal(cs, last_centroidal_phase, last_iter)
                    logger.info("-- Add a cs_com to the queue")
                    self.pipe_cs_com_in.send([cs_com, last_iter])
                else:
                    timeout = True
                    logger.warning("Loop centroidal closed because pipe is empty since 10 seconds")
            if last_iter:
                logger.info("Centroidal last iter received, close the pipe and terminate process.")
        except:
            logger.error("FATAL ERROR in loop centroidal: ")
            traceback.print_exc()
            sys.exit(0)
        self.pipe_cs_com_in.close()

    def loop_wholebody(self):
        """
        Loop waiting for data in pipe_cs_com, computing the wholebody motion for each new data and sending the
        results in queue_qt
        """
        last_v = None
        robot = None
        last_iter = False
        timeout = False
        # Set the current config, either from the planned ContactSequence or from the data stored in last_phase
        last_q = self.cs.contactPhases[0].q_init
        if last_q is None or last_q.shape[0] < self.robot.nq:
            logger.info("initial config not defined in CS, set it from last phase.")
            # get current last_phase config:
            last_phase = self.get_last_phase()
            if logger.isEnabledFor(logging.INFO) and last_phase:
                logger.info("last_phase.q_final shape: %d", last_phase.q_final.shape[0])
            while last_phase is None or last_phase.q_final.shape[0] < self.robot.nq:
                # Wait for the data to be updated by another process
                last_phase = self.get_last_phase()
            last_q = last_phase.q_final
            logger.info("Got last_q from last_phase, start wholebody loop ...")
            logger.info("last_q in loop_wholebody = %s", last_q)
        try:
            while not last_iter and not timeout:
                if self.pipe_cs_com_out.poll(TIMEOUT_CONNECTIONS):
                    cs_com, last_iter = self.pipe_cs_com_out.recv()
                    logger.info("## Run wholebody")
                    cs_wb, last_q, last_v, last_phase, robot = self.compute_wholebody(
                        robot, cs_com, last_q, last_v, last_iter)
                    logger.info("-- Add a cs_wb to the queue")
                    self.queue_qt.put([cs_wb.concatenateQtrajectories(), cs_wb.concatenateDQtrajectories(), last_phase, last_iter])
                else:
                    timeout = True
                    logger.warning("Loop wholebody closed because pipe is empty since 10 seconds")
            if last_iter:
                logger.info("Wholebody last iter received, close the pipe and terminate process.")
        except:
            logger.error("FATAL ERROR in loop wholebody: ")
            traceback.print_exc()
            sys.exit(0)

    def loop_viewer(self):
        """
        Loop waiting for data in queue_qt and displaying each new trajectories.
        Before displaying each new data, it store the new last_phase in shared memory, this phase correspond to the last
        configuration and contacts that will be displayed for the current iteration.
        It watch for the "stop_motion" flag, if received the loop stop at the end of the current iteration
        """
        self.loop_viewer_lock.acquire()
        logger.warning("## Start a loop_viewer")
        self.stop_motion_flag.value = False
        last_iter = False
        timeout = TIMEOUT_CONNECTIONS
        try:
            while not last_iter:
                q_t, dq_t, last_phase, last_iter = self.queue_qt.get(timeout=timeout)
                timeout = 0.1
                if last_phase:
                    self.set_last_phase(last_phase)
                self.execute_motion(q_t, dq_t)
                if self.stop_motion_flag.value:
                    logger.info("STOP MOTION in viewer")
                    last_iter = True
        except queue_empty:
            logger.warning("Loop viewer closed because queue is empty since 10 seconds")
        except:
            logger.error("FATAL ERROR in loop viewer: ")
            traceback.print_exc()
            sys.exit(0)
        self.queue_qt.close()
        self.loop_viewer_lock.release()
        logger.warning("## End of loop_viewer")

    def stop_process(self):
        """
        Terminate the compute_cs, centroidal and wholebody process, close all the pipes
        and send the "stop motion" flag to the viewer
        """
        self.stop_motion_flag.value = True
        logger.warning("STOP MOTION flag sent")
        if self.process_compute_cs:
            self.process_compute_cs.terminate()
        if self.pipe_cs_in:
            self.pipe_cs_in.close()
        if self.pipe_cs_out:
            self.pipe_cs_out.close()
        if self.pipe_cs_com_in:
            self.pipe_cs_com_in.close()
        if self.pipe_cs_com_out:
            self.pipe_cs_com_out.close()
        #if self.queue_qt:
        #    self.queue_qt.close()

        if self.process_centroidal:
            self.process_centroidal.terminate()
        if self.process_wholebody:
            self.process_wholebody.terminate()

    def start_viewer_process(self):
        """
        Create a new queue_qt object and start the loop_viewer method in a new process
        """
        self.queue_qt = Queue()
        self.process_viewer = Process(target=self.loop_viewer)
        self.process_viewer.start()
        atexit.register(self.process_viewer.terminate)

    def start_process(self):
        """
        Create new pipes and queue objects and start the centroidal, wholebody and viewer loops in new processes
        Also delete the last_phase stored
        """
        self.pipe_cs_out, self.pipe_cs_in = Pipe(False)
        self.pipe_cs_com_out, self.pipe_cs_com_in = Pipe(False)
        #self.last_phase_pickled = Array(c_ubyte, MAX_PICKLE_SIZE)
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
        """
        Split the complete ContactSequence stored in self.cs and send each subsequence in the pipe_cs
        """
        pid_centroidal = 0
        last_iter_centroidal = False
        logger.info("## Compute from cs,  size = %d", self.cs.size())
        last_phase = self.get_last_phase()
        if last_phase:
            tools.setFinalFromInitialValues(last_phase, self.cs.contactPhases[0])
        while pid_centroidal + 5 < self.cs.size():
            logger.debug("## Current pid = %d", pid_centroidal)
            if pid_centroidal + 7 >= self.cs.size():
                logger.debug("## Last centroidal iter")
                # last iter, take all the remaining phases
                num_phase = self.cs.size() - pid_centroidal
                last_iter_centroidal = True
            else:
                num_phase = 5
            logger.debug("## Num phase = %d", num_phase)
            # Extract the phases [pid_centroidal; pid_centroidal +num_phases] from cs_full
            cs_iter = ContactSequence(0)
            for i in range(pid_centroidal, pid_centroidal + num_phase):
                logger.debug("-- Add phase : %d", i)
                cs_iter.append(self.cs.contactPhases[i])
            self.pipe_cs_in.send([cs_iter, last_iter_centroidal])  # This call may be blocking if the pipe is full
            pid_centroidal += 2
        self.pipe_cs_in.close()

    def compute_cs_requirements(self):
        """
        Compute all the required data to use the ContactSequence stored in self.cs as input for the centroidal method
        or the wholebody method
        """
        tools.computePhasesTimings(self.cs, self.cfg)
        tools.computePhasesCOMValues(self.cs, self.cfg.Robot.DEFAULT_COM_HEIGHT)
        tools.setAllUninitializedContactModel(self.cs, cfg.Robot)
        tools.computeRootTrajFromContacts(self.fullBody, self.cs)
        tools.setAllUninitializedFrictionCoef(self.cs, self.cfg.MU)

    def compute_stopping_cs(self, move_to_support_polygon=True):
        """
        Compute a Contact Sequence with centroidal trajectories to bring the current last_phase
        to a stop without contact changes
        :param move_to_support_polygon: if True, add a trajectory to put the CoM above the center of the support polygon
        :return:
        """
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
            if move_to_support_polygon:
                phase_projected = ContactPhase()
                phase_projected.timeInitial = phase.timeFinal
                phase_projected.duration = DURATION_0_STEP
                tools.copyContactPlacement(phase, phase_projected)
                tools.setInitialFromFinalValues(phase, phase_projected)
                phase_projected.c_final = tools.computeCenterOfSupportPolygonFromPhase(
                    phase_stop, self.fullBody.DEFAULT_COM_HEIGHT)
                #FIXME 'default height'
                tools.connectPhaseTrajToFinalState(phase_projected)
                cs_ref.append(phase_projected)
        else:
            # TODO try 1 step :
            raise RuntimeError("One step capturability not implemented yet !")
        tools.computeRootTrajFromContacts(self.fullBody, cs_ref)
        self.last_phase = cs_ref.contactPhases[-1].copy()
        # define the final root position, translation from the CoM position and rotation from the feet rotation
        q_final = np.zeros(7)
        q_final[:3] = self.last_phase.c_final[::]
        placement_rot_root, _ = tools.rootOrientationFromFeetPlacement(self.cfg.Robot, None, self.last_phase, None)
        quat_root = Quaternion(placement_rot_root.rotation)
        q_final[3:7] = [quat_root.x, quat_root.y, quat_root.z, quat_root.w]
        self.last_phase.q_final = q_final
        self.last_phase_flag.value = False
        self.last_phase_pickled = Array(c_ubyte, MAX_PICKLE_SIZE)  # reset currently stored whole body last phase
        return cs_ref

    def run_zero_step_capturability(self, move_to_support_polygon=True):
        """
        Compute the centroidal trajectory to bring the current last_phaseto a stop without contact changes.
        Then start a viewer and a wholebody processes to generate and display the motion corresponding to this
        centroidal trajectory.
        :param move_to_support_polygon: if True, add a trajectory to put the CoM above the center of the support polygon
        """
        cs_ref = self.compute_stopping_cs(move_to_support_polygon)
        self.start_viewer_process()
        self.cfg.IK_dt = 0.02
        p = Process(target=self.compute_wholebody_queue, args=(cs_ref, ))
        p.start()

    def stop_motion(self, move_to_support_polygon=True):
        """
        Terminate all the running contact, centroidal and wholebody processes
        and remove the stepping stones from the display.
        If the robot is not at a stop, compute and display a motion bringing it at a steady state
        :param move_to_support_polygon: if True, add a trajectory to put the CoM above the center of the support polygon
        """
        self.stop_process()
        process_stones = Process(target=self.hide_stones_lock)
        process_stones.start()
        atexit.register(process_stones.terminate)
        if not self.is_at_stop():
            logger.warning("!!!!!! REQUIRE STOP MOTION: compute 0-step capturability")
            self.run_zero_step_capturability(move_to_support_polygon)

    def move_to_goal(self, root_goal):
        """
        Plan and execute a motion connecting the current configuration to one with the given root position
        If the robot is in motion, start by computing a safe stop motion.
        :param root_goal: list of size 3 or 7: translation and quaternion for the desired root position
        If the quaternion part is not specified, the final orientation is not constrained
        :return:
        """
        self.stop_motion(False)
        self.plan_guide(root_goal)
        logger.info("Guide planning solved, path id = %d", self.current_guide_id)
        self.compute_cs_from_guide()
        self.compute_cs_requirements()
        logger.info("Start process")

        self.start_process()
        time.sleep(0.1)
        logger.info("@@@ Start compute_from_cs @@@")
        self.process_compute_cs = Process(target=self.compute_from_cs)
        self.process_compute_cs.start()
        atexit.register(self.process_compute_cs.terminate)
        logger.info("@@@ END compute_from_cs @@@")

    def find_closest_guide_time(self, path_id):
        """
        Look for the index in the guide path that give the closest distance between the root position at this index
        and the wholebody root position in the last_phase configuration
        :param path_id: 
        :return: 
        """
        last_phase = self.get_last_phase()
        if last_phase is None:
            return 0.
        root_wb = np.array(last_phase.q_final[0:2])
        DT = 0.01
        current_t = 0.
        min_t = 0.
        min_dist = math.inf
        ps = self.guide_planner.ps
        t_max = ps.pathLength(path_id)
        while current_t <= t_max:
            root_guide = np.array(ps.configAtParam(path_id, current_t)[0:2])
            dist = np.linalg.norm(root_wb - root_guide)
            if dist < min_dist:
                min_dist = dist
                min_t = current_t
            current_t += DT
        return min_t

    def is_path_valid(self, path_id):
        """
        Check if the given path id stored in self.guide_planner.ps is valid or not
        :param path_id: the id of the path
        :return: True if the path is completely valid, False otherwise
        """
        DT = 0.01  # FIXME: timestep of the discretization for the collision checking of the path
        ps = self.guide_planner.ps
        robot_rom = self.guide_planner.rbprmBuilder
        t = self.find_closest_guide_time(path_id) - 0.1
        if t < 0:
            t = 0.
        t_max = ps.pathLength(path_id)
        while t <= t_max:
            report = robot_rom.isConfigValid(ps.configAtParam(path_id, t))
            if not report[0]:
                return False
            t += DT
        report = robot_rom.isConfigValid(ps.configAtParam(path_id, t_max))
        if not report[0]:
            return False
        else:
            return True

    def add_obstacle_to_viewer(self, name, size, position, color=[0, 0, 1, 1]):
        """
        Add an obstacle (a box) to the viewer. This call is blocking as it wait for a lock to access to the gepetto-gui API
        :param name: the node name of the new obstacle
        :param size: the size of the box [x, y, z]
        :param position: the placement (translation + quaternion) of the obstacle in the world frame
        :param color: the color (rgba) of the obstacle
        """
        node_name = "world/environments/" + name  #FIXME: change the prefix if there is changes in pinocchio ...
        logger.info("Waiting lock to add obstacles ...")
        self.viewer_lock.acquire()
        # add the obstacle to the viewer:
        self.gui.addBox(node_name, size[0], size[1], size[2], color)
        # move the obstacle to the given placement:
        self.gui.applyConfiguration(node_name, position)
        self.gui.refresh()
        self.viewer_lock.release()
        logger.info("Obstacles added in the viewer")

    def add_obstacle_to_problem_solvers(self, name, size, position, obstacle_client):
        """
        Add an obstacle to the environment of the planner
        :param name: the name of the obstacle
        :param size: the size of the box (x, y, z)
        :param position: the placement (translation + quaternion) of the obstacle
        :param obstacle_client: a corba-server Obstacle client instance
        """
        # add the obstacle to the problem solver:
        obstacle_client.createBox(name, size[0] + SCALE_OBSTACLE_COLLISION, size[1] + SCALE_OBSTACLE_COLLISION,
                                  size[2] + SCALE_OBSTACLE_COLLISION)
        obstacle_client.addObstacle(name, True, False)
        # move the obstacle to the given placement:
        obstacle_client.moveObstacle(name, position)

    def add_obstacle(self, size, position, color=[0, 0, 1, 1]):
        """
        Add a cube to the environment, and recompute a motion if the current computed motion become invalid
        :param size: The size of the cube [x, y, z]
        :param position: The placement of the cube: either a list of length 3 for the translation or
        a list of size 7 for translation + quaternion
        :param color: color of the obstacle in the viewer, blue by default
        :return:
        """
        logger.warning("!!!! ADD OBSTACLE REQUESTED")
        name = "obstacle_0"  #FIXME: change the prefix if there is changes in pinocchio ...
        # Add an id until it is an unused name:
        i = 1
        obs_names = self.guide_planner.ps.client.obstacle.getObstacleNames(True, False)
        while name in obs_names:
            name = "obstacle_" + str(i)
            i += 1

        if len(position) == 3:
            position += [0, 0, 0, 1]
        logger.info("!!!! Addobstacle name : %s", name)
        self.add_obstacle_to_problem_solvers(name, size, position, self.guide_planner.ps.client.obstacle)
        self.add_obstacle_to_problem_solvers(name, size, position, self.fullBody.client.obstacle)

        logger.info("!!!! obstacle added to the problem")
        # add obstacle to the viewer, do it in a process as this call is blocking:
        process_obstacle = Process(target=self.add_obstacle_to_viewer, args=(name, size, position, color))
        process_obstacle.start()
        atexit.register(process_obstacle.terminate)
        logger.info("!!!! start thread to display obstacle")

        # Check if the motion must be re-planned :
        if not self.is_at_stop():
            logger.info("!!!!!! Add obstacle during motion, check path ...")
            valid = self.is_path_valid(self.current_guide_id)
            if valid:
                logger.warning("!!!!!! Current path is still valid, continue ...")
            else:
                logger.warning("!!!!!! Current path is now invalid ! Compute a new one ...")
                # Re plan the motion
                self.move_to_goal(self.current_root_goal)
        else:
            logger.warning("!!!!!! Add obstacle: The robot is not in motion")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run a multicontact-locomotion-planning scenario")
    parser.add_argument('demo_name',
                        type=str,
                        help="The name of the demo configuration file to load. "
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
