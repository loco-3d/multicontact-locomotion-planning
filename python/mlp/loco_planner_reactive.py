from mlp import LocoPlanner
from mlp.config import Config
from mlp.viewer.display_tools import disp_wb_pinocchio, initScenePinocchio
import argparse
import atexit
import subprocess
import time
import os, sys, traceback
import numpy as np
np.set_printoptions(precision=6)
import eigenpy
import curves
from curves import piecewise, SE3Curve, polynomial
from multicontact_api import ContactSequence
from mlp.utils.cs_tools import computePhasesTimings, setInitialFromFinalValues, setAllUninitializedFrictionCoef
from mlp.utils.cs_tools import computePhasesCOMValues, computeRootTrajFromContacts
from multiprocessing import Process, Queue, Pipe, Value, Array, Lock
from ctypes import c_ubyte, c_bool
from queue import Empty as queue_empty
import pickle
eigenpy.switchToNumpyArray()

MAX_PICKLE_SIZE = 5000 # maximal size (in byte) of the pickled representation of a contactPhase

def update_root_traj_timings(cs):
    for cp in cs.contactPhases:
        cp.root_t = SE3Curve(cp.root_t(cp.root_t.min()), cp.root_t(cp.root_t.max()), cp.timeInitial, cp.timeFinal)

def clean_phase_trajectories(phase):
    """
    Delete all the centroidal trajectories of the given phase
    :param phase:
    :return:
    """
    phase.c_t = None
    phase.dc_t = None
    phase.ddc_t = None
    phase.L_t = None
    phase.dL_t = None
    phase.root_t = None

def copy_array(arr1, arr2):
    """
    Copy arr1 in arr2 element by elements (DEBUG tests)
    :param arr1:
    :param arr2:
    :return:
    """
    if len(arr1) > MAX_PICKLE_SIZE:
        raise ValueError("In copy array: given array is too big, size = "+len(arr1))
    for i, el in enumerate(arr1):
        arr2[i] = el

def load_from_array(arr):
    pic = bytes(arr)
    x = pickle.loads(pic)
    return x

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
        self.robot = None
        self.gui = None
        self.last_phase = Array(c_ubyte, MAX_PICKLE_SIZE) # will contain the last contact phase send to the viewer
        self.stop_motion_flag = Value(c_bool)

    def compute_centroidal(self, cs, previous_phase,
                           last_iter=False):  # update the initial state with the data from the previous intermediate state:
        if previous_phase:
            setInitialFromFinalValues(previous_phase, cs.contactPhases[0])

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
            last_phase = cs_com.contactPhases[1]

        if last_q is not None:
            cs_ref.contactPhases[0].q_init = last_q
        if last_v is not None:
            t_init = cs_ref.contactPhases[0].timeInitial
            cs_ref.contactPhases[0].dq_t = piecewise(polynomial(last_v.reshape(-1, 1), t_init, t_init))

        ### Wholebody
        update_root_traj_timings(cs_ref)
        if not self.WholebodyInputs.checkAndFillRequirements(cs_ref, self.cfg, self.fullBody):
            raise RuntimeError(
                "The current contact sequence cannot be given as input to the wholeBody method selected.")
        cs_wb, robot = self.generate_wholebody(self.cfg, cs_ref, self.fullBody, robot=robot)
        # print("-- compute whole body END")
        # WholebodyOutputs.assertRequirements(cs_wb)
        last_q = cs_wb.contactPhases[-1].q_t(cs_wb.contactPhases[-1].timeFinal)
        last_v = cs_wb.contactPhases[-1].dq_t(cs_wb.contactPhases[-1].timeFinal)
        clean_phase_trajectories(last_phase)
        last_phase.root_t = cs_ref.contactPhases[-1].root_t
        last_phase.q_final = last_q
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
                    copy_array(pickle.dumps(last_phase), self.last_phase)
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

    def stop_motion(self):
        self.stop_process()
        #self.clear_viewer_queue()
        return load_from_array(self.last_phase)

    def start_process(self):
        self.pipe_cs_out, self.pipe_cs_in = Pipe(False)
        self.pipe_cs_com_out, self.pipe_cs_com_in = Pipe(False)
        self.queue_qt = Queue()
        self.last_phase = Array(c_ubyte, MAX_PICKLE_SIZE) # will contain the last contact phase send to the viewer
        self.stop_motion_flag = Value(c_bool)
        self.stop_motion_flag.value = False

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

        self.process_viewer = Process(target=self.loop_viewer)
        self.process_viewer.start()
        atexit.register(self.process_viewer.terminate)


    def compute_from_cs(self):
        pid_centroidal = 0
        last_iter_centroidal = False
        print("## Compute from cs,  size = ", self.cs.size())
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

    def init_viewer(self):
        subprocess.run(["killall", "gepetto-gui"])
        self.process_gepetto_gui = subprocess.Popen("gepetto-gui",
                                                    stdout=subprocess.PIPE,
                                                    stderr=subprocess.DEVNULL,
                                                    preexec_fn=os.setpgrp)
        atexit.register(self.process_gepetto_gui.kill)
        time.sleep(3)
        self.robot, self.gui = initScenePinocchio(cfg.Robot.urdfName + cfg.Robot.urdfSuffix,
                                                  cfg.Robot.packageName, cfg.ENV_NAME)

    def run(self):
        self.run_contact_generation()
        self.init_viewer()
        self.cs = computePhasesTimings(self.cs, self.cfg)
        self.cs = computePhasesCOMValues(self.cs, self.cfg.Robot.DEFAULT_COM_HEIGHT)
        computeRootTrajFromContacts(self.fullBody, self.cs)
        setAllUninitializedFrictionCoef(self.cs, self.cfg.MU)

        self.start_process()
        time.sleep(2)
        self.compute_from_cs()

    ## debug helper
    def stop_and_retry(self):
        self.stop_motion()
        time.sleep(0.2)
        self.start_process()
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

    # kill already existing instance of the server
    subprocess.run(["killall", "hpp-rbprm-server"])
    # run the server in background :
    # stdout and stderr outputs of the child process are redirected to devnull (hidden).
    # preexec_fn is used to ignore ctrl-c signal send to the main script (otherwise they are forwarded to the child process)
    process_server = subprocess.Popen("hpp-rbprm-server",
                                      stdout=subprocess.PIPE,
                                      stderr=subprocess.DEVNULL,
                                      preexec_fn=os.setpgrp)
    # register cleanup methods to kill server when exiting python interpreter
    atexit.register(process_server.kill)
    time.sleep(3)


    loco_planner = LocoPlannerReactive(cfg)
    loco_planner.run()



