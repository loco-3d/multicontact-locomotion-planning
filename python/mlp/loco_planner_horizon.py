from mlp import LocoPlanner
from mlp.config import Config
from mlp.viewer.display_tools import disp_wb_pinocchio, initScenePinocchio
import argparse
import atexit
import subprocess
import time
import os
import numpy as np
np.set_printoptions(precision=6)
import eigenpy
import curves
from curves import piecewise, SE3Curve, polynomial
from multicontact_api import ContactSequence
from mlp.utils.cs_tools import computePhasesTimings, setInitialFromFinalValues, setAllUninitializedFrictionCoef
from mlp.utils.cs_tools import computePhasesCOMValues, computeRootTrajFromContacts
from multiprocessing import Process, Queue
from queue import Empty as queue_empty

eigenpy.switchToNumpyArray()

def update_root_traj_timings(cs):
    for cp in cs.contactPhases:
        cp.root_t = SE3Curve(cp.root_t(cp.root_t.min()), cp.root_t(cp.root_t.max()), cp.timeInitial, cp.timeFinal)


def compute_centroidal(generate_centroidal, CentroidalInputs, #CentroidalOutputs,
                       cfg, cs, previous_phase, last_iter = False):    # update the initial state with the data from the previous intermediate state:
    if previous_phase:
        setInitialFromFinalValues(previous_phase, cs.contactPhases[0])

    if not CentroidalInputs.checkAndFillRequirements(cs, cfg, None):
        raise RuntimeError(
            "The current contact sequence cannot be given as input to the centroidal method selected.")
    cs_full = generate_centroidal(cfg, cs, None, None)
    # CentroidalOutputs.assertRequirements(cs_full)
    if last_iter:
        return cs_full, None
    else:
        cs_cut = ContactSequence(0)
        for i in range(3):
            cs_cut.append(cs_full.contactPhases[i])
        return cs_cut, cs_cut.contactPhases[1]

def compute_wholebody(generate_effector_trajectories, EffectorInputs, #EffectorOutputs,
                      generate_wholebody, WholebodyInputs, #WholebodyOutputs,
                      cfg, fullBody, robot, cs_com,
                      last_q = None, last_v = None, last_iter = False):    ### Effector trajectory reference
    if not EffectorInputs.checkAndFillRequirements(cs_com, cfg, fullBody):
        raise RuntimeError(
            "The current contact sequence cannot be given as input to the end effector method selected.")
    cs_ref_full = generate_effector_trajectories(cfg, cs_com, fullBody)
    # EffectorOutputs.assertRequirements(cs_ref_full)
    if last_iter:
        cs_ref = cs_ref_full
    else:
        cs_cut = ContactSequence()
        for i in range(2):
            cs_cut.append(cs_ref_full.contactPhases[i])
        cs_ref = cs_cut

    if last_q is not None:
        cs_ref.contactPhases[0].q_init = last_q
    if last_v is not None:
        t_init = cs_ref.contactPhases[0].timeInitial
        cs_ref.contactPhases[0].dq_t = piecewise(polynomial(last_v.reshape(-1, 1), t_init, t_init))

    ### Wholebody
    update_root_traj_timings(cs_ref)
    if not WholebodyInputs.checkAndFillRequirements(cs_ref, cfg, fullBody):
        raise RuntimeError(
            "The current contact sequence cannot be given as input to the wholeBody method selected.")
    cs_wb, robot = generate_wholebody(cfg, cs_ref, fullBody, robot=robot)
    #print("-- compute whole body END")
    # WholebodyOutputs.assertRequirements(cs_wb)
    last_q =  cs_wb.contactPhases[-1].q_t(cs_wb.contactPhases[-1].timeFinal)
    last_v = cs_wb.contactPhases[-1].dq_t(cs_wb.contactPhases[-1].timeFinal)
    return cs_wb, last_q, last_v, robot


def loop_centroidal(queue_cs, queue_cs_com,
                    generate_centroidal, CentroidalInputs,  # CentroidalOutputs,
                    cfg):
    last_centroidal_phase = None
    last_iter = False
    while not last_iter:
        cs, last_iter = queue_cs.get()
        #print("## Run centroidal")
        cs_com, last_centroidal_phase = compute_centroidal(generate_centroidal, CentroidalInputs,  # CentroidalOutputs,
                                                           cfg, cs, last_centroidal_phase, last_iter)
        #print("-- Add a cs_com to the queue")
        queue_cs_com.put([cs_com, last_iter])
    queue_cs_com.close()



def loop_wholebody( queue_cs_com, queue_q_t,
                    generate_effector_trajectories, EffectorInputs, #EffectorOutputs,
                    generate_wholebody, WholebodyInputs, #WholebodyOutputs,
                    cfg, fullBody):
    last_q = None
    last_v = None
    robot = None
    last_iter = False
    while not last_iter:
        cs_com, last_iter = queue_cs_com.get()
        #print("## Run wholebody")
        cs_wb, last_q, last_v, robot = compute_wholebody(generate_effector_trajectories, EffectorInputs,  # EffectorOutputs,
                                             generate_wholebody, WholebodyInputs,  # WholebodyOutputs,
                                             cfg, fullBody, robot,
                                            cs_com, last_q, last_v, last_iter)
        #print("-- Add a cs_wb to the queue")
        queue_q_t.put([cs_wb.concatenateQtrajectories(), last_q])
    queue_q_t.close()


def loop_viewer(queue_q_t, cfg):
    robot, gui = initScenePinocchio(cfg.Robot.urdfName + cfg.Robot.urdfSuffix , cfg.Robot.packageName, cfg.ENV_NAME)
    while True:
        q_t, last_displayed_q = queue_q_t.get()
        disp_wb_pinocchio(robot, q_t, cfg.DT_DISPLAY)



class LocoPlannerHorizon(LocoPlanner):

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
        self.generate_centroidal, self.CentroidalInputs, self.CentroidalOutputs = self.cfg.get_centroidal_method()
        self.generate_effector_trajectories, self.EffectorInputs, self.EffectorOutputs = \
            self.cfg.get_effector_initguess_method()
        self.generate_wholebody, self.WholebodyInputs, self.WholebodyOutputs = self.cfg.get_wholebody_method()

        self.process_centroidal = None
        self.process_wholebody = None
        self.process_viewer = None
        self.process_gepetto_gui = None
        self.queue_cs = None
        self.queue_cs_com = None
        self.queue_q_t = None

        self.last_displayed_q = None # last config displayed


    def start_process(self):
        if self.process_centroidal:
            self.process_centroidal.terminate()
        if self.process_wholebody:
            self.process_wholebody.terminate()
        self.queue_cs = Queue(10)
        self.queue_cs_com = Queue(10)
        self.queue_q_t = Queue(5)
        self.last_displayed_q = None

        self.process_centroidal = Process(target=loop_centroidal, args=(self.queue_cs, self.queue_cs_com,
                                                                             self.generate_centroidal,
                                                                             self.CentroidalInputs,
                                                                             self.cfg))
        self.process_centroidal.start()
        self.process_wholebody = Process(target=loop_wholebody, args=(self.queue_cs_com, self.queue_q_t,
                                                                           self.generate_effector_trajectories,
                                                                           self.EffectorInputs,
                                                                           self.generate_wholebody,
                                                                           self.WholebodyInputs,
                                                                           self.cfg,
                                                                      self.fullBody))
        self.process_wholebody.start()

        if not self.process_viewer:
            subprocess.run(["killall", "gepetto-gui"])
            self.process_gepetto_gui = subprocess.Popen("gepetto-gui",
                                              stdout=subprocess.PIPE,
                                              stderr=subprocess.DEVNULL,
                                              preexec_fn=os.setpgrp)
            atexit.register(self.process_gepetto_gui.kill)
            time.sleep(3)
            self.process_viewer = Process(target=loop_viewer, args=(self.queue_q_t,
                                                                         self.cfg))
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
                cfg.DURATION_CONNECT_GOAL = self.previous_connect_goal
            else:
                num_phase = 5
            #print("## Num phase = ", num_phase)
            # Extract the phases [pid_centroidal; pid_centroidal +num_phases] from cs_full
            cs_iter = ContactSequence(0)
            for i in range(pid_centroidal, pid_centroidal + num_phase):
                #print("-- Add phase : ", i)
                cs_iter.append(self.cs.contactPhases[i])
            self.queue_cs.put([cs_iter, last_iter_centroidal])
            pid_centroidal += 2
        self.queue_cs.close()

    def run(self):
        self.run_contact_generation()
        self.cs = computePhasesTimings(self.cs, self.cfg)
        self.cs = computePhasesCOMValues(self.cs, self.cfg.Robot.DEFAULT_COM_HEIGHT)
        computeRootTrajFromContacts(self.fullBody, self.cs)
        setAllUninitializedFrictionCoef(self.cs, self.cfg.MU)

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


    loco_planner = LocoPlannerHorizon(cfg)
    loco_planner.run()



