from mlp import LocoPlanner
from mlp.config import Config
from mlp.viewer.display_tools import displayWBmotion
import argparse
import numpy as np
np.set_printoptions(precision=6)
import eigenpy
import curves
from curves import piecewise, SE3Curve
from multicontact_api import ContactSequence
from mlp.utils.cs_tools import computePhasesTimings, setInitialFromFinalValues, setAllUninitializedFrictionCoef
from mlp.utils.cs_tools import computePhasesCOMValues, computeRootTrajFromContacts
from multiprocessing import Process, SimpleQueue

eigenpy.switchToNumpyArray()

def update_root_traj_timings(cs):
    for cp in cs.contactPhases:
        cp.root_t = SE3Curve(cp.root_t(cp.root_t.min()), cp.root_t(cp.root_t.max()), cp.timeInitial, cp.timeFinal)

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
        cfg.IK_store_joints_derivatives = False
        cfg.IK_store_joints_torque = False
        cfg.contact_generation_method = "sl1m"
        self.previous_connect_goal = cfg.DURATION_CONNECT_GOAL
        cfg.DURATION_CONNECT_GOAL = 0.
        self.last_wb_phase = None
        self.cs_com_list = [] # FIXME: remove after debug
        super().__init__(cfg)
        self.generate_centroidal, self.CentroidalInputs, self.CentroidalOutputs = self.cfg.get_centroidal_method()
        self.generate_effector_trajectories, self.EffectorInputs, self.EffectorOutputs = \
            self.cfg.get_effector_initguess_method()
        self.generate_wholebody, self.WholebodyInputs, self.WholebodyOutputs = self.cfg.get_wholebody_method()

    def compute_centroidal(self, cs, previous_phase, last_iter=False):
        # update the initial state with the data from the previous intermediate state:
        if previous_phase:
            setInitialFromFinalValues(previous_phase, cs.contactPhases[0])

        if not self.CentroidalInputs.checkAndFillRequirements(cs, self.cfg, self.fullBody):
            raise RuntimeError(
                "The current contact sequence cannot be given as input to the centroidal method selected.")
        cs_full = self.generate_centroidal(self.cfg, cs, None, self.fullBody)
        # CentroidalOutputs.assertRequirements(cs_full)
        if last_iter:
            return cs_full, None
        else:
            cs_cut = ContactSequence(0)
            for i in range(3):
                cs_cut.append(cs_full.contactPhases[i])
            return cs_cut, cs_cut.contactPhases[1]

    def compute_wholebody(self, cs_com, last_q=None, last_iter=False):
        ### Effector trajectory reference
        if not self.EffectorInputs.checkAndFillRequirements(cs_com, self.cfg, self.fullBody):
            raise RuntimeError(
                "The current contact sequence cannot be given as input to the end effector method selected.")
        cs_ref_full = self.generate_effector_trajectories(self.cfg, cs_com, self.fullBody)
        # EffectorOutputs.assertRequirements(cs_ref_full)
        if last_iter:
            cs_ref = cs_ref_full
        else:
            cs_cut = ContactSequence()
            for i in range(2):
                cs_cut.append(cs_ref_full.contactPhases[i])
            cs_ref = cs_cut

        if last_q:
            cs_ref.contactPhases[0].q_init = last_q

        ### Wholebody
        update_root_traj_timings(cs_ref)
        if not self.WholebodyInputs.checkAndFillRequirements(cs_ref, self.cfg, self.fullBody):
            raise RuntimeError(
                "The current contact sequence cannot be given as input to the wholeBody method selected.")
        cs_wb = self.generate_wholebody(self.cfg, cs_ref, self.fullBody)
        # WholebodyOutputs.assertRequirements(cs_wb)
        return cs_wb.concatenateQtrajectories(), cs_wb.contactPhases[-1].q_t(cs_wb.contactPhases[-1].timeFinal)

    def solve(self):

        self.run_contact_generation()
        self.cs
        self.cs = computePhasesTimings(self.cs, self.cfg)
        self.cs = computePhasesCOMValues(self.cs, self.cfg.Robot.DEFAULT_COM_HEIGHT)
        computeRootTrajFromContacts(self.fullBody, self.cs)
        setAllUninitializedFrictionCoef(self.cs, self.cfg.MU)

        pid_centroidal = 0
        first_iter_centroidal = True
        last_iter_centroidal = False
        last_q = None
        last_centroidal_phase = None
        print("## Cs size = ", self.cs.size())
        while pid_centroidal + 5 < self.cs.size():
            print("## Current pid = ", pid_centroidal)
            if pid_centroidal + 7 >= self.cs.size():
                print("## Last centroidal iter")
                # last iter, take all the remaining phases
                num_phase = self.cs.size() - pid_centroidal
                last_iter_centroidal = True
                cfg.DURATION_CONNECT_GOAL = self.previous_connect_goal
            else:
                num_phase = 5
            print("## Num phase = ", num_phase)
            # Extract the phases [pid_centroidal; pid_centroidal +num_phases] from cs_full
            cs_iter = ContactSequence(0)
            for i in range(pid_centroidal, pid_centroidal + num_phase):
                print("-- Add phase : ", i)
                cs_iter.append(self.cs.contactPhases[i])

            # solve the current centroidal problem:
            print("## Run centroidal")
            cs_com, last_centroidal_phase = self.compute_centroidal(cs_iter, last_centroidal_phase, last_iter_centroidal)

            print("## Run wholebody")
            q_t, last_q = self.compute_wholebody(cs_com, last_q, last_iter_centroidal)

            displayWBmotion(self.viewer, q_t, self.cfg.DT_DISPLAY)

            if first_iter_centroidal:
                first_iter_centroidal = False
                self.cfg.COM_SHIFT_Z = 0.
                self.cfg.TIME_SHIFT_COM = 0.
            pid_centroidal += 2

    def run(self):
        self.solve()



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run a multicontact-locomotion-planning scenario")
    parser.add_argument('demo_name', type=str, help="The name of the demo configuration file to load. "
                                                    "Must be a valid python file, either inside the PYTHONPATH"
                                                    "or inside the mlp.demo_configs folder. ")
    args = parser.parse_args()
    demo_name = args.demo_name

    cfg = Config()
    cfg.load_scenario_config(demo_name)

    loco_planner = LocoPlannerHorizon(cfg)
    loco_planner.run()






