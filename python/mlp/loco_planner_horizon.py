from mlp import LocoPlanner
from mlp.config import Config
from mlp.viewer.display_tools import displayWBmotion
import os, sys, argparse
import numpy as np
np.set_printoptions(precision=6)
import eigenpy
import time
import pinocchio
import curves
import multicontact_api
from multicontact_api import ContactSequence, ContactPhase
from mlp.utils.cs_tools import computePhasesTimings, setInitialFromFinalValues, setAllUninitializedFrictionCoef
from mlp.utils.cs_tools import computePhasesCOMValues, computeRootTrajFromContacts

eigenpy.switchToNumpyArray()

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


    def solve_current(self):

        self.run_contact_generation()
        cs_full = self.cs
        cs_full = computePhasesTimings(cs_full, self.cfg)
        cs_full = computePhasesCOMValues(cs_full, self.cfg.Robot.DEFAULT_COM_HEIGHT)
        computeRootTrajFromContacts(self.fullBody, cs_full)
        setAllUninitializedFrictionCoef(cs_full, self.cfg.MU)
        self.cs_full = cs_full # FIXME : remove it after debug

        pid_centroidal = 0
        first_iter_centroidal = True
        last_iter_centroidal = False
        print("## Cs size = ", cs_full.size())
        while pid_centroidal + 5 < cs_full.size():
            print("## Current pid = ", pid_centroidal)
            if pid_centroidal + 7 >= cs_full.size():
                print("## Last centroidal iter")
                # last iter, take all the remaining phases
                num_phase = cs_full.size() - pid_centroidal
                last_iter_centroidal = True
                cfg.DURATION_CONNECT_GOAL = self.previous_connect_goal
            else:
                num_phase = 5
            print("## Num phase = ", num_phase)
            # Extract the phases [pid_centroidal; pid_centroidal +num_phases] from cs_full
            self.cs = ContactSequence(0)
            for i in range(pid_centroidal, pid_centroidal + num_phase):
                print("-- Add phase : ", i)
                self.cs.append(cs_full.contactPhases[i])
            # update the initial state with the data from the previous intermediate state:
            if not first_iter_centroidal:
                print("## Set initial from final values")
                setInitialFromFinalValues(self.cs_com.contactPhases[1], self.cs.contactPhases[0])

            # solve the current centroidal problem:
            print("## Run centroidal")
            self.run_centroidal()

            print("## Extract first two phases for wholebody")
            if not last_iter_centroidal:
                # extract the first 2 phases of cs_com to run the whole body problem:
                cs_com_full = self.cs_com
                self.cs_com_list += [cs_com_full]
                self.cs_com = ContactSequence(0)
                for i in range(3):
                    self.cs_com.append(cs_com_full.contactPhases[i])

            self.cs_ref = None
            self.run_effector_init_guess()
            if not last_iter_centroidal:
                cs_ref_full = self.cs_ref
                self.cs_ref = ContactSequence(0)
                for i in range(2):
                    self.cs_ref.append(cs_ref_full.contactPhases[i])

            print("## New cs_ref size : ", self.cs_com.size())
            if not first_iter_centroidal:
                self.cs_ref.contactPhases[0].q_init = self.last_wb_phase.q_t(self.last_wb_phase.timeFinal)

            self.run_wholebody()
            self.last_wb_phase = self.cs_wb.contactPhases[-1]
            print("## Whole body solved, size : ", self.cs_wb.size())
            displayWBmotion(self.viewer, self.cs_wb.concatenateQtrajectories(), self.cfg.DT_DISPLAY)


            if first_iter_centroidal:
                first_iter_centroidal = False
                self.cfg.COM_SHIFT_Z = 0.
                self.cfg.TIME_SHIFT_COM = 0.
            pid_centroidal += 2

    def run(self):
        self.solve_current()



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






