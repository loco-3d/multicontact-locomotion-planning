# Copyright (c) 2020, CNRS
# Authors: Pierre Fernbach <pfernbac@laas.fr>
import unittest
import subprocess
import time
from mlp import LocoPlanner, Config
from utils import check_motion
import os
from mlp.utils.cs_tools import addPhaseFromConfig, setFinalState
from pinocchio import SE3
from numpy import array
import multicontact_api
from multicontact_api import ContactSequence
import mlp.viewer.display_tools as display_tools
from talos_rbprm.talos import Robot  # change robot here

def create_stairs_cs():
    ENV_NAME = "multicontact/bauzil_stairs"

    fb, v = display_tools.initScene(Robot, ENV_NAME, False)
    cs = ContactSequence(0)

    # Create an initial contact phase :
    q_ref = fb.referenceConfig[::] + [0] * 6
    q_ref[0:2] = [0.07, 1.2]
    addPhaseFromConfig(fb, cs, q_ref, [fb.rLegId, fb.lLegId])

    step_height = 0.1
    step_width = 0.3
    displacement = SE3.Identity()
    displacement.translation = array([step_width, 0, step_height])
    cs.moveEffectorOf(fb.rfoot, displacement)
    cs.moveEffectorOf(fb.lfoot, displacement)

    q_end = q_ref[::]
    q_end[0] += step_width
    q_end[2] += step_height
    fb.setCurrentConfig(q_end)
    com = fb.getCenterOfMass()
    setFinalState(cs, array(com), q=q_end)
    return cs

class TestTalosStairsManualLimbRRT(unittest.TestCase):
    def test_talos_stairs_manual_limb_rrt(self):
        subprocess.run(["killall", "hpp-rbprm-server"])
        process = subprocess.Popen("hpp-rbprm-server")
        time.sleep(3)

        cfg = Config()
        cfg.load_scenario_config("talos_stairs10")
        cfg.contact_generation_method = "load"
        cfg.centroidal_method = "momentumopt"
        cfg.IK_store_centroidal = True
        cfg.IK_store_zmp = True
        cfg.IK_store_effector = True
        cfg.IK_store_contact_forces = True
        cfg.IK_store_joints_derivatives = True
        cfg.IK_store_joints_torque = True
        cfg.ITER_DYNAMIC_FILTER = 0
        cs = create_stairs_cs()
        if not os.path.exists(cfg.CONTACT_SEQUENCE_PATH):
            os.makedirs(cfg.CONTACT_SEQUENCE_PATH)
        filename = cfg.CS_FILENAME
        print("Write contact sequence binary file : ", filename)
        cs.saveAsBinary(filename)


        loco_planner = LocoPlanner(cfg)
        loco_planner.run()

        check_motion(self, loco_planner)

        process.kill()



if __name__ == '__main__':
    unittest.main()
