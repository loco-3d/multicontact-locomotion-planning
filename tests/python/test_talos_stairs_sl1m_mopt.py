# Copyright (c) 2020, CNRS
# Authors: Pierre Fernbach <pfernbac@laas.fr>
import unittest
import subprocess
import time
from mlp import LocoPlanner, Config

class TestTalosStairsSl1mMopt(unittest.TestCase):
    def test_talos_stairs_sl1m_mopt(self):
        subprocess.run(["killall", "hpp-rbprm-server"])
        process = subprocess.Popen("hpp-rbprm-server")
        time.sleep(3)

        cfg = Config()
        cfg.IK_store_centroidal = True
        cfg.IK_store_zmp = True
        cfg.IK_store_effector = True
        cfg.IK_store_contact_forces = True
        cfg.IK_store_joints_derivatives = True
        cfg.IK_store_joints_torque = True
        cfg.load_scenario_config("talos_stairs10")
        cfg.contact_generation_method = "sl1m"
        cfg.centroidal_method = "momentumopt"
        cfg.wholebody_method="none"
        cfg.ITER_DYNAMIC_FILTER = 0

        loco_planner = LocoPlanner(cfg)
        loco_planner.run()

        self.assertTrue(loco_planner.cs.size() > 2)
        self.assertTrue(loco_planner.cs.size() < 50)
        self.assertEqual(loco_planner.cs.size(), loco_planner.cs_com.size())
        self.assertEqual(loco_planner.cs.size(), loco_planner.cs_ref.size())
        self.assertIsNone(loco_planner.cs_wb)

        # check that the sequence contains the expected data:
        self.assertTrue(loco_planner.cs.haveConsistentContacts())

        self.assertTrue(loco_planner.cs_ref.haveConsistentContacts())
        self.assertTrue(loco_planner.cs_ref.haveTimings())
        self.assertTrue(loco_planner.cs_ref.haveCentroidalTrajectories())
        self.assertTrue(loco_planner.cs_ref.haveZMPtrajectories())
        self.assertTrue(loco_planner.cs_ref.haveEffectorsTrajectories(1e-2))

        process.kill()



if __name__ == '__main__':
    unittest.main()
