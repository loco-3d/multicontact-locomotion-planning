from mlp import LocoPlanner, Config
from hpp.corbaserver.rbprm.utils import ServerManager
import time, subprocess, shutil

def store_all(cfg):
    cfg.IK_store_centroidal = True
    cfg.IK_store_zmp = True
    cfg.IK_store_effector = True
    cfg.IK_store_contact_forces = True
    cfg.IK_store_joints_derivatives = True
    cfg.IK_store_joints_torque = True

def set_cs_names(cfg, demo_name):
    cfg.CS_FILENAME = demo_name + ".cs"
    cfg.COM_FILENAME = demo_name + "_COM.cs"
    cfg.REF_FILENAME = demo_name + "_REF.cs"
    cfg.WB_FILENAME = demo_name + "_WB.cs"

def gen_com_motion():
    import mlp.contact_sequence.manually_defined.com_motion_above_feet
    cfg = Config()
    cfg.load_scenario_config("talos_flatGround")
    cfg.centroidal_method="load"
    demo_name = "com_motion_above_feet"
    set_cs_names(cfg, demo_name)
    store_all(cfg)

    loco_planner = LocoPlanner(cfg)
    loco_planner.run()

def gen_step_in_place_quasistatic():
    import mlp.contact_sequence.manually_defined.step_in_place_quasiStatic
    cfg = Config()
    cfg.load_scenario_config("talos_flatGround_quasiStatic")
    cfg.centroidal_method="load"
    demo_name = "step_in_place_quasistatic"
    set_cs_names(cfg, demo_name)
    store_all(cfg)

    loco_planner = LocoPlanner(cfg)
    loco_planner.run()

def gen_step_in_place():
    import mlp.contact_sequence.manually_defined.step_in_place
    cfg = Config()
    cfg.load_scenario_config("talos_flatGround")
    cfg.contact_generation_method = "load"
    cfg.centroidal_method="momentumopt"
    cfg.ITER_DYNAMIC_FILTER = 0
    demo_name = "step_in_place"
    set_cs_names(cfg, demo_name)
    store_all(cfg)

    loco_planner = LocoPlanner(cfg)
    loco_planner.run()


def gen_walk_20cm_cs():
    import mlp.contact_sequence.manually_defined.walk_1m
    shutil.move("talos_flatGround.cs", "walk_20cm.cs")
    shutil.copy("walk_20cm.cs", "walk_20cm_quasistatic.cs")


def gen_walk_20cm():
    cfg = Config()
    cfg.load_scenario_config("talos_flatGround")
    cfg.contact_generation_method = "load"
    cfg.centroidal_method = "momentumopt"
    cfg.ITER_DYNAMIC_FILTER = 2
    demo_name = "walk_20cm"
    set_cs_names(cfg, demo_name)
    store_all(cfg)

    loco_planner = LocoPlanner(cfg)
    loco_planner.run()

def gen_walk_20cm_quasistatic():
    cfg = Config()
    cfg.load_scenario_config("talos_flatGround_quasiStatic")
    cfg.contact_generation_method = "load"
    cfg.centroidal_method = "quasistatic"
    cfg.ITER_DYNAMIC_FILTER = 0
    demo_name = "walk_20cm_quasistatic"
    set_cs_names(cfg, demo_name)
    store_all(cfg)

    loco_planner = LocoPlanner(cfg)
    loco_planner.run()


if __name__ == "__main__":
    with ServerManager('hpp-rbprm-server'):
        gen_com_motion()
        gen_step_in_place_quasistatic()
        gen_step_in_place()
        gen_walk_20cm_cs()
        gen_walk_20cm()
        gen_walk_20cm_quasistatic()



