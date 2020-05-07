import subprocess
import time
import atexit
import argparse
import os
from mlp import LocoPlanner, Config
from hpp.corbaserver.rbprm.utils import ServerManager


# init argument parser
parser = argparse.ArgumentParser(description="Run a multicontact-locomotion-planning scenario")
parser.add_argument('demo_name', type=str, help="The name of the demo configuration file to load. "
                                                "Must be a valid python file, either inside the PYTHONPATH"
                                                "or inside the mlp.demo_configs folder. ")
parser.add_argument("-n", "--no_viewer", help="Run mlp without visualization.",action="store_true")
args = parser.parse_args()
demo_name = args.demo_name

cfg = Config()
cfg.load_scenario_config(demo_name)

with ServerManager('hpp-rbprm-server'):
    if not args.no_viewer:
        with ServerManager('gepetto-gui'):
            loco_planner = LocoPlanner(cfg)
            loco_planner.run()
    else:
        loco_planner = LocoPlanner(cfg)
        loco_planner.run()



