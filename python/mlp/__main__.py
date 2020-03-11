import subprocess
import time
import atexit
import argparse
import os
from mlp import LocoPlanner, Config


# init argument parser
parser = argparse.ArgumentParser(description="Run a multicontact-locomotion-planning scenario")
parser.add_argument('demo_name', type=str, help="The name of the demo configuration file to load. "
                                                "Must be a valid python file, either inside the PYTHONPATH"
                                                "or inside the mlp.demo_configs folder. ")
args = parser.parse_args()
demo_name = args.demo_name

cfg = Config()
cfg.load_scenario_config(demo_name)


# kill already existing instance of the viewer/server
subprocess.run(["killall", "gepetto-gui"])
subprocess.run(["killall", "hpp-rbprm-server"])
# run the viewer/server in background
# stdout and stderr outputs of the child process are redirected to devnull (hidden).
# preexec_fn is used to ignore ctrl-c signal send to the main script (otherwise they are forwarded to the child process)
process_viewer = subprocess.Popen("gepetto-gui",
                                  stdout=subprocess.PIPE,
                                  stderr=subprocess.DEVNULL,
                                  preexec_fn=os.setpgrp)
process_server = subprocess.Popen("hpp-rbprm-server",
                                  stdout=subprocess.PIPE,
                                  stderr=subprocess.DEVNULL,
                                  preexec_fn=os.setpgrp)
# wait a little for the initialization of the server/viewer
time.sleep(3)

# register cleanup methods to kill viewer/server when exiting python interpreter
atexit.register(process_server.kill)
atexit.register(process_viewer.kill)


loco_planner = LocoPlanner(cfg)
loco_planner.run()



