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
parser.add_argument("-n", "--no_viewer", help="Run mlp without visualization.",action="store_true")
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

# do the same for the viewer, exept if --no-viewer flag is set
disable_viewer = args.no_viewer
print("disable viewer : ", disable_viewer)
if disable_viewer is None:
    subprocess.run(["killall", "gepetto-gui"])
    process_viewer = subprocess.Popen("gepetto-gui",
                                      stdout=subprocess.PIPE,
                                      stderr=subprocess.DEVNULL,
                                      preexec_fn=os.setpgrp)
    atexit.register(process_viewer.kill)


# wait a little for the initialization of the server/viewer
time.sleep(3)


loco_planner = LocoPlanner(cfg)
loco_planner.run()



