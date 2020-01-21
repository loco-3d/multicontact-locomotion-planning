from multicontact_api import ContactSequenceHumanoid, ContactPhaseHumanoid
import mlp.viewer.display_tools as display_tools
from mlp.utils.cs_tools import *
import mlp.config as cfg

from talos_rbprm.talos import Robot  # change robot here
ENV_NAME = "multicontact/bauzil_stairs"

fb, v = display_tools.initScene(Robot, ENV_NAME, False)
cs = ContactSequenceHumanoid(0)

#Create an initial contact phase :
q_ref = fb.referenceConfig[::] + [0] * 6
q_ref[0:2] = [0.07, 1.2]
addPhaseFromConfig(fb, v, cs, q_ref, [fb.rLegId, fb.lLegId])

num_steps = 6
step_height = 0.1
step_width = 0.3

for i in range(num_steps):
    moveEffectorOf(fb, v, cs, fb.rfoot, [step_width, 0, step_height])
    moveEffectorOf(fb, v, cs, fb.lfoot, [step_width, 0, step_height])

q_end = q_ref[::]
q_end[0] += step_width * num_steps
q_end[2] += step_height * num_steps
setFinalState(cs, q=q_end)

DEMO_NAME = "talos_stairs10"
filename = cfg.CONTACT_SEQUENCE_PATH + "/" + DEMO_NAME + ".cs"
print("Write contact sequence binary file : ", filename)
cs.saveAsBinary(filename)
