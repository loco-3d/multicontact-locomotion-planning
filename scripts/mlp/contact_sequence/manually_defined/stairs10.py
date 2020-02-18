from mlp.utils.cs_tools import addPhaseFromConfig, setFinalState
from pinocchio import SE3
from numpy import array
import multicontact_api
from multicontact_api import ContactSequence
import mlp.viewer.display_tools as display_tools
import mlp.config as cfg
from talos_rbprm.talos import Robot  # change robot here

multicontact_api.switchToNumpyArray()

ENV_NAME = "multicontact/bauzil_stairs"

fb, v = display_tools.initScene(Robot, ENV_NAME, False)
gui = v.client.gui
sceneName = v.sceneName

cs = ContactSequence(0)

#Create an initial contact phase :
q_ref = fb.referenceConfig[::] + [0] * 6
q_ref[0:2] = [0.07, 1.2]
addPhaseFromConfig(fb, cs, q_ref, [fb.rLegId, fb.lLegId])

num_steps = 6
step_height = 0.1
step_width = 0.3
displacement = SE3.Identity()
displacement.translation = array([step_width, 0, step_height])

for i in range(num_steps):
    cs.moveEffectorOf(fb.rfoot, displacement)
    cs.moveEffectorOf(fb.lfoot, displacement)

q_end = q_ref[::]
q_end[0] += step_width * num_steps
q_end[2] += step_height * num_steps
fb.setCurrentConfig(q_end)
com = fb.getCenterOfMass()
setFinalState(cs, array(com), q=q_end)

display_tools.displaySteppingStones(cs, gui, sceneName, fb)


DEMO_NAME = "talos_stairs10"
filename = cfg.CONTACT_SEQUENCE_PATH + "/" + DEMO_NAME + ".cs"
print("Write contact sequence binary file : ", filename)
cs.saveAsBinary(filename)
