from mlp.utils.cs_tools import addPhaseFromConfig, moveEffectorOf, setFinalState, removeContact, moveEffectorToPlacement
import multicontact_api
from multicontact_api import ContactSequence
import mlp.viewer.display_tools as display_tools
multicontact_api.switchToNumpyArray()
import mlp.config as cfg

from talos_rbprm.talos_fixedUpper import Robot  # change robot here
ENV_NAME = "multicontact/ground"

fb, v = display_tools.initScene(Robot, ENV_NAME, False)
cs = ContactSequence(0)

#Create an initial contact phase :
q_ref = fb.referenceConfig[::] + [0] * 6
addPhaseFromConfig(fb, v, cs, q_ref, [fb.rLegId, fb.lLegId])

moveEffectorOf(fb, v, cs, fb.rfoot, [0.2, 0, 0])
moveEffectorOf(fb, v, cs, fb.lfoot, [0.2, 0, 0])

q_end = q_ref[::]
q_end[0] += 0.2
setFinalState(cs, q=q_end)

DEMO_NAME = "talos_flatGround"
filename = cfg.CONTACT_SEQUENCE_PATH + "/" + DEMO_NAME + ".cs"
print("Write contact sequence binary file : ", filename)
cs.saveAsBinary(filename)
