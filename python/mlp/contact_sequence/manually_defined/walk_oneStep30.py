from mlp.utils.cs_tools import addPhaseFromConfig, setFinalState
import multicontact_api
from multicontact_api import ContactSequence
import mlp.viewer.display_tools as display_tools
from numpy import array
from pinocchio import SE3
from talos_rbprm.talos import Robot  # change robot here
multicontact_api.switchToNumpyArray()

ENV_NAME = "multicontact/ground"

fb, v = display_tools.initScene(Robot, ENV_NAME, False)
gui = v.client.gui
sceneName = v.sceneName

cs = ContactSequence(0)

#Create an initial contact phase :
q_ref = fb.referenceConfig[::] + [0] * 6
addPhaseFromConfig(fb, cs, q_ref, [fb.rLegId, fb.lLegId])

displacement = SE3.Identity()
displacement.translation = array([0.2, 0, 0])

cs.moveEffectorOf(fb.rfoot, displacement)
cs.moveEffectorOf(fb.lfoot, displacement)

q_end = q_ref[::]
q_end[0] += 0.2
fb.setCurrentConfig(q_end)
com = fb.getCenterOfMass()
setFinalState(cs, com, q=q_end)

display_tools.displaySteppingStones(cs, gui, sceneName, fb)


DEMO_NAME = "talos_flatGround"
filename =  DEMO_NAME + ".cs"
print("Write contact sequence binary file : ", filename)
cs.saveAsBinary(filename)
