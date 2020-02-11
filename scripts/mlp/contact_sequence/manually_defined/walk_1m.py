from mlp.utils.cs_tools import addPhaseFromConfig
import multicontact_api
from multicontact_api import ContactSequence
import mlp.viewer.display_tools as display_tools
from pinocchio import SE3
from numpy import array
from mlp.utils.cs_tools import walk
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

walk(fb, cs, 1., 0.2, [fb.rLegId, fb.lLegId])

display_tools.displaySteppingStones(cs, gui, sceneName, fb)

filename = "talos_flatGround.cs"
print("Write contact sequence binary file : ", filename)
cs.saveAsBinary(filename)
