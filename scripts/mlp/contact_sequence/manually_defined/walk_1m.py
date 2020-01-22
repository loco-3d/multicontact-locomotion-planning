from multicontact_api import ContactSequenceHumanoid, ContactPhaseHumanoid
import mlp.viewer.display_tools as display_tools
from mlp.utils.cs_tools import *
import mlp.config as cfg

from talos_rbprm.talos import Robot  # change robot here
ENV_NAME = "multicontact/ground"

fb, v = display_tools.initScene(Robot, ENV_NAME, False)
cs = ContactSequenceHumanoid(0)

#Create an initial contact phase :
q_ref = fb.referenceConfig[::] + [0] * 6
addPhaseFromConfig(fb, v, cs, q_ref, [fb.rLegId, fb.lLegId])

walk(fb, v, cs, 0.5, 0.1, [fb.rLegId, fb.lLegId])

DEMO_NAME = "talos_flatGround"
filename = cfg.CONTACT_SEQUENCE_PATH + "/" + DEMO_NAME + ".cs"
print("Write contact sequence binary file : ", filename)
cs.saveAsBinary(filename)
