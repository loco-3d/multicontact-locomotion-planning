from mlp.utils.cs_tools import addPhaseFromConfig, moveEffectorOf, setFinalState, removeContact, moveEffectorToPlacement
import multicontact_api
from multicontact_api import ContactSequenceHumanoid
import mlp.viewer.display_tools as display_tools
import mlp.config as cfg
from pinocchio import SE3
from mlp.utils.util import rotatePlacement
from talos_rbprm.talos import Robot  # change robot here
from mlp.utils.cs_tools import moveEffectorToPlacement
import numpy as np
from hpp.corbaserver.rbprm.rbprmstate import State, StateHelper
multicontact_api.switchToNumpyArray()

ENV_NAME = "multicontact/stairsAirbus_noRamp"

fb, v = display_tools.initScene(Robot, ENV_NAME, False)
cs = ContactSequenceHumanoid(0)

#Create an initial contact phase :
q_ref = [
    0,
    0,
    0.9832773,
    0,
    0.0,
    0.0,
    1.0,  #Free flyer
    1.45,
    0.0,
    -0.611354,
    1.059395,
    -0.448041,
    -0.001708,  #Left Leg
    -1.45,
    0.0,
    -0.611354,
    1.059395,
    -0.448041,
    -0.001708,  #Right Leg
    0.0,
    0.006761,  #Chest
    0.25847,
    1.3,
    -0.5,
    -1.8,
    1.2,
    0.2,
    0.4,
    -0.005,  #Left Arm
    -0.25847,
    -1.3,
    0.5,
    -1.8,
    -1.2,
    -0.2,
    0.4,
    -0.005,  #Right Arm
    0.0,
    0.0
] + [0] * 6

q_ref[0:2] = [-3., -1.7575]
v(q_ref)  # move root position
q_ref[2] -= 0.015
fb.setReferenceConfig(q_ref)
fb.setPostureWeights(fb.postureWeights_straff[::] + [0] * 6)

# create hand contacts :
pRH = SE3.Identity()
pRH = rotatePlacement(pRH, 'y', -0.8115781021773631)
n = computeContactNormal(pRH).tolist()

state = State(fb, q=q_ref, limbsIncontact=[fb.rLegId, fb.lLegId])
pLHand = [-2.75, -1.235, 1.02]
pRHand = [-2.75, -2.28, 1.02]
"""
from tools.display_tools import *
createSphere('s',v)
moveSphere('s',v,pLHand)
"""

state, success = StateHelper.addNewContact(state, fb.rArmId, pRHand, n, lockOtherJoints=True)
assert success, "Unable to create contact for right hand"
state, success = StateHelper.addNewContact(state, fb.lArmId, pLHand, n, lockOtherJoints=True)
assert success, "Unable to create contact for right hand"
q_ref = state.q()

fb.setReferenceConfig(q_ref)
fb.setPostureWeights(fb.postureWeights_straff[::] + [0] * 6)
v(q_ref)

addPhaseFromConfig(fb, v, cs, q_ref, [fb.rLegId, fb.lLegId, fb.rArmId, fb.lArmId])

num_steps = 1
step_height = 0.22
step_width = 0.207

gait = [fb.rhand, fb.lhand, fb.rfoot, fb.lfoot]
for i in range(num_steps):
    for eeName in gait:
        moveEffectorOf(fb, v, cs, eeName, [step_width, 0, step_height])

removeContact(fb, cs, fb.rhand)
removeContact(fb, cs, fb.lhand)

q_end = q_ref[::]
q_end[0] += 0.15 + num_steps * step_width
q_end[2] += num_steps * step_height

setFinalState(cs, q=q_end)

DEMO_NAME = "talos_stairsAirbus"
filename = cfg.CONTACT_SEQUENCE_PATH + "/" + DEMO_NAME + ".cs"
print("Write contact sequence binary file : ", filename)
cs.saveAsBinary(filename)
