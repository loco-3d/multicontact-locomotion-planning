from mlp.utils.cs_tools import addPhaseFromConfig, setFinalState
import multicontact_api
from multicontact_api import ContactSequence, ContactPatch
import mlp.viewer.display_tools as display_tools
from pinocchio import SE3
from numpy import array
import mlp.config as cfg

from talos_rbprm.talos import Robot  # change robot here
multicontact_api.switchToNumpyArray()

ENV_NAME = "multicontact/ground"

# Build the robot object and the viewer
fb, v = display_tools.initScene(Robot, ENV_NAME, False)
gui = v.client.gui
sceneName = v.sceneName
# display the origin and x,y,z axis in the viewer :
v.addLandmark(sceneName, 1)
#initialize an empty contact sequence
cs = ContactSequence(0)

# start from the reference configuration of the robot :
q_ref = fb.referenceConfig[::] + [0] * 6
#q_ref[0:2] = [ , ] # change the x,y position of the base as needed
# q_ref[2] +=  # the z position of the base already correspond to a floor at z=0, change it accordingly to the environment
# q_ref[3:7] =   # quaternion (x,y,z) of the base

# display the configuration
v(q_ref)

# create a first contact phase corresponding to this configuration
limbsInContact = [fb.rLegId, fb.lLegId]  # define the limbs in contact for this first phase
addPhaseFromConfig(fb, cs, q_ref, limbsInContact)

print("number of contact phases in the contact sequence : ", cs.size())
# there is now one phase in our 'cs' object

# we can now add new contact phases by changing the contacts of the last phase of the sequence:
cs.breakContact(fb.rfoot)  # add a new contact phase to the sequence: break the right feet contact
# create a new contact for the right feet at a different position :
placement = cs.contactPhases[0].contactPatch(fb.rfoot).placement.copy()  # take the previous position of the right feet
pos = placement.translation
pos[0] += 0.15  # move of 15cm in the x direction
placement.translation = pos
# add a new contact phase, with a contact created for the right feet at the given placement :
cs.createContact(fb.rfoot, ContactPatch(placement))

#display the contacts in gepetto viewer :
#display_tools.displaySteppingStones(cs, gui, v.sceneName, fb)
print("number of contact phases in the contact sequence : ", cs.size())
# There is now 3 contact phases (corresponding to one step)

#first phase:
print("Right feet contact of phase 0 ")
print(cs.contactPhases[0].isEffectorInContact(fb.rfoot))
print(cs.contactPhases[0].contactPatch(fb.rfoot).placement)
print("Left feet contact of phase 0 ")
print(cs.contactPhases[0].isEffectorInContact(fb.lfoot))
print(cs.contactPhases[0].contactPatch(fb.lfoot).placement)
#second phase:
print("Right feet contact of phase 1 ")
print(cs.contactPhases[1].isEffectorInContact(fb.rfoot))
print("Left feet contact of phase 1 ")
print(cs.contactPhases[1].isEffectorInContact(fb.lfoot))
print(cs.contactPhases[1].contactPatch(fb.lfoot).placement)
#Third phase:
print("Right feet contact of phase 2 ")
print(cs.contactPhases[2].isEffectorInContact(fb.rfoot))
print(cs.contactPhases[2].contactPatch(fb.rfoot).placement)
print("Left feet contact of phase 2 ")
print(cs.contactPhases[2].isEffectorInContact(fb.lfoot))
print(cs.contactPhases[2].contactPatch(fb.lfoot).placement)

# A new step can easily be added with the following method:
displacement = SE3.Identity()
displacement.translation = array([0.3, 0, 0])
cs.moveEffectorOf(fb.lfoot, displacement)
# this method automatically add the required intermediate contact phase (with a contact break if needed)

# let's add 4 more steps
cs.moveEffectorOf(fb.rfoot, displacement)
cs.moveEffectorOf(fb.lfoot, displacement)
cs.moveEffectorOf(fb.rfoot, displacement)
displacement.translation = array([0.15, 0, 0])
cs.moveEffectorOf(fb.lfoot, displacement)


# display the position of the new contacts :
display_tools.displaySteppingStones(cs, gui, sceneName, fb)
# display the wholebody configurations at each step :
#display_tools.displayContactSequence(v, cs, 0.5)  # there is 6 steps

print("Number of contact phases : ", cs.size())

# serialize the contact sequence:
DEMO_NAME = "talos_flatGround"
filename = cfg.CONTACT_SEQUENCE_PATH + "/" + DEMO_NAME + ".cs"
print("Write contact sequence binary file : ", filename)
cs.saveAsBinary(filename)
