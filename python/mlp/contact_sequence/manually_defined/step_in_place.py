from mlp.utils.cs_tools import addPhaseFromConfig, setFinalState
import multicontact_api
from multicontact_api import ContactSequence, ContactPatch
import mlp.viewer.display_tools as display_tools
from pinocchio import SE3
from numpy import array

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
com = array(fb.getCenterOfMass())

# create a first contact phase corresponding to this configuration
limbsInContact = [fb.rLegId, fb.lLegId]  # define the limbs in contact for this first phase
addPhaseFromConfig(fb, cs, q_ref, limbsInContact)

transform = SE3.Identity()
cs.moveEffectorOf(fb.rfoot, transform)
cs.moveEffectorOf(fb.lfoot, transform)
cs.moveEffectorOf(fb.rfoot, transform)
cs.moveEffectorOf(fb.lfoot, transform)

setFinalState(cs,com, q_ref)



assert cs.haveConsistentContacts()

cs.saveAsBinary("talos_flatGround.cs")
