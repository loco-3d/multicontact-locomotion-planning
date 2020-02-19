from mlp.utils.cs_tools import addPhaseFromConfig, setFinalState
import multicontact_api
from multicontact_api import ContactSequence, ContactPatch
import mlp.viewer.display_tools as display_tools
import mlp.config as cfg
from pinocchio import SE3
from mlp.utils.util import rotatePlacement
from talos_rbprm.talos import Robot  # change robot here
from numpy import array
multicontact_api.switchToNumpyArray()

ENV_NAME = "multicontact/stairsAirbus_noRamp"

fb, v = display_tools.initScene(Robot, ENV_NAME, False)
gui = v.client.gui
sceneName = v.sceneName

cs = ContactSequence(0)

#Create an initial contact phase :
q_ref = fb.referenceConfig[::] + [0] * 6

q_ref[0:2] = [-3.15, -1.7575]
v(q_ref)  # move root position
q_ref[2] -= 0.015
q_ref[24] -= 1.3  # lift up arms
q_ref[32] -= 1.3
v(q_ref)

addPhaseFromConfig(fb, cs, q_ref, [fb.rLegId, fb.lLegId])

pRH = SE3.Identity()
pRH = rotatePlacement(pRH, 'y', -0.8115781021773631)
pLH = pRH.copy()

#pLH.translation = np.matrix([-2.635,-1.235,1.13]).T
#pRH.translation = np.matrix([-2.635,-2.28,1.13]).T
pLH.translation = array([-2.842, -1.235, 0.91])
pRH.translation = array([-2.842, -2.28, 0.91])

cs.createContact(fb.rhand, ContactPatch(pRH))
cs.createContact(fb.lhand, ContactPatch(pLH))

displacement = SE3.Identity()
displacement.translation = array([0.15, 0, 0])

cs.moveEffectorOf(fb.rfoot, displacement)
cs.moveEffectorOf(fb.lfoot, displacement)

num_steps = 1
step_height = 0.22
step_width = 0.207

gait = [fb.rhand, fb.lhand, fb.rfoot, fb.lfoot]
displacement.translation = array([step_width, 0, step_height])
for i in range(num_steps):
    for eeName in gait:
        cs.moveEffectorOf(eeName, displacement )

cs.breakContact(fb.rhand)
cs.breakContact(fb.lhand)

q_end = q_ref[::]
q_end[0] += 0.15 + num_steps * step_width
q_end[2] += num_steps * step_height
fb.setCurrentConfig(q_end)
com = fb.getCenterOfMass()
setFinalState(cs, com, q=q_end)

display_tools.displaySteppingStones(cs, gui, sceneName, fb)


DEMO_NAME = "talos_stairsAirbus"
filename = cfg.CONTACT_SEQUENCE_PATH + "/" + DEMO_NAME + ".cs"
print("Write contact sequence binary file : ", filename)
cs.saveAsBinary(filename)
