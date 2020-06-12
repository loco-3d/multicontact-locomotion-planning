from mlp.utils.cs_tools import addPhaseFromConfig, setFinalState
from mlp.utils.util import genCOMTrajFromPhaseStates, genAMTrajFromPhaseStates
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
phase = cs.contactPhases[0]
c_mid = array(fb.getCenterOfMass())
c_r =  phase.contactPatch('leg_right_sole_fix_joint').placement.translation
c_r[2] = c_mid[2] - 0.02
c_l =  phase.contactPatch('leg_left_sole_fix_joint').placement.translation
c_l[2] = c_mid[2] - 0.02

transform = SE3.Identity()
cs.moveEffectorOf(fb.rfoot, transform)
cs.contactPhases[-3].c_final = c_l
cs.contactPhases[-2].c_init = c_l
cs.contactPhases[-2].c_final = c_l
cs.contactPhases[-1].c_init = c_l
cs.moveEffectorOf(fb.lfoot, transform)
cs.contactPhases[-3].c_final = c_r
cs.contactPhases[-2].c_init = c_r
cs.contactPhases[-2].c_final = c_r
cs.contactPhases[-1].c_init = c_r
cs.moveEffectorOf(fb.rfoot, transform)
cs.contactPhases[-3].c_final = c_l
cs.contactPhases[-2].c_init = c_l
cs.contactPhases[-2].c_final = c_l
cs.contactPhases[-1].c_init = c_l
cs.moveEffectorOf(fb.lfoot, transform)
cs.contactPhases[-3].c_final = c_r
cs.contactPhases[-2].c_init = c_r
cs.contactPhases[-2].c_final = c_r
cs.contactPhases[-1].c_init = c_r
setFinalState(cs,com, q_ref)

cs.saveAsBinary("step_in_place_quasistatic.cs")

t = 0
for pid, phase in enumerate(cs.contactPhases):
    # define timing :
    phase.timeInitial = t
    if phase.numContacts() == 2:
        phase.timeFinal = t + 2. # DS duration
    else:
        phase.timeFinal = t + 1.5 ## SS duration
    t = phase.timeFinal
    # define init/end CoM position :
    if phase.numContacts() == 1:
        phase.c_init = cs.contactPhases[pid-1].c_final
        phase.c_final = cs.contactPhases[pid-1].c_final
    else:
        if pid > 0:
            phase.c_init = cs.contactPhases[pid-1].c_final
        if pid < cs.size() -1 :
            if cs.contactPhases[pid+1].isEffectorInContact(fb.lfoot):
                phase.c_final = c_l
            elif cs.contactPhases[pid+1].isEffectorInContact(fb.rfoot):
                phase.c_final = c_r
        else:
            phase.c_final = c_mid

for phase in cs.contactPhases:
    genCOMTrajFromPhaseStates(phase)
    genAMTrajFromPhaseStates(phase)


assert cs.haveConsistentContacts()
assert cs.haveConsistentContacts()
assert cs.haveCentroidalValues()
assert cs.haveCentroidalTrajectories()

cs.saveAsBinary("step_in_place_quasistatic_COM.cs")
