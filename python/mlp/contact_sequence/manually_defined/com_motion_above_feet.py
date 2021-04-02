from mlp.utils.cs_tools import addPhaseFromConfig, generateZeroAMreference
import multicontact_api
from multicontact_api import ContactSequence
import mlp.viewer.display_tools as display_tools
from pinocchio import SE3
from numpy import array, zeros
from ndcurves import piecewise, polynomial

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

q_ref = fb.referenceConfig[::] + [0] * 6

# display the configuration
v(q_ref)

# create a first contact phase corresponding to this configuration
limbsInContact = [fb.rLegId, fb.lLegId]  # define the limbs in contact for this first phase
addPhaseFromConfig(fb, cs, q_ref, limbsInContact)
phase = cs.contactPhases[0]

com = array(fb.getCenterOfMass())
phase.c_init = com
phase.c_final = com
phase.q_final = phase.q_init
# generate com motion :
t = 0
# first go above the right feet in 2 seconds
c0 = com.copy()
dc0 = zeros(3)
ddc0 = zeros(3)
c1 = phase.contactPatch('leg_right_sole_fix_joint').placement.translation
c1[2] = c0[2] - 0.03 # go down 3cm to avoid kinematic limits
dc1 = zeros(3)
ddc1 = zeros(3)
c_t_right = polynomial(c0, dc0, ddc0, c1, dc1, ddc1, t, t+2)
t = c_t_right.max()
c0 = c1.copy()
# o above the left feet in 3 seconds:
c1 = phase.contactPatch('leg_left_sole_fix_joint').placement.translation
c1[2] = c0[2]
c_t_left = polynomial(c0, dc0, ddc0, c1, dc1, ddc1, t, t+3)
t = c_t_left.max()
c0 = c1.copy()
# go back to the initial CoM position in 2 seconds
c1 = com
c_t_mid = polynomial(c0, dc0, ddc0, c1, dc1, ddc1, t, t+2)

c_t =  piecewise(c_t_right)
c_t.append(c_t_left)
c_t.append(c_t_mid)

phase.c_t = c_t
phase.dc_t = c_t.compute_derivate(1)
phase.ddc_t = c_t.compute_derivate(2)

# set the timings of the phase :
phase.timeInitial = c_t.min()
phase.timeFinal = c_t.max()

# set a 0 AM trajectory
generateZeroAMreference(cs)

assert cs.haveTimings()
assert cs.haveConsistentContacts()
assert cs.haveCentroidalValues()
assert cs.haveCentroidalTrajectories()

cs.saveAsBinary("com_motion_above_feet_COM.cs")

