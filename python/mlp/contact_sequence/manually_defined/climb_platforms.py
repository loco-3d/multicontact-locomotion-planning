from mlp.utils.cs_tools import addPhaseFromConfig, createPhaseFromConfig
import multicontact_api
from multicontact_api import ContactSequence
import mlp.viewer.display_tools as display_tools
from pinocchio import SE3
from numpy import array
from mlp.utils.cs_tools import walk
from talos_rbprm.talos import Robot  # change robot here
multicontact_api.switchToNumpyArray()

ENV_NAME = "multicontact/plateforme_surfaces_noscale"

fb, v = display_tools.initScene(Robot, ENV_NAME, False)
gui = v.client.gui
sceneName = v.sceneName
cs = ContactSequence(0)

cs_platforms = ContactSequence(0)
cs_platforms.loadFromBinary("talos_plateformes.cs")
p0_platform = cs_platforms.contactPhases[0]

q = [   0.0,
        0.0,
        1.02127,
        0.0,
        0.0,
        0.0,
        1.,  # Free flyer
        0.0,
        0.0,
        -0.411354,
        0.859395,
        -0.448041,
        -0.001708,  # Left Leg
        0.0,
        0.0,
        -0.411354,
        0.859395,
        -0.448041,
        -0.001708,  # Right Leg
        0.0,
        0.006761,  # Chest
        0.40,
        0.24,
        -0.6,
        -1.45,
        0.0,
        -0.0,
        0.,
        -0.005,  # Left Arm
        -0.4,
        -0.24,
        0.6,
        -1.45,
        0.0,
        0.0,
        0.,
        -0.005,  # Right Arm
        0.,
        0.
    ] + [0]*6

q[:2] = [-0.15, 0.25]
addPhaseFromConfig(fb, cs, q, [fb.rLegId, fb.lLegId])
q[:3] = [0.11, 0.25, 1.18127]
p_up = createPhaseFromConfig(fb, q, [fb.rLegId, fb.lLegId])

#make the first step to the platform
cs.moveEffectorToPlacement(fb.rfoot,p_up.contactPatch(fb.rfoot).placement)
cs.moveEffectorToPlacement(fb.lfoot,p_up.contactPatch(fb.lfoot).placement)
# Make the second step to connect the cs_plateform
cs.moveEffectorToPlacement(fb.rfoot,p0_platform.contactPatch(fb.rfoot).placement)
cs.moveEffectorToPlacement(fb.lfoot,p0_platform.contactPatch(fb.lfoot).placement)

# copy the cs_plateform

for phase in cs_platforms.contactPhases[1:]:
    cs.append(phase)


## Add final step :
q[0] = 1.14
p_end = createPhaseFromConfig(fb, q, [fb.rLegId, fb.lLegId])
q[:3] = [1.37, 0.25, 1.02127]
p_floor = createPhaseFromConfig(fb, q, [fb.rLegId, fb.lLegId])

#make a step to got to the edge of the platform
cs.moveEffectorToPlacement(fb.rfoot,p_end.contactPatch(fb.rfoot).placement)
cs.moveEffectorToPlacement(fb.lfoot,p_end.contactPatch(fb.lfoot).placement)
#make the last step on the floor
cs.moveEffectorToPlacement(fb.rfoot,p_floor.contactPatch(fb.rfoot).placement)
cs.moveEffectorToPlacement(fb.lfoot,p_floor.contactPatch(fb.lfoot).placement)

display_tools.displaySteppingStones(cs, gui, sceneName, fb)
filename = "talos_platformes_full.cs"
print("Write contact sequence binary file : ", filename)
cs.saveAsBinary(filename)
