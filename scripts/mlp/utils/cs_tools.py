from multicontact_api import ContactSequenceHumanoid, ContactPhaseHumanoid
from pinocchio import SE3, Quaternion
import mlp.viewer.display_tools as display_tools
from mlp.utils.util import SE3FromConfig,contactPatchForEffector,isContactActive,getContactPlacement
import numpy as np 
import types

def addPhaseFromConfig(fb,v,cs,q,limbsInContact):
    phase = ContactPhaseHumanoid()
    phase.reference_configurations.append(np.matrix((q)).T)
    v(q)
    fb.setCurrentConfig(q)
    state = np.matrix(np.zeros(9)).T
    state[0:3] = np.matrix(fb.getCenterOfMass()).T
    phase.init_state = state.copy()
    phase.final_state = state.copy()
    # set Identity to each contact placement (otherwise it's uninitialized)
    phase.RF_patch.placement = SE3.Identity()
    phase.LF_patch.placement = SE3.Identity()
    phase.RH_patch.placement = SE3.Identity()
    phase.LH_patch.placement = SE3.Identity()
    for limb in limbsInContact : 
        eeName = fb.dict_limb_joint[limb]
        q_j = fb.getJointPosition(eeName)
        patch = contactPatchForEffector(phase,eeName,fb)
        placement = SE3FromConfig(q_j)
        patch.placement = placement.act(fb.dict_offset[eeName])
        patch.active= True
        
    cs.contact_phases.append(phase)
    display_tools.displaySteppingStones(cs,v.client.gui,v.sceneName,fb)

def removeContact(Robot,cs,eeName):
    print "- Remove contact : ",eeName
    phase = ContactPhaseHumanoid(cs.contact_phases[-1])
    contactPatchForEffector(phase,eeName,Robot).active = False
    cs.contact_phases.append(phase)    

# add one or two contact phases to the sequence in order to move the effector eeName 
# to the desired placement
def moveEffectorToPlacement(Robot,v,cs,eeName,placement):
    print "## Move effector "+eeName+" to placement : "+str(placement.translation.T)
    prev_phase = cs.contact_phases[-1]
    if isContactActive(prev_phase,eeName,Robot):
        print "#Contact repositionning, add two new phases."
        removeContact(Robot,cs,eeName)
    print "+ Contact creation : ",eeName
    phase = ContactPhaseHumanoid(cs.contact_phases[-1])
    patch = contactPatchForEffector(phase,eeName,Robot)
    patch.placement = placement
    patch.active=True
    cs.contact_phases.append(phase)    
    display_tools.displaySteppingStones(cs,v.client.gui,v.sceneName,Robot)
    

# add one or two contact phases to the sequence in order to move the effector eeName of
#the value in translation
def moveEffectorOf(Robot,v,cs,eeName,translation):
    print "## Move effector "+eeName+" of : "+str(translation)
    prev_phase = cs.contact_phases[-1]
    assert isContactActive(prev_phase,eeName,Robot), "Cannot use 'moveEffectorOf' if the effector is not in contact in the last phase."
    placement = getContactPlacement(prev_phase,eeName,Robot).copy()
    if isinstance(translation,types.ListType):
        translation = np.matrix(translation).T
    assert translation.shape[0] == 3 ,"translation must be a 3D vector"
    placement.translation += translation
    moveEffectorToPlacement(Robot,v,cs,eeName,placement)

def setFinalState(cs,com = None,q=None):
    phase = cs.contact_phases[-1]
    if q:
        phase.reference_configurations[0] = np.matrix((q)).T
    if not com : 
        com_x = 0.
        com_y = 0.
        if phase.LF_patch.active :
            com_x += phase.LF_patch.placement.translation[0,0]
            com_y += phase.LF_patch.placement.translation[1,0]
        if phase.RF_patch.active :
            com_x += phase.RF_patch.placement.translation[0,0]
            com_y += phase.RF_patch.placement.translation[1,0]
        if phase.LH_patch.active :
            com_x += phase.LH_patch.placement.translation[0,0]
            com_y += phase.LH_patch.placement.translation[1,0]
        if phase.RH_patch.active :
            com_x += phase.RH_patch.placement.translation[0,0]
            com_y += phase.RH_patch.placement.translation[1,0]  
        com_x /= phase.numActivePatches()   
        com_y /= phase.numActivePatches() 
        com_z = phase.init_state[2]
        com = np.matrix([com_x,com_y,com_z]).T
    elif isinstance(com,types.ListType):
        com = np.matrix(com).T
    state = phase.init_state.copy()
    state[0:3] = com
    phase.final_state = state
    
# generate a walking motion from the last phase in the contact sequence.
# the contacts will be moved in the order of the 'gait' list. With the first one move only of half the stepLength
# TODO : make it generic ! it's currently limited to motion in the x direction
def walk(Robot,v,cs,distance,stepLength,gait):
    prev_phase = cs.contact_phases[-1]
    for limb in gait : 
        eeName = Robot.dict_limb_joint[limb]
        assert isContactActive(prev_phase,eeName,Robot), "All limbs in gait should be in contact in the first phase"
    isFirst = True
    reached = False
    remainingDistance = distance
    while remainingDistance >= 0 :
        for limb in gait : 
            if isFirst:
                length = stepLength/2.
                isFirst = False
            else : 
                length = stepLength
            if length > remainingDistance:
                length = remainingDistance
            translation = [length,0,0]
            moveEffectorOf(Robot,v,cs,Robot.dict_limb_joint[limb],translation)
        remainingDistance -= stepLength
    translation = [stepLength/2.,0,0]    
    moveEffectorOf(Robot,v,cs,Robot.dict_limb_joint[gait[0]],translation) 
    q_end = Robot.referenceConfig[::]+[0]*6
    q_end[0] += distance
    setFinalState(cs,q=q_end)
