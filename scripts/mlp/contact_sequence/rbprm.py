import pinocchio as pin
from pinocchio import SE3, Quaternion
from pinocchio.utils import *
import inspect
import mlp.config as cfg
import multicontact_api
from multicontact_api import ContactPhaseHumanoid, ContactSequenceHumanoid
from mlp.utils.util import quatFromConfig,copyPhaseContacts,copyPhaseContactPlacements,contactPatchForEffector
import importlib

VERBOSE = True

def setContactActivityFromRBRMState(phase,fb,stateId):
    for limbId in fb.limbs_names:
        eeName = fb.dict_limb_joint[limbId]
        patch = contactPatchForEffector(phase,eeName)
        patch.active = fb.isLimbInContact(limbId,stateId)
        
def setContactPlacementFromRBPRMState(phase,fb,stateId,limbs = None):
    if limbs == None:
        limbs = fb.limbs_names
    for limbId in limbs:
        eeName = fb.dict_limb_joint[limbId]
        if fb.isLimbInContact(limbId,stateId):
            [p,n] = fb.clientRbprm.rbprm.computeCenterOfContactAtStateForLimb(stateId,False,limbId)
            placement = SE3.Identity()
            placement.translation = np.matrix(p).T
            if fb.cType == "_3_DOF":
                normal = np.matrix(n).T
                quat = Quaternion.FromTwoVectors(np.matrix(fb.dict_normal[eeName]).T,normal)
            else :
                q_r = fb.getJointPosition(eeName)
                quat = quatFromConfig(q_r)
            placement.rotation = quat.matrix()            
            patch = contactPatchForEffector(phase,eeName) 
            patch.placement = placement

def runRBPRMScript():
    #the following script must produce a sequence of configurations in contact (configs) 
    # with exactly one contact change between each configurations
    # It must also initialise a FullBody object name fullBody and optionnaly a Viewer object named V    
    scriptName = 'scenarios.'+cfg.SCRIPT_PATH+'.'+cfg.DEMO_NAME
    print "Run RBPRM script : ",scriptName
    cp = importlib.import_module(scriptName)
    if hasattr(cp,'beginId'):
        beginId = cp.beginId
    else :
        beginId = 0
    if hasattr(cp,'endId'):
        endId = cp.endId
    else :    
        endId = len(cp.configs) - 1    
    return cp.fullBody,cp.v,beginId,endId

def contactSequenceFromRBPRMConfigs(fb,beginId,endId):
    print "generate contact sequence from planning : "
    n_states = endId-beginId + 1 
    # There could be either contact break, creation or repositionning between each adjacent states. 
    # But there should be only contacts break or creation between each adjacent contactPhases
    cs = ContactSequenceHumanoid(0)
    prev_phase = None
    phaseId = 0
    
    # for each contact state we must create 2 phase (one with all the contact and one with the next replaced contact(s) broken)
    for stateId in range(beginId,endId+1):
        if VERBOSE :
            print "current state id = ",stateId
        # %%%%%%%%%  add phase with all the contacts in the rbprm State: %%%%%%%%%%%%%
        phase = ContactPhaseHumanoid()
        current_config = fb.getConfigAtState(stateId)        
        fb.setCurrentConfig(current_config)
        
        # Determine the contact changes which are going to occur : 
        if stateId < endId:
            next_variations = fb.getContactsVariations(stateId,stateId+1)
        else :
            next_variations = []
        if VERBOSE :
            print "variations : ",next_variations
        assert len(next_variations) <= 1 , "Several changes of contacts in adjacent states, not implemented yet !"
        if len(next_variations) == 0 :
            if VERBOSE:
                print "no variations !"
            movingLimb = None
            contact_break = False
            contact_create=False
        else:
            movingLimb = next_variations[0]
            contact_break = fb.isLimbInContact(movingLimb,stateId)
            contact_create = fb.isLimbInContact(movingLimb,stateId+1)
        if contact_break and contact_create:
            contact_reposition = True
        else:
            contact_reposition = False
        if VERBOSE : 
            print "movingLimb = ",movingLimb
            print "break = "+str(contact_break)+ " ; create = "+str(contact_create)+ " ; reposition = "+str(contact_reposition)
        
        if movingLimb or stateId == endId: # add a ContactPhase corresponding to the current state
            # Build the phase contact patches (placement and activity)
            setContactActivityFromRBRMState(phase,fb,stateId)
            if not prev_phase:
                setContactPlacementFromRBPRMState(phase,fb,stateId)            
            else :
                copyPhaseContactPlacements(prev_phase,phase)            
                # now we change the contacts that have moved : 
                previous_variations = fb.getContactsVariations(stateId-1,stateId)
                setContactPlacementFromRBPRMState(phase,fb,stateId,previous_variations)            
                
            # assign current wholebody config as reference in phase : 
            phase.reference_configurations.append(np.matrix((current_config)).T)                    
            # retrieve the COM position for init and final state
            init_state = np.matrix(np.zeros(9)).T
            init_state[0:3] = np.matrix(fb.getCenterOfMass()).transpose()
            init_state[3:6] = np.matrix(current_config[-6:-3]).transpose()
            phase.init_state=init_state
            final_state = init_state.copy()
            if not contact_reposition and stateId < endId : # in the case of contact reposition, the CoM motion will take place in the next intermediate phase. For the current phase the init and final state are equals
                current_config = fb.getConfigAtState(stateId+1) 
                fb.setCurrentConfig(current_config)
                final_state[0:3] = np.matrix(fb.getCenterOfMass()).transpose()
                final_state[3:6] = np.matrix(current_config[-6:-3]).transpose()                 
            phase.final_state=final_state
            
            # add phase to contactSequence :
            cs.contact_phases.append(phase)
            phaseId += 1
            prev_phase = phase 
            if VERBOSE:
                print "add a phase at id : ",phaseId-1
            if contact_reposition :
                # %%%%%% create intermediate state, by removing the contact repositionned betwen stateId and stateId+1 %%%%%%%% 
                phase = ContactPhaseHumanoid()
                # copy previous placement :
                copyPhaseContacts(prev_phase,phase)
                # find the contact to break : 
                patch = contactPatchForEffector(phase,fb.dict_limb_joint[movingLimb])
                patch.active = False
                # assign current wholebody config as reference in phase :
                phase.reference_configurations.append(np.matrix((current_config)).T)                 
                # retrieve the COM position for init and final state     
                phase.init_state = prev_phase.final_state.copy()
                final_state = phase.init_state.copy()
                current_config = fb.getConfigAtState(stateId+1) 
                fb.setCurrentConfig(current_config)
                final_state[0:3] = np.matrix(fb.getCenterOfMass()).transpose()
                final_state[3:6] = np.matrix(current_config[-6:-3]).transpose()              
                phase.final_state=final_state.copy()
                # add phase to contactSequence :
                cs.contact_phases.append(phase)
                phaseId += 1
                prev_phase = phase 
                if VERBOSE :
                    print "add an intermediate contact phase at id : ",phaseId-1
            # end adding intermediate contact phase
        # end if movingLimb or stateId == endId
    # end for each stateId
    # assign contact models : 
    # only used by muscod ?? But we need to fill it otherwise the serialization fail
    for k,phase in enumerate(cs.contact_phases):
        RF_patch = phase.RF_patch
        cm = RF_patch.contactModel
        cm.mu = cfg.MU
        cm.ZMP_radius = 0.01
        RF_patch.contactModelPlacement = SE3.Identity()
        LF_patch = phase.LF_patch
        cm = LF_patch.contactModel
        cm.mu = cfg.MU
        cm.ZMP_radius = 0.01
        LF_patch.contactModelPlacement = SE3.Identity()
        LH_patch = phase.LH_patch
        cm = LH_patch.contactModel
        cm.mu = cfg.MU
        cm.ZMP_radius = 0.01
        LH_patch.contactModelPlacement = SE3.Identity()
        RH_patch = phase.RH_patch            
        cm = RH_patch.contactModel
        cm.mu = cfg.MU
        cm.ZMP_radius = 0.01
        RH_patch.contactModelPlacement = SE3.Identity()    
        
    return cs

def generateContactSequence():
    fb,viewer,beginId,endId = runRBPRMScript()
    cs = contactSequenceFromRBPRMConfigs(fb,beginId,endId)
    return cs,fb,viewer