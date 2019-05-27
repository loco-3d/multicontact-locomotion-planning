import pinocchio as pin
from pinocchio import SE3, Quaternion
from pinocchio.utils import *
import inspect
import mlp.config as cfg
import multicontact_api
from multicontact_api import WrenchCone,SOC6,ContactPatch, ContactPhaseHumanoid, ContactSequenceHumanoid
global i_sphere 
from mlp.utils.util import quatFromConfig,copyPhaseContacts,copyPhaseContactPlacements,contactPatchForEffector
import importlib

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
    return cp.fullBody,cp.v,cp.configs,beginId,endId

def contactSequenceFromRBPRMConfigs(fb,configs,beginId,endId):
    print "generate contact sequence from planning : "
    global i_sphere
    i_sphere = 0
    #print "MR offset",MRsole_offset
    #print "ML offset",MLsole_offset  
    n_double_support = len(configs)
    # config only contains the double support stance
    n_steps = n_double_support*2 -1 
    # Notice : what we call double support / simple support are in fact the state with all the contacts and the state without the next moving contact
    cs = ContactSequenceHumanoid(n_steps)
    unusedPatch = cs.contact_phases[0].LF_patch.copy()
    unusedPatch.placement = SE3.Identity()
    unusedPatch.active= False

    
    # for each contact state we must create 2 phase (one with all the contact and one with the next replaced contact(s) broken)
    for stateId in range(beginId,endId+1):
        # %%%%%%%%%  all the contacts : %%%%%%%%%%%%%
        cs_id = (stateId-beginId)*2
        config_id=stateId-beginId
        phase_d = cs.contact_phases[cs_id]
        fb.setCurrentConfig(configs[config_id])
        """
        init_guess_for_phase = init_guess_provided
        if init_guess_for_phase:
            c_init_guess = curves_initGuess[config_id]
            t_init_guess = timings_initGuess[config_id]
            init_guess_for_phase = isinstance(c_init_guess,bezier)
            if init_guess_for_phase:
                print "bezier curve provided for config id : "+str(config_id)
        """
        setContactActivityFromRBRMState(phase_d,fb,stateId)
        if stateId==beginId:
            setContactPlacementFromRBPRMState(phase_d,fb,stateId)            
        else :
            copyPhaseContactPlacements(phase_s,phase_d)            
            # now we change the contacts that have moved : 
            variations = fb.getContactsVariations(stateId-1,stateId)
            if len(variations) != 1:
                print "Several contact changes between states "+str(stateId-1)+" and "+str(stateId)+" : "+str(variations)
            assert len(variations)==1, "Several changes of contacts in adjacent states, not implemented yet !"
            setContactPlacementFromRBPRMState(phase_d,fb,stateId,variations)            
            
        # retrieve the COM position for init and final state (equal for double support phases)
        init_state = np.matrix(np.zeros(9)).T
        init_state[0:3] = np.matrix(fb.getCenterOfMass()).transpose()
        init_state[3:6] = np.matrix(configs[config_id][-6:-3]).transpose()
        final_state = init_state.copy()
        #phase_d.time_trajectory.append((fb.getDurationForState(stateId))*cfg.DURATION_n_CONTACTS/cfg.SPEED)
        phase_d.init_state=init_state
        phase_d.final_state=final_state
        phase_d.reference_configurations.append(np.matrix((configs[config_id])).T)        
        #print "done for double support"
        
        if stateId < endId :
            # %%%%%% simple support : %%%%%%%% 
            phase_s = cs.contact_phases[cs_id + 1]
            # copy previous placement :
            copyPhaseContacts(phase_d,phase_s)
            # find the contact to break : 
            variations = fb.getContactsVariations(stateId,stateId+1)
            if len(variations) != 1:
                print "Several contact changes between states "+str(stateId)+" and "+str(stateId+1)+" : "+str(variations)
            assert len(variations)==1, "Several changes of contacts in adjacent states, not implemented yet !"        
            for var in variations:
                patch = contactPatchForEffector(phase_s,fb.dict_limb_joint[var])
                patch.active = False
                
            # retrieve the COM position for init and final state     
            phase_s.reference_configurations.append(np.matrix((configs[config_id])).T)
            init_state = phase_d.init_state.copy()
            final_state = phase_d.final_state.copy()
            fb.setCurrentConfig(configs[config_id+1])
            final_state[0:3] = np.matrix(fb.getCenterOfMass()).transpose()
            final_state[3:6] = np.matrix(configs[config_id+1][-6:-3]).transpose()              
            #phase_s.time_trajectory.append((fb.getDurationForState(stateId))*(1-cfg.DURATION_n_CONTACTS)/cfg.SPEED) 
            phase_s.init_state=init_state.copy()
            phase_s.final_state=final_state.copy()
            #print "done for single support"      

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
    fb,viewer,configs,beginId,endId = runRBPRMScript()
    cs = contactSequenceFromRBPRMConfigs(fb,configs,beginId,endId)
    return cs,fb,viewer