import pinocchio as se3
from pinocchio import SE3, Quaternion
from pinocchio.utils import *
import inspect
import hpp_wholebody_motion.config as cfg
import locomote
from locomote import WrenchCone,SOC6,ContactPatch, ContactPhaseHumanoid, ContactSequenceHumanoid
global i_sphere 

def quatFromConfig(q):
    return Quaternion(q[6],q[3],q[4],q[5])

def generateContactSequence(fb,configs,beginId,endId):
    print "generate contact sequence from planning : "
    global i_sphere
    i_sphere = 0
    #print "MR offset",MRsole_offset
    #print "ML offset",MLsole_offset  
    n_double_support = len(configs)
    # config only contains the double support stance
    n_steps = n_double_support*2 -1 
    # Notice : what we call double support / simple support are in fact the state with all the contacts and the state without the next moving contact
    extraDOF = int(fb.client.robot.getDimensionExtraConfigSpace())
    configSize = fb.client.robot.getConfigSize() - extraDOF
    cs = ContactSequenceHumanoid(n_steps)
    unusedPatch = cs.contact_phases[0].LF_patch.copy()
    unusedPatch.placement = SE3.Identity()
    unusedPatch.active= False

    
    # for each contact state we must create 2 phase (one with all the contact and one with the next replaced contact(s) broken)
    for stateId in range(beginId,endId):
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
        
        # compute MRF and MLF : the position of the contacts
        q_rl = fb.getJointPosition(fb.rfoot)
        q_ll = fb.getJointPosition(fb.lfoot)
        q_rh = fb.getJointPosition(fb.rhand)
        q_lh = fb.getJointPosition(fb.lhand)
        
        
        # feets
        MRF = SE3.Identity()
        MLF = SE3.Identity()
        MRF.translation = np.matrix(q_rl[0:3]).T
        MLF.translation = np.matrix(q_ll[0:3]).T
        if not cfg.FORCE_STRAIGHT_LINE : 
            rot_rl = quatFromConfig(q_rl)
            rot_ll = quatFromConfig(q_ll)
            MRF.rotation = rot_rl.matrix()
            MLF.rotation = rot_ll.matrix()
        
        # apply the transform ankle -> center of contact
        MRF *= fb.MRsole_offset
        MLF *= fb.MLsole_offset
        
        # hands
        MRH = SE3()
        MLH = SE3()
        MRH.translation = np.matrix(q_rh[0:3]).T
        MLH.translation = np.matrix(q_lh[0:3]).T
        rot_rh = quatFromConfig(q_rh)
        rot_lh = quatFromConfig(q_lh)
        MRH.rotation = rot_rh.matrix()
        MLH.rotation = rot_lh.matrix()   
        
        MRH *= fb.MRhand_offset
        MLH *= fb.MLhand_offset    
        
        phase_d.RF_patch.placement = MRF
        phase_d.LF_patch.placement = MLF
        phase_d.RH_patch.placement = MRH
        phase_d.LH_patch.placement = MLH
        
        
        # initial state : Set all new contacts patch (either with placement computed below or unused)
        if stateId==beginId:
            # FIXME : for loop ? how ?
            if fb.isLimbInContact(fb.rLegId,stateId):
                phase_d.RF_patch.active = True
            else:
                phase_d.RF_patch.active = False
            if fb.isLimbInContact(fb.lLegId,stateId):
                phase_d.LF_patch.active = True
            else:
                phase_d.LF_patch.active = False
            if fb.isLimbInContact(fb.rArmId,stateId):
                phase_d.RH_patch.active = True
            else:
                phase_d.RH_patch.active = False
            if fb.isLimbInContact(fb.lArmId,stateId):
                phase_d.LH_patch.active = True
            else:
                phase_d.LH_patch.active = False
        else:   
            # we need to copy the unchanged patch from the last simple support phase (and not create a new one with the same placement)
            phase_d.RF_patch = phase_s.RF_patch
            phase_d.RF_patch.active = fb.isLimbInContact(fb.rLegId,stateId)
            phase_d.LF_patch = phase_s.LF_patch
            phase_d.LF_patch.active = fb.isLimbInContact(fb.lLegId,stateId)
            phase_d.RH_patch = phase_s.RH_patch
            phase_d.RH_patch.active = fb.isLimbInContact(fb.rArmId,stateId)
            phase_d.LH_patch = phase_s.LH_patch
            phase_d.LH_patch.active = fb.isLimbInContact(fb.lArmId,stateId)
            
            # now we change the contacts that have moved : 
            variations = fb.getContactsVariations(stateId-1,stateId)
            if len(variations) != 1:
                print "Several contact changes between states "+str(stateId-1)+" and "+str(stateId)+" : "+str(variations)
            assert len(variations)==1, "Several changes of contacts in adjacent states, not implemented yet !"
            for var in variations:     
                # FIXME : for loop in variation ? how ?
                if var == fb.lLegId:
                    phase_d.LF_patch.placement = MLF
                if var == fb.rLegId:
                    phase_d.RF_patch.placement = MRF
                if var == fb.lArmId:
                    phase_d.LH_patch.placement = MLH
                if var == fb.rArmId:
                    phase_d.RH_patch.placement = MRH
                    
        # retrieve the COM position for init and final state (equal for double support phases)
        init_state = phase_d.init_state.copy()
        init_state[0:3] = np.matrix(fb.getCenterOfMass()).transpose()
        init_state[3:9] = np.matrix(configs[config_id][-6:]).transpose()
        final_state = init_state.copy()
        #phase_d.time_trajectory.append((fb.getDurationForState(stateId))*cfg.DURATION_n_CONTACTS/cfg.SPEED)
        phase_d.init_state=init_state
        phase_d.final_state=final_state
        phase_d.reference_configurations.append(np.matrix((configs[config_id][:configSize])).T)        
        #print "done for double support"
        
        
        # %%%%%% simple support : %%%%%%%% 
        phase_s = cs.contact_phases[cs_id + 1]
        # copy previous placement :
        phase_s.RF_patch = phase_d.RF_patch
        phase_s.LF_patch = phase_d.LF_patch
        phase_s.RH_patch = phase_d.RH_patch
        phase_s.LH_patch = phase_d.LH_patch 
        # find the contact to break : 
        variations = fb.getContactsVariations(stateId,stateId+1)
        if len(variations) != 1:
            print "Several contact changes between states "+str(stateId)+" and "+str(stateId+1)+" : "+str(variations)
        assert len(variations)==1, "Several changes of contacts in adjacent states, not implemented yet !"        
        for var in variations:
            if var == fb.lLegId:
                phase_s.LF_patch.active = False
            if var == fb.rLegId:
                phase_s.RF_patch.active = False
            if var == fb.lArmId:
                phase_s.LH_patch.active = False
            if var == fb.rArmId:
                phase_s.RH_patch.active = False
        # retrieve the COM position for init and final state 
         
        phase_s.reference_configurations.append(np.matrix((configs[config_id][:configSize])).T)
        init_state = phase_d.init_state.copy()
        final_state = phase_d.final_state.copy()
        fb.setCurrentConfig(configs[config_id+1])
        final_state[0:3] = np.matrix(fb.getCenterOfMass()).transpose()
        final_state[3:9] = np.matrix(configs[config_id+1][-6:]).transpose()              
        #phase_s.time_trajectory.append((fb.getDurationForState(stateId))*(1-cfg.DURATION_n_CONTACTS)/cfg.SPEED) 
        phase_s.init_state=init_state.copy()
        phase_s.final_state=final_state.copy()
        #print "done for single support"      
        
    # add the final double support stance : 
    phase_d = cs.contact_phases[n_steps-1]
    fb.setCurrentConfig(configs[n_double_support - 1])
    # compute MRF and MLF : the position of the contacts
    q_rl = fb.getJointPosition(fb.rfoot)
    q_ll = fb.getJointPosition(fb.lfoot)
    q_rh = fb.getJointPosition(fb.rhand)
    q_lh = fb.getJointPosition(fb.lhand)

    # feets
    MRF = SE3.Identity()
    MLF = SE3.Identity()
    MRF.translation = np.matrix(q_rl[0:3]).T
    MLF.translation = np.matrix(q_ll[0:3]).T
    if not cfg.FORCE_STRAIGHT_LINE :  
        rot_rl = quatFromConfig(q_rl)
        rot_ll = quatFromConfig(q_ll)
        MRF.rotation = rot_rl.matrix()
        MLF.rotation = rot_ll.matrix()
    # apply the transform ankle -> center of contact
    MRF *= fb.MRsole_offset
    MLF *= fb.MLsole_offset

    # hands
    MRH = SE3()
    MLH = SE3()
    MRH.translation = np.matrix(q_rh[0:3]).T
    MLH.translation = np.matrix(q_lh[0:3]).T
    rot_rh = quatFromConfig(q_rh)
    rot_lh = quatFromConfig(q_lh)
    MRH.rotation = rot_rh.matrix()
    MLH.rotation = rot_lh.matrix()   
    
    MRH *= fb.MRhand_offset
    MLH *= fb.MLhand_offset    
    
    # we need to copy the unchanged patch from the last simple support phase (and not create a new one with the same placement
    phase_d.RF_patch = phase_s.RF_patch
    phase_d.RF_patch.active = fb.isLimbInContact(fb.rLegId,endId)
    phase_d.LF_patch = phase_s.LF_patch
    phase_d.LF_patch.active = fb.isLimbInContact(fb.lLegId,endId)
    phase_d.RH_patch = phase_s.RH_patch
    phase_d.RH_patch.active = fb.isLimbInContact(fb.rArmId,endId)
    phase_d.LH_patch = phase_s.LH_patch
    phase_d.LH_patch.active = fb.isLimbInContact(fb.lArmId,endId)
    
    # now we change the contacts that have moved : 
    variations = fb.getContactsVariations(endId-1,endId)
    #assert len(variations)==1, "Several changes of contacts in adjacent states, not implemented yet !"
    for var in variations:     
        # FIXME : for loop in variation ? how ?
        if var == fb.lLegId:
            phase_d.LF_patch.placement = MLF
        if var == fb.rLegId:
            phase_d.RF_patch.placement = MRF
        if var == fb.lArmId:
            phase_d.LH_patch.placement = MLH
        if var == fb.rArmId:
            phase_d.RH_patch.placement = MRH
    # retrieve the COM position for init and final state (equal for double support phases)    
    phase_d.reference_configurations.append(np.matrix((configs[-1][:configSize])).T)
    init_state = phase_d.init_state
    init_state[0:3] = np.matrix(fb.getCenterOfMass()).transpose()
    init_state[3:9] = np.matrix(configs[-1][-6:]).transpose()
    final_state = init_state.copy()
    #phase_d.time_trajectory.append(0.)
    phase_d.init_state=init_state
    phase_d.final_state=final_state   
    #print "done for last state"
    
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
