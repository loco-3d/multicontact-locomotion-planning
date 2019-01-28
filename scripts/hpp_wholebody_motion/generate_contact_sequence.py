import pinocchio as se3
from pinocchio import SE3, Quaternion
from pinocchio.utils import *
from spline import bezier
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
    if extraDOF <= 0 :
        extraDOF = None
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
        phase_d.reference_configurations.append(np.matrix((configs[config_id][:-extraDOF])).T)        
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
         
        phase_s.reference_configurations.append(np.matrix((configs[config_id][:-extraDOF])).T)
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
    phase_d.reference_configurations.append(np.matrix((configs[-1][:-extraDOF])).T)
    init_state = phase_d.init_state
    init_state[0:3] = np.matrix(fb.getCenterOfMass()).transpose()
    init_state[3:9] = np.matrix(configs[-1][-6:]).transpose()
    final_state = init_state.copy()
    #phase_d.time_trajectory.append(0.)
    phase_d.init_state=init_state
    phase_d.final_state=final_state   
    #print "done for last state"
        
    return cs

## straight line from the center of the support polygon of the current phase to the next one
def generateGeometricInitGuess(cs_origin):
    cs = ContactSequenceHumanoid(cs_origin)
    p0 = cs.contact_phases[0]
    com_z = p0.init_state[2,0]
    previous_phase = None
    # first, compute the new init/final position for each state : in the center of the support polygon
    for pid in range(1,cs.size()-1):
        phase = cs.contact_phases[pid] 
        state = phase.init_state
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
        state[0] = com_x
        state[1] = com_y
        state[2] = com_z
        phase.init_state = state
        #print "phase : "+str(pid)+" com Init = "+str(com_x)+","+str(com_y)+","+str(com_z)
        if previous_phase != None :
            previous_phase.final_state = phase.init_state.copy()
        previous_phase = cs.contact_phases[pid]        
    
    # then, generate a straight line from init_state to final_state for each phase :
    t_total = 0.    
    for pid in range(cs.size()):
        phase = cs.contact_phases[pid] 
        duration = 0.
        if phase.numActivePatches() == 1:
            duration = cfg.DURATION_SS
        if phase.numActivePatches() == 2:
            duration = cfg.DURATION_DS        
        if phase.numActivePatches() == 3:
            duration = cfg.DURATION_TS        
        if phase.numActivePatches() == 4:
            duration = cfg.DURATION_QS        
        if phase.numActivePatches() > 4:
            raise Exception("Case not implemented")
        if pid == 0:
            duration = cfg.DURATION_INIT
        if pid == (cs.size()-1):
            duration = cfg.DURATION_FINAL
        com0 = phase.init_state[0:3]
        com1 = phase.final_state[0:3]
        vel = (com1-com0)/duration
        am = np.matrix(np.zeros(3)).T
        t = 0.
        while t < duration :
            u = t/duration
            state = np.matrix(np.zeros(9)).T            
            com = com0*(1.-u) + com1*(u)
            state[0:3] = com
            state[3:6] = vel
            state[6:9] = am
            phase.state_trajectory.append(state)
            phase.time_trajectory.append(t_total)
            t += cfg.SOLVER_DT
            t_total +=cfg.SOLVER_DT
        state[0:3] = com1
        state[3:6] = vel
        state[6:9] = am
        phase.state_trajectory.append(state)
        phase.time_trajectory.append(t_total)      
    return cs
"""
for id_state in range(beginId,endId):
    print "id_state = ",str(id_state)
    pid = fb.isDynamicallyReachableFromState(id_state,id_state+1,True,numPointsPerPhases=0)
    if len(pid) != 4:
        print "Cannot compute qp initial guess for state "+str(id_state)
        return generateContactSequence(fb,configs,beginId,endId,viewer)
    c_qp = fb.getPathAsBezier(int(pid[0]))
    t_qp = [ps.pathLength(int(pid[1])), ps.pathLength(int(pid[2])), ps.pathLength(int(pid[3]))]
    curves_initGuess.append(c_qp)
    timings_initGuess.append(t_qp)
 """   

def initGuessForPhaseFromBezier(phase,c,current_t,start,end,append = False):
    from spline import bezier    
    #print "start = ",start
    assert start >= 0 and start < c.max(), "t start invalid"
    #print "end = ",end
    assert end > 0 and end <= c.max(), "t end invalid"
    dc = c.compute_derivate(1)
    ddc = dc.compute_derivate(1)
    if not append:
        state = phase.init_state
        state[0:3] = c(start)
        phase.init_state = state
    state = phase.final_state
    state[0:3] = c(end)
    phase.final_state = state
    length = end - start
    nsteps = int(round(length/cfg.SOLVER_DT))
    #print "num steps : ",nsteps
    assert abs(nsteps - length/cfg.SOLVER_DT) < 1e-10 , "bezier curve length should be a multiple of the solver time step, t = "+str(c.max())+" dt = "+str(cfg.SOLVER_DT)
    if append:
        assert phase.time_trajectory[-1] == current_t ," called method with append = True and unconsistent phase time_trajectory"
    else :
        assert len(phase.time_trajectory) == 0 and len(phase.state_trajectory) == 0 , "called method with append = False and phase trajectory is not empty"
    t = start
    if append :
        t += cfg.SOLVER_DT
        current_t += cfg.SOLVER_DT
    else :
        nsteps += 1
    for i in range(nsteps):
        #print "t curve = ",t
        #print "t tot = ",current_t + i*cfg.SOLVER_DT
        x = np.matrix(np.zeros(9)).T
        x[0:3] = c(t)
        x[3:6] = dc(t)
        #x[6:9] #am
        u = np.matrix(np.zeros(6)).T
        u[0:3] = ddc(t)
        #u[3:6] # am variation
        phase.state_trajectory.append(x)
        phase.control_trajectory.append(u)
        phase.time_trajectory.append(current_t + i*cfg.SOLVER_DT)
        t += cfg.SOLVER_DT
        if t > c.max():
            t = c.max() # may happend due to numerical imprecisions
        
def generateCROCinitGuess(cs_origin,fb,beginId,endId):
    cs = ContactSequenceHumanoid(cs_origin)
    # for each phase in the cs, create a corresponding FullBody State and call CROC,
    # then discretize the solution and fill the cs struct
    # Make the assumption that the CS was created with the generateContactSequence method from the same fb object
    for id_state in range(beginId,endId):
        #print "id_state = ",str(id_state)
        pid = fb.isDynamicallyReachableFromState(id_state,id_state+1,True,numPointsPerPhases=0)
        if len(pid) != 4:
            print "Cannot compute qp initial guess for state "+str(id_state)
            return cs_origin
        if id_state == 0:
            append = False
            t =0.
        else :
            append = True
            t = cs.contact_phases[id_state*2].time_trajectory[-1]
        c = fb.getPathAsBezier(int(pid[0]))
        assert c.min() == 0 ,"bezier curve should start at t=0."     
        start = 0
        end = fb.client.problem.pathLength(int(pid[1]))
        #print "first DS phase"
        initGuessForPhaseFromBezier(cs.contact_phases[id_state*2], c,t,start,end,append)
        start = end
        end += fb.client.problem.pathLength(int(pid[2]))
        #print " SS phase"
        initGuessForPhaseFromBezier(cs.contact_phases[id_state*2+1], c,cs.contact_phases[id_state*2].time_trajectory[-1],start,end)
        start = end
        end += fb.client.problem.pathLength(int(pid[3])) 
        assert abs(end - c.max()) < 1e-10 ,"Error in computation of time interval"
        #print "second DS phase"        
        initGuessForPhaseFromBezier(cs.contact_phases[id_state*2+2], c,cs.contact_phases[id_state*2+1].time_trajectory[-1],start,end)
        
    return cs
     
        
