import numpy as np
import hpp_wholebody_motion.config as cfg
import locomote
from locomote import WrenchCone,SOC6,ContactPatch, ContactPhaseHumanoid, ContactSequenceHumanoid

## straight line from the center of the support polygon of the current phase to the next one
def generateCentroidalTrajectory(cs_origin):
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
