import numpy as np
from numpy.linalg import norm
import os
import timeopt
import locomote
from locomote import WrenchCone,SOC6,ContactPatch, ContactPhaseHumanoid, ContactSequenceHumanoid
import time
import hpp_wholebody_motion.config as cfg
import hpp_wholebody_motion.viewer.display_tools as display
from hpp_wholebody_motion.utils.util import *

CONTACT_ANKLE_LEVEL = True # probably only required for hrp2, as the center of the feet is not the center of the flexibility ...

def isContactEverActive(cs,eeName):
    for phase in cs.contact_phases:
        if eeName == timeopt.EndeffectorID.RF :
            if phase.RF_patch.active:
                return True
        elif eeName == timeopt.EndeffectorID.LF :
            if phase.LF_patch.active:
                return True
        elif eeName == timeopt.EndeffectorID.RH :
            if phase.RH_patch.active:
                return True
        elif eeName == timeopt.EndeffectorID.LH :
            if phase.LH_patch.active:
                return True
        else :
            raise Exception("Unknown effector name") 
    return False


## check if two given timeOpt adjacent indices belong to the same phase or not
# by checking the contact forces of each effector
def isNewPhaseFromContact(tp,k0,k1):
    isNew = False
    for i in range(4):
        if norm(tp.getContactForce(i, k0)) > 2. and norm(tp.getContactForce(i, k1)) <= 2.:
            isNew = True
        if norm(tp.getContactForce(i, k1)) > 2. and norm(tp.getContactForce(i, k0)) <= 2.:
            isNew = True
    return isNew 

# check if k is the first id of a new phase, by comparing to the lengths in the initial guess
def isNewPhaseFromCS(cs_com,cs_initGuess,p_id):
    """
    id = 0
    for phase in cs.contact_phases :
        if id == 0:
            id += len(phase.time_trajectory)
        else :
            id += len(phase.time_trajectory) - 1             
        if k1 == id :
            return True
        if k1 < id :
            return False
    return False
    """
    if len(cs_com.contact_phases[p_id].time_trajectory) == len(cs_initGuess.contact_phases[p_id].time_trajectory):
        return True
    else :
        return False
    
def isNewPhase(tp,k0,k1,cs_com,cs_initGuess,p_id):
    if cs_initGuess :
        return isNewPhaseFromCS(cs_com,cs_initGuess,p_id)
    else : 
        return isNewPhaseFromContact(tp,k0,k1)
        

## helper method to deal with StateVectorState and other exposed c++ vectors : 
# replace vec[i] with value if it already exist or append the value
def appendOrReplace(vec,i,value):
    assert len(vec) >= i , "There is an uninitialized gap in the vector."
    if i < len(vec) :
        vec[i] = value
    else :
        vec.append(value)
        
def fillCSFromTimeopt(cs,cs_initGuess,tp):
    cs_com = ContactSequenceHumanoid(cs)
    
    # extract infos from tp to fill cs.contact_phases struct
    u = [0] * 6
    x = [0] * 9
    MASS = tp.getMass()
    p_id = 0 # phase id in cs
    k_id = 1 # id in the current phase
    # tp.getTime(0) == dt !! not 0 
    p0 = cs_com.contact_phases[0]
    init_state = p0.init_state
    init_state[0:3] = tp.getInitialCOM()
    p0.init_state = init_state 
    state = p0.init_state
    appendOrReplace(p0.time_trajectory,0,0.)
    appendOrReplace(p0.state_trajectory,0,state)
    appendOrReplace(p0.control_trajectory,0,np.matrix(np.zeros(6)).T)
    for k in range(tp.getTrajectorySize()):
        appendOrReplace(cs_com.contact_phases[p_id].time_trajectory,k_id,tp.getTime(k))
        #extract x and u from tp : 
        if k == 0:
            u[0:3] = ((tp.getLMOM(k)/MASS) / tp.getTime(k)).tolist() # acceleration
            u[3:6] = ((tp.getAMOM(k))/(tp.getTime(k))).tolist() # angular momentum variation
        u[0:3] = (((tp.getLMOM(k)/MASS) - (tp.getLMOM(k-1)/MASS)) / (tp.getTime(k)-tp.getTime(k-1))).tolist()#acceleration
        u[3:6] = ((tp.getAMOM(k) - tp.getAMOM(k-1))/(tp.getTime(k)-tp.getTime(k-1))).tolist() # angular momentum variation
        #print "control = ",np.matrix(u)    
        x[0:3] = tp.getCOM(k).tolist() # position
        x[3:6] = (tp.getLMOM(k)/MASS).tolist() # velocity
        x[6:9] = tp.getAMOM(k).tolist() # angular momentum        
        
        appendOrReplace(cs_com.contact_phases[p_id].control_trajectory,k_id,np.matrix(u))
        appendOrReplace(cs_com.contact_phases[p_id].state_trajectory,k_id,np.matrix(x))
        
        if k > 0 and isNewPhase(tp,k-1,k,cs_com,cs_initGuess,p_id):
            #last k of current phase, first k of next one (same state_traj and time)
            # set final state of current phase : 
            cs_com.contact_phases[p_id].final_state = np.matrix(x)
            # first k of the current phase
            p_id += 1                            
            if p_id < cs_com.size()  :
                k_id = 0
                cs_com.contact_phases[p_id].init_state = np.matrix(x)                
                appendOrReplace(cs_com.contact_phases[p_id].time_trajectory,k_id,tp.getTime(k))
                appendOrReplace(cs_com.contact_phases[p_id].control_trajectory,k_id,np.matrix(u))
                appendOrReplace(cs_com.contact_phases[p_id].state_trajectory,k_id,np.matrix(x))                
        k_id +=1
    p_end = cs_com.contact_phases[-1]
    final_state = p_end.final_state
    final_state[0:3] = tp.getFinalCOM()
    p_end.final_state = final_state
    # apparently tp.get{COM,LMOM,AMOM}(tp.getTrajectorySize()-1) don't give the value of the last segment (but the previous one),
    # so we add a last point in the trajectories here : 
    appendOrReplace(p_end.time_trajectory,k_id,tp.getTime(tp.getTrajectorySize()-1) + cfg.SOLVER_DT)
    appendOrReplace(p_end.control_trajectory,k_id,np.matrix(np.zeros(6)).T)
    appendOrReplace(p_end.state_trajectory,k_id,final_state.copy()) 
    # because of this the last phase will not have the desired duration, but will be a dt longer ... 
    return cs_com
        
# helper method to make the link between timeopt.EndEffector and cs.Patch        
def getPhasePatchforEE(phase,ee):
    if ee == timeopt.EndeffectorID.RF:
        if CONTACT_ANKLE_LEVEL :
            return JointPatchForEffector(phase,cfg.Robot.rfoot)
        else:
            return phase.RF_patch
    if ee == timeopt.EndeffectorID.LF:
        if CONTACT_ANKLE_LEVEL :
            return JointPatchForEffector(phase,cfg.Robot.lfoot)
        else:
            return phase.LF_patch        
    if ee == timeopt.EndeffectorID.RH:
        if CONTACT_ANKLE_LEVEL :
            return JointPatchForEffector(phase,cfg.Robot.rhand)
        else:
            return phase.RH_patch        
    if ee == timeopt.EndeffectorID.LH:
        if CONTACT_ANKLE_LEVEL :
            return JointPatchForEffector(phase,cfg.Robot.lhand)
        else:
            return phase.LH_patch        
    
# extract a list of effector phase (t start, t end, placement) (as defined by timeopt) from cs struct
def extractEffectorPhasesFromCS(cs,ee):
    ee_phases = []    
    pid =0 
    # find a first active patch or given effector : 
    while pid < cs.size() :
        p = cs.contact_phases[pid]
        patch = getPhasePatchforEE(p,ee)        
        if patch.active: 
            previous_patch = patch
            t_start = p.time_trajectory[0]
            t_end = p.time_trajectory[-1]
            # find the last phase where the same patch is active
            pid += 1            
            while patch.active and patch.placement == previous_patch.placement and pid <= cs.size():                     
                t_end = p.time_trajectory[-1]
                if pid < cs.size():
                    p = cs.contact_phases[pid]            
                    patch = getPhasePatchforEE(p,ee)
                pid += 1                     
            pid -=1
            ee_phases += [[t_start,t_end,previous_patch.placement]]
        pid += 1
    return ee_phases

def extractEffectorPhasesFromCSWithoutInitGuess(cs,ee):
    #TODO : same as above but take hardcoded duration (depending on the type of phases instead of the one inside the trajectory)
    raise Exception("Not implemented yet.")

def addCOMviapoints(tp,cs,viewer = None) : 
    phase_previous =None
    for pid in range(1,cs.size()-1) : 
        phase = cs.contact_phases[pid]
        phase_previous = cs.contact_phases[pid-1]
        phase_next = cs.contact_phases[pid+1]
        if phase_previous and (phase_previous.numActivePatches() < phase.numActivePatches()) and phase_next and (phase_next.numActivePatches() < phase.numActivePatches()) : 
            com = phase.init_state[0 : 3]
            tp.setViapoint(phase.time_trajectory[0],com)
            if viewer and cfg.DISPLAY_WP_COST :
                display.displaySphere(viewer,com.T.tolist()[0])



def extractAllEffectorsPhasesFromCS(cs,cs_initGuess,ee_ids):
    effectors_phases = {}
    size = 0
    for ee in ee_ids:
        #print "looking for phases for effector : ",ee
        if cs_initGuess != None :
            ee_phases = extractEffectorPhasesFromCS(cs_initGuess,ee)
        else :
            ee_phases = extractEffectorPhasesFromCSWithoutInitGuess(cs,ee)
        #print ee_phases
        #print "num of phases : ",len(ee_phases)
        size += len(ee_phases)
        if len(ee_phases) > 0 :
            effectors_phases.update({ee:ee_phases})
    return effectors_phases,size

def copyPhaseContacts(phase_in,phase_out):
    phase_out.RF_patch = phase_in.RF_patch
    phase_out.LF_patch = phase_in.LF_patch
    phase_out.RH_patch = phase_in.RH_patch
    phase_out.LH_patch = phase_in.LH_patch
    
# fill state trajectory and time trajectory with a linear trajectory connecting init_state to final_state
# the trajectories vectors must be empty when calling this method ! 
# Note : don't fill control_traj for now
def generateLinearInterpTraj(phase,duration,t_total):
    com0 = phase.init_state[0:3]
    com1 = phase.final_state[0:3]
    vel = (com1-com0)/duration
    acc = np.matrix(np.zeros(3)).T
    L = np.matrix(np.zeros(3)).T
    dL = np.matrix(np.zeros(3)).T
    state = np.matrix(np.zeros(9)).T
    control = np.matrix(np.zeros(6)).T    
    t = 0.
    while t < duration - 0.0001 :
        u = t/duration
        com = com0*(1.-u) + com1*(u)
        state[0:3] = com
        state[3:6] = vel
        state[6:9] = L
        control[0:3] = acc
        control[3:6] = dL
        phase.state_trajectory.append(state)
        phase.control_trajectory.append(control)        
        phase.time_trajectory.append(t_total)
        t += cfg.SOLVER_DT
        t_total +=cfg.SOLVER_DT
    state[0:3] = com1
    state[3:6] = vel
    state[6:9] = L
    control[0:3] = acc
    control[3:6] = dL    
    phase.state_trajectory.append(state)
    phase.control_trajectory.append(control)            
    phase.time_trajectory.append(t_total)
    return phase


# add an initial and final phase that only move the COM along z from the given distance
def addInitAndGoalShift(cs_in):
    cs = cs_com = ContactSequenceHumanoid(cs_in.size()+2)
    # copy all phases but leave one phase at the beginning and at the end : 
    for k in range(cs_in.size()):
        phase = cs_in.contact_phases[k].copy()
        # shift times to take in account the new phase : 
        for i in range(len(phase.time_trajectory)):
            phase.time_trajectory[i] += cfg.TIME_SHIFT_COM
        cs.contact_phases[k+1] = phase
        
    # now add new first phase :    
    phase = cs.contact_phases[0]
    s_final = cs.contact_phases[1].init_state
    s_init = s_final.copy()
    s_init[2] -= cfg.COM_SHIFT_Z
    phase.init_state = s_init
    phase.final_state = s_final
    generateLinearInterpTraj(phase,cfg.TIME_SHIFT_COM,0)
    copyPhaseContacts(cs.contact_phases[1],phase)
    phase.reference_configurations = cs.contact_phases[1].reference_configurations
    # add the new final phase
    phase = cs.contact_phases[-1]
    s_init = cs.contact_phases[-2].final_state
    s_final = s_init.copy()
    s_final[2] -= cfg.COM_SHIFT_Z
    phase.init_state = s_init
    phase.final_state = s_final
    generateLinearInterpTraj(phase,cfg.TIME_SHIFT_COM,cs.contact_phases[-2].time_trajectory[-1])
    copyPhaseContacts(cs.contact_phases[-2],phase)   
    phase.reference_configurations = cs.contact_phases[-2].reference_configurations
    
    return cs
    

def generateCentroidalTrajectory(cs,cs_initGuess = None, viewer =None):
    q_init = cs.contact_phases[0].reference_configurations[0].copy()
    num_phases = cs.size()
    ee_ids = [timeopt.EndeffectorID.RF, timeopt.EndeffectorID.LF,timeopt.EndeffectorID.RH,timeopt.EndeffectorID.LH]
    
    effectors_phases,size = extractAllEffectorsPhasesFromCS(cs,cs_initGuess,ee_ids)
    #print "final dic : ",effectors_phases
    print "final number of phases : ", size    
    # initialize timeopt problem : 
    tp = timeopt.problem(size)
    com_init = cs.contact_phases[0].init_state[:3]
    vel_init = cs.contact_phases[0].init_state[3:6]
    com_end = cs.contact_phases[-1].final_state[:3]
    com_init[2] += cfg.COM_SHIFT_Z
    com_end[2] += cfg.COM_SHIFT_Z
    tp.setInitialCOM(com_init)
    tp.setInitialLMOM(vel_init*cfg.MASS)
    tp.setFinalCOM(com_end)    
    p0= cs.contact_phases[0]
    for ee in ee_ids:
        patch = getPhasePatchforEE(p0,ee)
        tp.setInitialPose(isContactEverActive(cs,ee), patch.placement.translation, patch.placement.rotation, ee)
    tp.setMass(cfg.MASS);#FIXME
    tp.getMass();
    
    #add all effector phases to the problem : 
    i = 0
    for ee in effectors_phases.keys():
        for phase in effectors_phases[ee]:
            tp.setPhase(i, timeopt.phase(ee, phase[0],  phase[1],  phase[2].translation,  phase[2].rotation))
            i += 1
    if cfg.USE_WP_COST:
        addCOMviapoints(tp,cs_initGuess,viewer)
    cfg_path=cfg.TIME_OPT_CONFIG_PATH + '/'+  cfg.TIMEOPT_CONFIG_FILE
    print "set configuration file for time-optimization : ",cfg_path
    tp.setConfigurationFile(cfg_path)
    tp.setTimeoptSolver(cfg_path)
    tStart = time.time()
    tp.solve()
    tTimeOpt = time.time() - tStart
    print "timeopt problem solved in : "+str(tTimeOpt)+" s"
    print "write results in cs"
    
    cs_result = fillCSFromTimeopt(cs,cs_initGuess,tp)
    if cfg.TIME_SHIFT_COM > 0 :
        cs_result = addInitAndGoalShift(cs_result)
    
    return cs_result, tp
