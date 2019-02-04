import numpy as np
from numpy.linalg import norm
import os
import timeopt
import locomote
from locomote import WrenchCone,SOC6,ContactPatch, ContactPhaseHumanoid, ContactSequenceHumanoid
import time
import hpp_wholebody_motion.config as cfg
import hpp_wholebody_motion.viewer.display_tools as display

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
            k_id = 0
            p_id += 1
            cs_com.contact_phases[p_id].init_state = np.matrix(x)                
            appendOrReplace(cs_com.contact_phases[p_id].time_trajectory,k_id,tp.getTime(k))
            appendOrReplace(cs_com.contact_phases[p_id].control_trajectory,k_id,np.matrix(u))
            appendOrReplace(cs_com.contact_phases[p_id].state_trajectory,k_id,np.matrix(x))                
        k_id +=1
    return cs_com
        
# helper method to make the link between timeopt.EndEffector and cs.Patch        
def getPhasePatchforEE(phase,ee):
    if ee == timeopt.EndeffectorID.RF:
        return phase.RF_patch
    if ee == timeopt.EndeffectorID.LF:
        return phase.LF_patch
    if ee == timeopt.EndeffectorID.RH:
        return phase.RH_patch
    if ee == timeopt.EndeffectorID.LH:
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


def generateCentroidalTrajectory(cs,cs_initGuess = None, viewer =None):
    q_init = cs.contact_phases[0].reference_configurations[0].copy()
    num_phases = cs.size()
    ee_ids = [timeopt.EndeffectorID.RF, timeopt.EndeffectorID.LF,timeopt.EndeffectorID.RH,timeopt.EndeffectorID.LH]
    
    effectors_phases,size = extractAllEffectorsPhasesFromCS(cs,cs_initGuess,ee_ids)
    #print "final dic : ",effectors_phases
    print "final number of phases : ", size    
    # initialize timeopt problem : 
    tp = timeopt.problem(size)
    tp.setInitialCOM(cs.contact_phases[0].init_state[:3])
    p0= cs.contact_phases[0]
    for ee in ee_ids:
        patch = getPhasePatchforEE(p0,ee)
        tp.setInitialPose(patch.active, patch.placement.translation, patch.placement.rotation, ee)
    tp.setMass(cfg.MASS);#FIXME
    tp.getMass();
    tp.setFinalCOM(cs.contact_phases[-1].final_state[0:3])
    
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
    
    
    return fillCSFromTimeopt(cs,cs_initGuess,tp), tp
