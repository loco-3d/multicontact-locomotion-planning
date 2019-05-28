import numpy as np
import mlp.config as cfg
import multicontact_api 
from multicontact_api import WrenchCone,SOC6,ContactPatch, ContactPhaseHumanoid, ContactSequenceHumanoid
from hpp_spline import bezier
from mlp.utils.util import createStateFromPhase, phasesHaveSameConfig,createFullbodyStatesFromCS

def writeTrajInPhase(phase,c,current_t,start,end,append = False):
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


# Not generic .... assume that there is always 2 contact phases for each state in fullBody        
def generateCentroidalTrajectory(cs,cs_initGuess = None, fb = None, viewer = None):
    if cs_initGuess :
        print "WARNING : in centroidal.croc, initial guess is ignored."
    if not fb : 
        raise ValueError("CROC called without fullBody object.")
    beginId,endId = createFullbodyStatesFromCS(cs,fb)
    print "beginid = ",beginId
    print "endId   = ",endId
    cs_result = ContactSequenceHumanoid(cs)
    if (2*(endId - beginId)+1) != cs.size():
        raise NotImplemented("Current implementation of CROC require to have 2 contact phases per States in fullBody")
    # for each phase in the cs, create a corresponding FullBody State and call CROC,
    # then discretize the solution and fill the cs struct
    # Make the assumption that the CS was created with the generateContactSequence method from the same fb object
    id_phase = 0 
    for id_state in range(beginId,endId):
        print "id_state = ",str(id_state)
        print "id_phase = ",str(id_phase)
        # First, compute the bezier curves between the states : 
        pid = fb.isDynamicallyReachableFromState(id_state,id_state+1,True,numPointsPerPhases=0)
        if len(pid) != 4:
            print "# ERROR : Cannot compute CROC between states "+str(id_state)+" , "+str(id_state+1)
            return cs
        c = fb.getPathAsBezier(int(pid[0]))
        # Now split and write the results in the correct contactPhases
        if id_phase == 0:
            append = False
            t =0.
        else :
            append = True
            t = cs_result.contact_phases[id_phase].time_trajectory[-1]        
        assert c.min() == 0 ,"bezier curve should start at t=0."  
        # First phase (second half of the trajectory of this phase if it's not the initial phase of the CS)        
        start = 0
        end = fb.client.problem.pathLength(int(pid[1]))
        writeTrajInPhase(cs_result.contact_phases[id_phase], c,t,start,end,append)
        #Second phase
        id_phase += 1 
        start = end
        end += fb.client.problem.pathLength(int(pid[2]))
        t = cs_result.contact_phases[id_phase-1].time_trajectory[-1]        
        writeTrajInPhase(cs_result.contact_phases[id_phase], c,t,start,end)
        # Third phase (only the first half of the trajectory, exept if it's the final contact phase of the CS)
        id_phase += 1 
        start = end
        end += fb.client.problem.pathLength(int(pid[3])) 
        t = cs_result.contact_phases[id_phase-1].time_trajectory[-1]
        assert abs(end - c.max()) < 1e-10 ,"Error in computation of time interval"
        writeTrajInPhase(cs_result.contact_phases[id_phase], c,t,start,end)
        
    return cs_result
     
