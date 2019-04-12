import numpy as np
import mlp.config as cfg
import locomote 
from locomote import WrenchCone,SOC6,ContactPatch, ContactPhaseHumanoid, ContactSequenceHumanoid
from hpp_spline import bezier
 

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
       

        
def generateCentroidalTrajectory(cs_origin,fb,beginId,endId):
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
     
