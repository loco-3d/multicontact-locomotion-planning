import numpy as np
import mlp.config as cfg
import multicontact_api
from multicontact_api import WrenchCone, SOC6, ContactPatch, ContactPhase, ContactSequence
from mlp.utils.requirements import Requirements
import math
VERBOSE = False
multicontact_api.switchToNumpyArray()

class Inputs(Requirements):
    timings = True
    COMvalues = True

class Outputs(Inputs):
    COMtrajectories = True


## straight line from the center of the support polygon of the current phase to the next one
def generateCentroidalTrajectory(cs, cs_initGuess=None, fullBody=None, viewer=None):
    if cs_initGuess:
        print("WARNING : in centroidal.geometric, initial guess is ignored.")
    cs_result = ContactSequence(cs)
    p0 = cs_result.contactPhases[0]
    com_z = p0.init_state[2]
    previous_phase = None
    # first, compute the new init/final position for each state : in the center of the support polygon
    for pid in range(1, cs_result.size() - 1):
        phase = cs_result.contactPhases[pid]
        state = phase.init_state
        com_x = 0.
        com_y = 0.
        if phase.LF_patch.active:
            com_x += phase.LF_patch.placement.translation[0]
            com_y += phase.LF_patch.placement.translation[1]
        if phase.RF_patch.active:
            com_x += phase.RF_patch.placement.translation[0]
            com_y += phase.RF_patch.placement.translation[1]
        if phase.LH_patch.active:
            com_x += phase.LH_patch.placement.translation[0]
            com_y += phase.LH_patch.placement.translation[1]
        if phase.RH_patch.active:
            com_x += phase.RH_patch.placement.translation[0]
            com_y += phase.RH_patch.placement.translation[1]
        com_x /= phase.numActivePatches()
        com_y /= phase.numActivePatches()
        # test : take com height from config found from planning :
        com_z = cs_result.contactPhases[pid].init_state[2]
        state[0] = com_x
        state[1] = com_y
        state[2] = com_z
        phase.init_state = state
        #print "phase : "+str(pid)+" com Init = "+str(com_x)+","+str(com_y)+","+str(com_z)
        if previous_phase != None:
            previous_phase.final_state = phase.init_state.copy()
        previous_phase = cs_result.contactPhases[pid]

    # then, generate a straight line from init_state to final_state for each phase :
    t_total = 0.
    for pid in range(cs_result.size()):
        phase = cs_result.contactPhases[pid]
        duration = computePhaseDuration(cs_result, pid)
        com0 = phase.init_state[0:3]
        com1 = phase.final_state[0:3]
        vel = (com1 - com0) / duration
        am = np.zeros(3)
        t = 0.
        while t < duration - 0.0001:
            u = t / duration
            state = np.zeros(9)
            com = com0 * (1. - u) + com1 * (u)
            state[0:3] = com
            state[3:6] = vel
            state[6:9] = am
            phase.state_trajectory.append(state)
            phase.time_trajectory.append(t_total)
            t += cfg.SOLVER_DT
            t_total += cfg.SOLVER_DT
        state[0:3] = com1
        state[3:6] = vel
        state[6:9] = am
        phase.state_trajectory.append(state)
        phase.time_trajectory.append(t_total)
    return cs_result
