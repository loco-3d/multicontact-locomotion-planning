import numpy as np
import mlp.config as cfg
import multicontact_api
from multicontact_api import WrenchCone, SOC6, ContactPatch, ContactPhaseHumanoid, ContactSequenceHumanoid
import math
from mlp.utils.util import computeEffectorTranslationBetweenStates, computeEffectorRotationBetweenStates
VERBOSE = False


def computePhaseDuration(cs, pid):
    duration = 0
    phase = cs.contact_phases[pid]
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
    if pid == (cs.size() - 1):
        duration = cfg.DURATION_FINAL
    # Adjust duration if needed to respect bound on effector velocity
    duration_feet = 0.
    duration_feet_trans = 0.
    duration_feet_rot = 0.
    if pid < cs.size() - 1:
        dist_feet = computeEffectorTranslationBetweenStates(phase, cs.contact_phases[pid + 1])
        if dist_feet > 0.:
            duration_feet_trans = (2. * cfg.EFF_T_DELAY + 2. * cfg.EFF_T_PREDEF) + dist_feet / cfg.FEET_MAX_VEL
        rot_feet = computeEffectorRotationBetweenStates(phase, cs.contact_phases[pid + 1])
        if rot_feet > 0.:
            duration_feet_rot = (2. * cfg.EFF_T_DELAY + 2. * cfg.EFF_T_PREDEF) + rot_feet / cfg.FEET_MAX_ANG_VEL
        duration_feet = max(duration_feet_trans, duration_feet_rot)
        # Make it a multiple of solver_dt :
        if duration_feet > 0.:
            duration_feet = math.ceil(duration_feet / cfg.SOLVER_DT) * cfg.SOLVER_DT
        if VERBOSE:
            print("for phase : ", pid)
            print("dist_feet            : ", dist_feet)
            print("duration translation : ", duration_feet_trans)
            print("rot_feet             : ", rot_feet)
            print("duration rotation    : ", duration_feet_rot)
            print("duration complete    : ", duration_feet)
    return max(duration, duration_feet)


## straight line from the center of the support polygon of the current phase to the next one
def generateCentroidalTrajectory(cs, cs_initGuess=None, fullBody=None, viewer=None):
    if cs_initGuess:
        print("WARNING : in centroidal.geometric, initial guess is ignored.")
    cs_result = ContactSequenceHumanoid(cs)
    p0 = cs_result.contact_phases[0]
    com_z = p0.init_state[2, 0]
    previous_phase = None
    # first, compute the new init/final position for each state : in the center of the support polygon
    for pid in range(1, cs_result.size() - 1):
        phase = cs_result.contact_phases[pid]
        state = phase.init_state
        com_x = 0.
        com_y = 0.
        if phase.LF_patch.active:
            com_x += phase.LF_patch.placement.translation[0, 0]
            com_y += phase.LF_patch.placement.translation[1, 0]
        if phase.RF_patch.active:
            com_x += phase.RF_patch.placement.translation[0, 0]
            com_y += phase.RF_patch.placement.translation[1, 0]
        if phase.LH_patch.active:
            com_x += phase.LH_patch.placement.translation[0, 0]
            com_y += phase.LH_patch.placement.translation[1, 0]
        if phase.RH_patch.active:
            com_x += phase.RH_patch.placement.translation[0, 0]
            com_y += phase.RH_patch.placement.translation[1, 0]
        com_x /= phase.numActivePatches()
        com_y /= phase.numActivePatches()
        # test : take com height from config found from planning :
        com_z = cs_result.contact_phases[pid].init_state[2, 0]
        state[0] = com_x
        state[1] = com_y
        state[2] = com_z
        phase.init_state = state
        #print "phase : "+str(pid)+" com Init = "+str(com_x)+","+str(com_y)+","+str(com_z)
        if previous_phase != None:
            previous_phase.final_state = phase.init_state.copy()
        previous_phase = cs_result.contact_phases[pid]

    # then, generate a straight line from init_state to final_state for each phase :
    t_total = 0.
    for pid in range(cs_result.size()):
        phase = cs_result.contact_phases[pid]
        duration = computePhaseDuration(cs_result, pid)
        com0 = phase.init_state[0:3]
        com1 = phase.final_state[0:3]
        vel = (com1 - com0) / duration
        am = np.matrix(np.zeros(3)).T
        t = 0.
        while t < duration - 0.0001:
            u = t / duration
            state = np.matrix(np.zeros(9)).T
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
