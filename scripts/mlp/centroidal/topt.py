import numpy as np
from numpy.linalg import norm
import os
import timeopt
import multicontact_api
from multicontact_api import WrenchCone, SOC6, ContactPatch, ContactPhase, ContactSequence
multicontact_api.switchToNumpyArray()

import time
import mlp.config as cfg
import mlp.viewer.display_tools as display
from mlp.utils.util import *

CONTACT_ANKLE_LEVEL = False  # probably only required for hrp2, as the center of the feet is not the center of the flexibility ...


def isContactEverActive(cs, eeName):
    for phase in cs.contactPhases:
        if eeName == timeopt.EndeffectorID.RF:
            if phase.RF_patch.active:
                return True
        elif eeName == timeopt.EndeffectorID.LF:
            if phase.LF_patch.active:
                return True
        elif eeName == timeopt.EndeffectorID.RH:
            if phase.RH_patch.active:
                return True
        elif eeName == timeopt.EndeffectorID.LH:
            if phase.LH_patch.active:
                return True
        else:
            raise Exception("Unknown effector name")
    return False


## check if two given timeOpt adjacent indices belong to the same phase or not
# by checking the contact forces of each effector
def isNewPhaseFromContact(tp, k0, k1):
    isNew = False
    for i in range(4):
        if norm(tp.getContactForce(i, k0)) > 2. and norm(tp.getContactForce(i, k1)) <= 2.:
            isNew = True
        if norm(tp.getContactForce(i, k1)) > 2. and norm(tp.getContactForce(i, k0)) <= 2.:
            isNew = True
    return isNew


# check if k is the first id of a new phase, by comparing to the lengths in the initial guess
def isNewPhaseFromCS(cs_com, cs_initGuess, p_id):
    """
    id = 0
    for phase in cs.contactPhases :
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
    if len(cs_com.contactPhases[p_id].time_trajectory) == len(cs_initGuess.contactPhases[p_id].time_trajectory):
        return True
    else:
        return False


def isNewPhase(tp, k0, k1, cs_com, cs_initGuess, p_id):
    if cs_initGuess:
        return isNewPhaseFromCS(cs_com, cs_initGuess, p_id)
    else:
        return isNewPhaseFromContact(tp, k0, k1)


def fillCSFromTimeopt(cs, cs_initGuess, tp):
    cs_com = ContactSequence(cs)

    # extract infos from tp to fill cs.contactPhases struct
    u = [0] * 6
    x = [0] * 9
    MASS = tp.getMass()
    p_id = 0  # phase id in cs
    k_id = 1  # id in the current phase
    # tp.getTime(0) == dt !! not 0
    p0 = cs_com.contactPhases[0]
    init_state = p0.init_state
    init_state[0:3] = tp.getInitialCOM()
    p0.init_state = init_state
    state = p0.init_state
    appendOrReplace(p0.time_trajectory, 0, 0.)
    appendOrReplace(p0.state_trajectory, 0, state)
    appendOrReplace(p0.control_trajectory, 0, np.zeros(6))
    for k in range(tp.getTrajectorySize()):
        appendOrReplace(cs_com.contactPhases[p_id].time_trajectory, k_id, tp.getTime(k))
        #extract x and u from tp :
        if k == 0:
            u[0:3] = ((tp.getLMOM(k) / MASS) / tp.getTime(k)).tolist()  # acceleration
            u[3:6] = ((tp.getAMOM(k)) / (tp.getTime(k))).tolist()  # angular momentum variation
        else:
            u[0:3] = (((tp.getLMOM(k) / MASS) - (tp.getLMOM(k - 1) / MASS)) /
                      (tp.getTime(k) - tp.getTime(k - 1))).tolist()  #acceleration
            u[3:6] = ((tp.getAMOM(k) - tp.getAMOM(k - 1)) /
                      (tp.getTime(k) - tp.getTime(k - 1))).tolist()  # angular momentum variation
        #print "control = ",np.matrix(u)
        x[0:3] = tp.getCOM(k).tolist()  # position
        x[3:6] = (tp.getLMOM(k) / MASS).tolist()  # velocity
        x[6:9] = tp.getAMOM(k).tolist()  # angular momentum

        appendOrReplace(cs_com.contactPhases[p_id].control_trajectory, k_id, np.array(u))
        appendOrReplace(cs_com.contactPhases[p_id].state_trajectory, k_id, np.array(x))

        if k > 0 and isNewPhase(tp, k - 1, k, cs_com, cs_initGuess, p_id) and p_id < cs_com.size() - 1:
            #last k of current phase, first k of next one (same state_traj and time)
            # set final state of current phase :
            cs_com.contactPhases[p_id].final_state = np.array(x)
            # first k of the current phase
            p_id += 1
            k_id = 0
            cs_com.contactPhases[p_id].init_state = np.array(x)
            appendOrReplace(cs_com.contactPhases[p_id].time_trajectory, k_id, tp.getTime(k))
            appendOrReplace(cs_com.contactPhases[p_id].control_trajectory, k_id, np.array(u))
            appendOrReplace(cs_com.contactPhases[p_id].state_trajectory, k_id, np.array(x))
        k_id += 1
    if cfg.DURATION_CONNECT_GOAL > 0:
        # timeopt solution is not guarantee to end at the desired final state.
        # so we add a final phase here, with a smooth motion from the final state atteined by timeopt to the desired one
        connectPhaseTrajToFinalState(cs_com.contactPhases[-1], cfg.DURATION_CONNECT_GOAL)
    return cs_com


# helper method to make the link between timeopt.EndEffector and cs.Patch
def getPhasePatchforEE(phase, ee):
    if ee == timeopt.EndeffectorID.RF:
        if CONTACT_ANKLE_LEVEL:
            return JointPatchForEffector(phase, cfg.Robot.rfoot)
        else:
            return phase.RF_patch
    if ee == timeopt.EndeffectorID.LF:
        if CONTACT_ANKLE_LEVEL:
            return JointPatchForEffector(phase, cfg.Robot.lfoot)
        else:
            return phase.LF_patch
    if ee == timeopt.EndeffectorID.RH:
        if CONTACT_ANKLE_LEVEL:
            return JointPatchForEffector(phase, cfg.Robot.rhand)
        else:
            return phase.RH_patch
    if ee == timeopt.EndeffectorID.LH:
        if CONTACT_ANKLE_LEVEL:
            return JointPatchForEffector(phase, cfg.Robot.lhand)
        else:
            return phase.LH_patch


# extract a list of effector phase (t start, t end, placement) (as defined by timeopt) from cs struct
def extractEffectorPhasesFromCS(cs, ee):
    ee_phases = []
    pid = 0
    # find a first active patch or given effector :
    while pid < cs.size():
        p = cs.contactPhases[pid]
        patch = getPhasePatchforEE(p, ee)
        if patch.active:
            previous_patch = patch
            t_start = p.time_trajectory[0]
            t_end = p.time_trajectory[-1]
            # find the last phase where the same patch is active
            pid += 1
            while patch.active and patch.placement == previous_patch.placement and pid <= cs.size():
                t_end = p.time_trajectory[-1]
                if pid < cs.size():
                    p = cs.contactPhases[pid]
                    patch = getPhasePatchforEE(p, ee)
                pid += 1
            pid -= 1
            ee_phases += [[t_start, t_end, previous_patch.placement]]
        pid += 1
    return ee_phases


def extractEffectorPhasesFromCSWithoutInitGuess(cs, ee):
    #TODO : same as above but take hardcoded duration (depending on the type of phases instead of the one inside the trajectory)
    raise Exception("Not implemented yet.")


def addCOMviapoints(tp, cs, cs_initGuess, viewer=None):
    phase_previous = None
    for pid in range(1, cs.size() - 1):
        phase = cs.contactPhases[pid]
        phase_previous = cs.contactPhases[pid - 1]
        phase_next = cs.contactPhases[pid + 1]
        if phase_previous and (phase_previous.numActivePatches() < phase.numActivePatches()) and phase_next and (
                phase_next.numActivePatches() < phase.numActivePatches()):
            com = phase.init_state[0:3]
            tp.setViapoint(cs_initGuess.contactPhases[pid].time_trajectory[0], com)
            if viewer and cfg.DISPLAY_WP_COST:
                display.displaySphere(viewer, com.tolist())


def extractAllEffectorsPhasesFromCS(cs, cs_initGuess, ee_ids):
    effectors_phases = {}
    size = 0
    for ee in ee_ids:
        #print "looking for phases for effector : ",ee
        if cs_initGuess != None:
            ee_phases = extractEffectorPhasesFromCS(cs_initGuess, ee)
        else:
            ee_phases = extractEffectorPhasesFromCSWithoutInitGuess(cs, ee)
        #print ee_phases
        #print "num of phases : ",len(ee_phases)
        size += len(ee_phases)
        if len(ee_phases) > 0:
            effectors_phases.update({ee: ee_phases})
    return effectors_phases, size


## fill state trajectory and time trajectory with a linear trajectory connecting init_state to final_state
## the trajectories vectors must be empty when calling this method !
#def generateLinearInterpTraj(phase,duration,t_total):
#com0 = phase.init_state[0:3]
#com1 = phase.final_state[0:3]
#vel = (com1-com0)/duration
#acc = np.matrix(np.zeros(3)).T
#L = np.matrix(np.zeros(3)).T
#dL = np.matrix(np.zeros(3)).T
#state = np.matrix(np.zeros(9)).T
#control = np.matrix(np.zeros(6)).T
#t = 0.
#while t < duration - 0.0001 :
#u = t/duration
#com = com0*(1.-u) + com1*(u)
#state[0:3] = com
#state[3:6] = vel
#state[6:9] = L
#control[0:3] = acc
#control[3:6] = dL
#phase.state_trajectory.append(state)
#phase.control_trajectory.append(control)
#phase.time_trajectory.append(t_total)
#t += cfg.SOLVER_DT
#t_total +=cfg.SOLVER_DT
#state[0:3] = com1
#state[3:6] = vel
#state[6:9] = L
#control[0:3] = acc
#control[3:6] = dL
#phase.state_trajectory.append(state)
#phase.control_trajectory.append(control)
#phase.time_trajectory.append(t_total)
#return phase


# add an initial and final phase that only move the COM along z from the given distance
def addInitShift(cs):
    # shit all times of TIME_SHIFT_COM
    for k in range(cs.size()):
        phase = cs.contactPhases[k]
        # shift times to take in account the new duration of init phase :
        for i in range(len(phase.time_trajectory)):
            phase.time_trajectory[i] += cfg.TIME_SHIFT_COM

    # now add new first phase :
    prev_phase_init = cs.contactPhases[0]
    phase_init = ContactPhase()
    copyPhaseContacts(prev_phase_init, phase_init)
    phase_init.reference_configurations = prev_phase_init.reference_configurations
    # generate trajectory for the 'shift' part :
    s_final = prev_phase_init.init_state.copy()
    s_init = s_final.copy()
    s_init[2] -= cfg.COM_SHIFT_Z
    phase_init.init_state = s_init
    phase_init.final_state = s_final
    genSplinesForPhase(phase_init, 0., cfg.TIME_SHIFT_COM)
    # fill the rest of the phase with the trajectory in previous_phase
    for i in range(1, len(prev_phase_init.time_trajectory)):
        # values for id = 0 are aready the last point in the trajectory computed by genSplinesForPhase
        phase_init.time_trajectory.append(prev_phase_init.time_trajectory[i])
        phase_init.state_trajectory.append(prev_phase_init.state_trajectory[i])
        phase_init.control_trajectory.append(prev_phase_init.control_trajectory[i])
    cs.contactPhases[0] = phase_init
    return cs


def generateCentroidalTrajectory(cs, cs_initGuess=None, fullBody=None, viewer=None):
    if cs_initGuess:
        print(
            "WARNING : in current implementation of timeopt.generateCentroidalTrajectory the initial guess is ignored. (TODO)"
        )
    q_init = cs.contactPhases[0].q_init.copy()
    num_phases = cs.size()
    ee_ids = [timeopt.EndeffectorID.RF, timeopt.EndeffectorID.LF, timeopt.EndeffectorID.RH, timeopt.EndeffectorID.LH]

    effectors_phases, size = extractAllEffectorsPhasesFromCS(cs, cs_initGuess, ee_ids)
    #print "final dic : ",effectors_phases
    print("final number of phases : ", size)
    # initialize timeopt problem :
    tp = timeopt.problem(size)
    com_init = cs.contactPhases[0].init_state[:3]
    vel_init = cs.contactPhases[0].init_state[3:6]
    com_end = cs.contactPhases[-1].final_state[:3]
    com_init[2] += cfg.COM_SHIFT_Z
    tp.setInitialCOM(com_init)
    tp.setInitialLMOM(vel_init * cfg.MASS)
    tp.setFinalCOM(com_end)
    p0 = cs.contactPhases[0]
    for ee in ee_ids:
        patch = getPhasePatchforEE(p0, ee)
        tp.setInitialPose(isContactEverActive(cs, ee), patch.placement.translation, patch.placement.rotation, ee)
    tp.setMass(cfg.MASS)
    #FIXME
    tp.getMass()

    #add all effector phases to the problem :
    i = 0
    for ee in effectors_phases.keys():
        for phase in effectors_phases[ee]:
            tp.setPhase(i, timeopt.phase(ee, phase[0], phase[1], phase[2].translation, phase[2].rotation))
            i += 1
    if cfg.USE_WP_COST:
        addCOMviapoints(tp, cs, cs_initGuess, viewer)
    cfg_path = cfg.TIME_OPT_CONFIG_PATH + '/' + cfg.TIMEOPT_CONFIG_FILE
    print("set configuration file for time-optimization : ", cfg_path)
    tp.setConfigurationFile(cfg_path)
    tp.setTimeoptSolver(cfg_path)
    tStart = time.time()
    tp.solve()
    tTimeOpt = time.time() - tStart
    print("timeopt problem solved in : " + str(tTimeOpt) + " s")
    print("write results in cs")

    cs_result = fillCSFromTimeopt(cs, cs_initGuess, tp)
    if cfg.TIME_SHIFT_COM > 0:
        cs_result = addInitShift(cs_result)

    return cs_result
