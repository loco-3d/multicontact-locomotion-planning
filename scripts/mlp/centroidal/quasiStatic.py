import numpy as np
import mlp.config as cfg
from multicontact_api import ContactPhaseHumanoid, ContactSequenceHumanoid
from mlp.utils.util import connectPhaseTrajToFinalState, createFullbodyStatesFromCS, fillPhaseTrajWithZeros, genSplinesForPhase

## Produce a centroidal trajectory where the CoM only move when the contact are fixed.
## It is then fixed during the swing phase where one contact is repositionned.
## The position reached by the CoM are given with 2-PAC
## The trajectory of the CoM is a quintic spline with initial and final velocity/acceleration constrained to 0


def getTargetCOMPosition(fullBody, id_state):
    tab = fullBody.isReachableFromState(id_state, id_state + 1, True, False)
    success = tab[0]
    assert success, "2-pac failed for state id : " + str(id_state)
    if len(tab) == 7:
        cBreak = np.matrix(tab[1:4]).T
        cCreate = np.matrix(tab[4:7]).T
        c = (cBreak + cCreate) / 2.
    else:
        c = np.matrix(tab[1:4]).T
    print("c before shift : ", c)
    c[2] += cfg.COM_SHIFT_Z
    print("c after  shift : ", c)
    return c


def moveToCOMPosition(phase, c, current_t, duration):
    # final com position equal to c
    final_state = np.matrix(np.zeros(9)).T
    final_state[0:3] = c
    phase.final_state = final_state
    genSplinesForPhase(phase, current_t, duration)


def generateCentroidalTrajectory(cs, cs_initGuess=None, fullBody=None, viewer=None):
    if cs_initGuess:
        print("WARNING : in centroidal.quasiStatic, initial guess is ignored.")
    if not fullBody:
        raise ValueError("quasiStatic called without fullBody object.")
    beginId, endId = createFullbodyStatesFromCS(cs, fullBody)
    print("beginid = ", beginId)
    print("endId   = ", endId)
    cs_result = ContactSequenceHumanoid(cs)

    def getPhaseDuration(sid, pid):
        if sid == endId:
            duration = cfg.DURATION_FINAL
        if pid == 0:
            duration = cfg.DURATION_INIT
        elif cs.contact_phases[pid].numActivePatches() == 1:
            duration = cfg.DURATION_SS
        elif cs.contact_phases[pid].numActivePatches() == 2:
            duration = cfg.DURATION_DS
        elif cs.contact_phases[pid].numActivePatches() == 3:
            duration = cfg.DURATION_TS
        elif cs.contact_phases[pid].numActivePatches() == 4:
            duration = cfg.DURATION_QS
        return duration

    # for each phase in the cs, create a corresponding FullBody State and call CROC,
    # then discretize the solution and fill the cs struct
    # Make the assumption that the CS was created with the generateContactSequence method from the same fb object
    id_phase = 0
    for id_state in range(beginId, endId + 1):
        print("id_state = ", str(id_state))
        print("id_phase = ", str(id_phase))
        phase_fixed = cs_result.contact_phases[id_phase]  # phase where the CoM move and the contacts are fixed
        # set initial state to be the final one of the previous phase :
        if id_phase > 1:
            phase_fixed.init_state = phase_swing.final_state.copy()
            current_t = phase_swing.time_trajectory[-1]
        else:
            current_t = 0.
        # compute 'optimal' position of the COM to go before switching phase:
        if id_state == endId:
            c = phase_fixed.final_state[0:3]
        else:
            c = getTargetCOMPosition(fullBody, id_state)
        # fill phase trajectories to connect to 'c'
        moveToCOMPosition(phase_fixed, c, current_t, getPhaseDuration(id_state, id_phase))
        id_phase += 1
        if id_state < endId:
            current_t = phase_fixed.time_trajectory[-1]
            phase_swing = cs_result.contact_phases[id_phase]  # phase where the CoM is fixed and an effector move
            # in swing phase, com do not move :
            phase_swing.init_state = phase_fixed.final_state.copy()
            phase_swing.final_state = phase_fixed.final_state.copy()
            fillPhaseTrajWithZeros(phase_swing, current_t, getPhaseDuration(id_state, id_phase))
            id_phase += 1

    return cs_result
