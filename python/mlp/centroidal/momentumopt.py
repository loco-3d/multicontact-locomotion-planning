try:
    import pymomentum as mopt
    from pymomentum import PlannerSetting, ContactPlanFromFile,ContactState, DynamicsOptimizer, DynamicsState, ContactType, EffId, KinematicsSequence
    from pysolver import ExitCode
except ImportError:
    message = "ERROR: Cannot import momentumopt python library.\n"
    message += "Did you correctly installed it?\n"
    message += "See https://github.com/machines-in-motion/kino_dynamic_opt"
    raise ImportError(message)
from math import floor
import multicontact_api
from multicontact_api import ContactSequence
import time
import mlp.viewer.display_tools as display
from mlp.utils.cs_tools import setCOMtrajectoryFromPoints, setAMtrajectoryFromPoints, connectPhaseTrajToFinalState, connectPhaseTrajToInitialState
import numpy as np
from numpy import array, append
from numpy.linalg import norm
from pinocchio import SE3
import mlp.config as cfg
from mlp.utils.requirements import Requirements

multicontact_api.switchToNumpyArray()


class Inputs(Requirements):
    timings = True
    consistentContacts = True
    COMvalues = True

class Outputs(Inputs):
    centroidalTrajectories = True


dict_ee_to_timeopt = {cfg.Robot.rfoot : EffId.right_foot.value(),
                    cfg.Robot.lfoot : EffId.left_foot.value() ,
                    cfg.Robot.rhand : EffId.right_hand.value(),
                    cfg.Robot.lhand : EffId.left_hand.value()}


def setDuration(planner_setting, cs):
    """
    Update the planner_setting with the time_horizon, dt and number of steps from the CS and the config file
    :param planner_setting:
    :param cs:
    :return:
    """
    duration = cs.contactPhases[-1].timeFinal
    dt = cfg.SOLVER_DT
    n_time_steps = int(floor(duration/dt))
    planner_setting.set(mopt.PlannerDoubleParam_TimeHorizon, duration)
    planner_setting.set(mopt.PlannerIntParam_NumTimesteps, n_time_steps)
    planner_setting.set(mopt.PlannerDoubleParam_TimeStep, dt)


def extractEffectorPhasesFromCS(cs, eeName):
    """
    extract a list of effector phase (t start, t end, placement) (as defined by momentumopt) from a ContactSequence
    :param cs:
    :param eeName: the effector name (CS notation)
    :return:
    """
    t_last = cs.contactPhases[-1].timeFinal
    ee_phases = []
    pid = 0
    # find a first active patch or given effector :
    while pid < cs.size():
        p = cs.contactPhases[pid]
        if p.isEffectorInContact(eeName):
            previous_placement = p.contactPatch(eeName).placement # first patch where this contact begin
            t_start = p.timeInitial
            t_end = p.timeFinal
            # find the last phase where the same patch is active
            pid += 1
            while p.isEffectorInContact(eeName) and p.contactPatch(eeName).placement == previous_placement \
                    and pid <= cs.size():
                t_end = p.timeFinal
                if pid < cs.size():
                    p = cs.contactPhases[pid]
                pid += 1
            pid -= 1
            if t_end == t_last:
                # increase the duration of the last contact by one dt,
                # otherwise momentumopt break the contact at the last iteration
                t_end += cfg.SOLVER_DT
            ee_phases += [[t_start, t_end, previous_placement]]
        pid += 1

    return ee_phases

def contactPlanFromCS(planner_setting, cs):
    """
    Create a pymomentum.ContactPlan from the multicontact_api.ContactSequence
    :param planner_setting:
    :param cs: a multicontact_api ContactSequence
    :return: the ContactPlan
    """
    contact_plan = ContactPlanFromFile()
    contact_plan.initialize(planner_setting)
    mopt_cs = contact_plan.contactSequence()

    eeNames = cs.getAllEffectorsInContact()
    for eeName in eeNames:
        phases = extractEffectorPhasesFromCS(cs, eeName)
        mopt_cs_ee = mopt_cs.contact_states(dict_ee_to_timeopt[eeName])
        for phase in phases:
            cp = ContactState()
            cp.start_time = phase[0]
            cp.end_time = phase[1]
            cp.contactType = ContactType.FlatContact # TODO: store/retrieve it from mcapi
            cp.active = True
            cp.placement = phase[2].homogeneous
            mopt_cs_ee.append(cp)

    return contact_plan

def setFinalCOM(planner_setting, cs):
    """
    set the CoM_motion for momentumopt from the contactSequence
    :param planner_setting:
    :param cs:
    :return:
    """
    com_motion = cs.contactPhases[-1].c_final - cs.contactPhases[0].c_init
    planner_setting.set(mopt.PlannerVectorParam_CenterOfMassMotion,com_motion)


def addCOMviapoints(planner_setting, cs, viewer=None):
    """
    Add CoM viapoint to the momentumopt problem. Computed for each double support phases from the planned configuration
    :param planner_setting:
    :param cs:
    :param viewer:
    :return:
    """
    com_viapoints = []
    for pid in range(1, cs.size() - 1):
        phase = cs.contactPhases[pid]
        phase_previous = cs.contactPhases[pid - 1]
        phase_next = cs.contactPhases[pid + 1]
        if (phase_previous.numContacts() < phase.numContacts())  and (phase_next.numContacts() < phase.numContacts()):
            com = phase.c_init
            com_viapoints += [np.array([phase.timeInitial, com[0], com[1], com[2]])]
            if viewer and cfg.DISPLAY_WP_COST:
                display.displaySphere(viewer, com.tolist())
    planner_setting.set(mopt.PlannerCVectorParam_Viapoints, com_viapoints)
    planner_setting.set(mopt.PlannerIntParam_NumViapoints, len(com_viapoints))

def initStateFromPhase(phase):
    """
    Create a momentumopt initial state from a multicontact_api contact phase
    :param phase: a multicontact_api ContactPhase
    :return: a pymomentum DynamicsState
    """
    ini_state = DynamicsState()
    com = phase.c_init
    if cfg.TIME_SHIFT_COM > 0.:
        com[2] += cfg.COM_SHIFT_Z
    ini_state.com = com
    force_distribution = 1./phase.numContacts() # assume contact forces are distributed between all contacts
    for eeName, patch in phase.contactPatches().items():
        ini_state.setEffPosition(dict_ee_to_timeopt[eeName], patch.placement.translation)
        ini_state.setEffActivation(dict_ee_to_timeopt[eeName], True)
        ini_state.setEffForce(EffId.right_foot, np.array([0, 0, force_distribution])) # FIXME : depend of the contact normal
    return ini_state

def buildEmptyKinSequence(planner_setting):
    """
    Create an empty kinematic sequence of the correct dimension
    :param planner_setting:
    :return: a pymomentum.KinematicSequence
    """
    kin_sequence = KinematicsSequence()
    kin_sequence.resize(planner_setting.get(mopt.PlannerIntParam_NumTimesteps), 1)
    return kin_sequence

def buildKinSequenceFromCS(planner_setting, cs, t_init):
    """
    Build a KinematicSequence and fill it with values from the centroidal trajectory stored in CS.
    :param planner_setting:
    :param cs:
    :param t_init: time at which the centroidal trajectory start in the CS
    :return: a pymomentum.KinematicsSequence
    """
    kin_sequence = KinematicsSequence()
    kin_sequence.resize(planner_setting.get(mopt.PlannerIntParam_NumTimesteps), 1)
    dt = planner_setting.get(mopt.PlannerDoubleParam_TimeStep)
    MASS = planner_setting.get(mopt.PlannerDoubleParam_RobotMass)
    states = kin_sequence.kinematics_states
    # fill centroidal trajectories values :
    #c_t = cs.concatenateCtrajectories()
    #dc_t = cs.concatenateDCtrajectories()
    L_t = cs.concatenateLtrajectories()
    for id, state in enumerate(states):
        t = t_init + (id+1) * dt
        #state.com = c_t(t)
        #state.lmom = dc_t(t) * MASS
        state.amom = L_t(t)
    return kin_sequence

def isNewPhase(ds1, ds2):
    """
    Check if two dynamicsState have the same contacts
    :param ds1:
    :param ds2:
    :return: True if they have the same contacts, False otherwise
    """
    assert ds1.effNum() == ds2.effNum(), "The two dynamic states do not comes from the same model."

    for i in range(ds1.effNum()):
        if ds1.effActivation(i) != ds2.effActivation(i):
            return True
    return False


def CSfromMomentumopt(planner_setting, cs, init_state, dyn_states, t_init = 0):
    """
    Create a ContactSequence and fill it with the results from momentumopt
    :param planner_setting:
    :param cs: a multicontact_api ContactSequence from which the contacts are copied
    :param init_state: initial state used by momentumopt
    :param dyn_states: the results of momentumopt
    :return: a multicontact_api ContactSequence with centroidal trajectories
    """
    cs_com = ContactSequence(cs)
    MASS = planner_setting.get(mopt.PlannerDoubleParam_RobotMass)
    p_id = 0  # phase id in cs
    # dyn_states[0] is at t == dt , not t == 0 ! use init_state for t == 0
    p0 = cs_com.contactPhases[0]
    c_init = init_state.com
    p0.timeInitial = t_init

    # init arrays to store the discrete points. one value per columns
    c_t = c_init.reshape(3, 1)
    dc_t = p0.dc_init.reshape(3, 1)
    ddc_t = p0.ddc_init.reshape(3, 1)
    L_t = p0.L_init.reshape(3, 1)
    dL_t = p0.dL_init.reshape(3, 1)
    times = array(t_init)
    current_t = t_init
    # loop for all dynamicsStates
    for k, ds in enumerate(dyn_states):
        #extract states values from ds :
        if k == 0:
            ddc = (ds.lmom / MASS) / ds.dt # acceleration
            dL = ds.amom / ds.dt  # angular momentum variation
        else:
            ddc = ((ds.lmom/ MASS) - (dyn_states[k-1].lmom / MASS)) / ds.dt
            dL = (ds.amom- dyn_states[k-1].amom) / ds.dt
        c = ds.com # position
        dc = ds.lmom / MASS # velocity
        L = ds.amom # angular momentum
        # stack the values in the arrays:
        c_t = append(c_t, c.reshape(3,1), axis = 1)
        dc_t = append(dc_t, dc.reshape(3,1), axis = 1)
        ddc_t = append(ddc_t, ddc.reshape(3,1), axis = 1)
        L_t = append(L_t, L.reshape(3,1), axis = 1)
        dL_t = append(dL_t, dL.reshape(3,1), axis = 1)
        current_t += ds.dt
        times = append(times, current_t)

        if k > 0 and isNewPhase(dyn_states[k-1], ds) and p_id < cs_com.size() - 1:
            #last k of current phase, first k of next one (same trajectories and time)
            # set the trajectories for the current phase from the arrays :
            phase = cs_com.contactPhases[p_id]
            setCOMtrajectoryFromPoints(phase, c_t, dc_t, ddc_t, times, overwriteInit= (p_id > 0))
            setAMtrajectoryFromPoints(phase, L_t, dL_t, times, overwriteInit= (p_id > 0))
            # set final time :
            phase.timeFinal = times[-1]
            # Start new phase :
            p_id += 1
            phase = cs_com.contactPhases[p_id]
            # set initial time :
            phase.timeInitial = times[-1]
            # reset arrays of values to only the last point :
            c_t = c_t[:,-1].reshape(3,1)
            dc_t = dc_t[:,-1].reshape(3,1)
            ddc_t = ddc_t[:,-1].reshape(3,1)
            L_t = L_t[:,-1].reshape(3,1)
            dL_t = dL_t[:,-1].reshape(3,1)
            times = times[-1]

    # set final phase :
    phase = cs_com.contactPhases[-1]
    setCOMtrajectoryFromPoints(phase, c_t, dc_t, ddc_t, times, overwriteFinal = (cfg.DURATION_CONNECT_GOAL == 0.))
    setAMtrajectoryFromPoints(phase, L_t, dL_t, times, overwriteFinal = (cfg.DURATION_CONNECT_GOAL == 0.))
    # set final time :
    phase.timeFinal = times[-1]

    return cs_com


def generateCentroidalTrajectory(cs, cs_initGuess=None, fullBody=None, viewer=None, first_iter = True):
    if cs_initGuess is not None and first_iter:
        print("WARNING : in current implementation of timeopt.generateCentroidalTrajectory"
              " the initial guess is ignored. (TODO)")
    if not first_iter:
        if cs_initGuess is None or not cs_initGuess.haveCentroidalValues():
            raise RuntimeError("Centroidal.momentumopt called after a first iteration without a valid reference ContactSequence provided.")

    tStart = time.time()
    # load planner settings from yaml:
    planner_setting = PlannerSetting()
    cfg_path = cfg.TIME_OPT_CONFIG_PATH + '/' + cfg.TIMEOPT_CONFIG_FILE
    print("Use configuration file for momentumopt : ", cfg_path)
    planner_setting.initialize(cfg_path)
    planner_setting.set(mopt.PlannerIntParam_NumActiveEndeffectors, len(cs.getAllEffectorsInContact()))
    setDuration(planner_setting, cs)
    contact_plan = contactPlanFromCS(planner_setting, cs)
    setFinalCOM(planner_setting, cs)

    if cfg.USE_WP_COST:
        addCOMviapoints(planner_setting, cs, viewer)

    ini_state = initStateFromPhase(cs.contactPhases[0])
    if first_iter:
        kin_sequence = buildEmptyKinSequence(planner_setting)
    else:
        kin_sequence = buildKinSequenceFromCS(planner_setting, cs_initGuess, cfg.TIME_SHIFT_COM)

    # build the dynamic optimizer:
    dyn_opt = DynamicsOptimizer()
    dyn_opt.initialize(planner_setting)
    # optimize the motion
    code = dyn_opt.optimize(ini_state, contact_plan, kin_sequence, not first_iter)
    print("Momentumopt internal solving time: " + str(dyn_opt.solveTime() / 1000.) + " s")
    if code != ExitCode.Optimal:
        print("!! WARNING: momentumopt exit with a non Optimal status: ", code)

    # now build a new multicontact_api contactSequence from the results of momentumopt:
    cs_result = CSfromMomentumopt(planner_setting, cs, ini_state, dyn_opt.dynamicsSequence().dynamics_states, cfg.TIME_SHIFT_COM)

    if cfg.TIME_SHIFT_COM > 0:
        connectPhaseTrajToInitialState(cs_result.contactPhases[0], cfg.TIME_SHIFT_COM)
    if cfg.DURATION_CONNECT_GOAL > 0:
        # momentumopt solution is not guarantee to end at the desired final state.
        # so we add a final phase here, with a smooth motion from the final state atteined by timeopt to the desired one
        connectPhaseTrajToFinalState(cs_result.contactPhases[-1], cfg.DURATION_CONNECT_GOAL)

    tCentroidal = time.time() - tStart
    print("Centroidal total time : " + str(tCentroidal) + " s")
    return cs_result



