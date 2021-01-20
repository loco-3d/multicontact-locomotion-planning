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
from mlp.utils.requirements import Requirements
import logging
logging.basicConfig(format='[%(name)-12s] %(levelname)-8s: %(message)s')
logger = logging.getLogger("momentumopt")
logger.setLevel(logging.ERROR) #DEBUG, INFO or WARNING

multicontact_api.switchToNumpyArray()


class CentroidalInputsMomentumopt(Requirements):
    timings = True
    consistentContacts = True
    COMvalues = True

class CentroidalOutputsMomentumopt(CentroidalInputsMomentumopt):
    centroidalTrajectories = True



def setDuration(planner_setting, cs, dt):
    """
    Update the planner_setting with the time_horizon, dt and number of steps from the CS and the config file
    :param planner_setting: a planner_setting instance from momentumopt that will be modified
    :param cs: the ContactSequence used to find the duration of the complete motion
    :param dt: discretization step used
    """
    duration = cs.contactPhases[-1].timeFinal - cs.contactPhases[0].timeInitial
    n_time_steps = int(floor(duration/dt))
    planner_setting.set(mopt.PlannerDoubleParam_TimeHorizon, duration)
    planner_setting.set(mopt.PlannerIntParam_NumTimesteps, n_time_steps)
    planner_setting.set(mopt.PlannerDoubleParam_TimeStep, dt)


def extractEffectorPhasesFromCS(cs, eeName, dt):
    """
    extract a list of effector phase (t start, t end, placement) (as defined by momentumopt) from a ContactSequence
    :param cs: the ContactSequence containing all the contact phases
    :param eeName: the effector name (multicontact_api notation)
    :param dt: discretization step used
    :return: The effector phases in momentumopt format
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
                t_end += dt
            ee_phases += [[t_start, t_end, previous_placement]]
        pid += 1

    return ee_phases

def contactPlanFromCS(planner_setting, cs, dict_ee_to_timeopt, dt):
    """
    Create a pymomentum.ContactPlan from the multicontact_api.ContactSequence
    :param planner_setting:
    :param cs: a multicontact_api ContactSequence
    :param dict_ee_to_timeopt: a dictionnary with key = effector names in ContactSequence,
     value = effector ID in momentumopt
    :param dt: discretization step used
    :return: the ContactPlan (momemtumopt format)
    """
    logger.info("Generate contact plan from CS ...")
    contact_plan = ContactPlanFromFile()
    contact_plan.initialize(planner_setting)
    mopt_cs = contact_plan.contactSequence()
    t_init = cs.contactPhases[0].timeInitial
    eeNames = cs.getAllEffectorsInContact()
    for eeName in eeNames:
        phases = extractEffectorPhasesFromCS(cs, eeName, dt)
        mopt_cs_ee = mopt_cs.contact_states(dict_ee_to_timeopt[eeName])
        for phase in phases:
            cp = ContactState()
            cp.start_time = phase[0] - t_init
            cp.end_time = phase[1] - t_init
            cp.contactType = ContactType.FlatContact # TODO: store/retrieve it from mcapi
            cp.active = True
            cp.placement = phase[2].homogeneous
            mopt_cs_ee.append(cp)
    logger.info("Generate contact plan from CS Done.")
    return contact_plan

def setFinalCOM(planner_setting, cs):
    """
    set the CoM_motion for momentumopt from the contactSequence
    :param planner_setting: the planner_setting from momemtumopt that will be modified
    :param cs: the ContactSequence containing the CoM motion
    """
    com_motion = cs.contactPhases[-1].c_final - cs.contactPhases[0].c_init
    logger.debug("Desired com motion : %s", com_motion)
    planner_setting.set(mopt.PlannerVectorParam_CenterOfMassMotion,com_motion)


def addCOMviapoints(planner_setting, cs, viewer=None, display_wp = False):
    """
    Add CoM viapoint to the momentumopt problem. Computed for each double support phases from the planned configuration
    :param planner_setting: the planner_setting from momemtumopt that will be modified
    :param cs: the ContactSequence containing the CoM position at the beginning of each new double support phases
    :param viewer: an instance of the gepetto-gui
    :param display_wp: if True and a viewer is provided, display black sphere at the position of the waypoints used
    """
    logger.info("Add waypoints ...")
    com_viapoints = []
    for pid in range(1, cs.size() - 1):
        phase = cs.contactPhases[pid]
        phase_previous = cs.contactPhases[pid - 1]
        phase_next = cs.contactPhases[pid + 1]
        if (phase_previous.numContacts() < phase.numContacts())  and (phase_next.numContacts() < phase.numContacts()):
            com = phase.c_init
            com_viapoints += [np.array([phase.timeInitial, com[0], com[1], com[2]])]
            if viewer and display_wp:
                display.displaySphere(viewer, com.tolist())
    planner_setting.set(mopt.PlannerCVectorParam_Viapoints, com_viapoints)
    planner_setting.set(mopt.PlannerIntParam_NumViapoints, len(com_viapoints))
    logger.info("Add waypoints done.")

def initStateFromPhase(phase, use_shift_com, com_shift_z, dict_ee_to_timeopt, mass):
    """
    Create a momentumopt initial state from a multicontact_api contact phase
    :param phase: a multicontact_api ContactPhase
    :param use_shift_com: True if the offset to the CoM must be applied
    :param com_shift_z: the distance along the z axis from which the CoM is moved before the beginning of the motion
    :param dict_ee_to_timeopt: a dictionnary with key = effector names in ContactSequence,
     value = effector ID in momentumopt
    :return: a pymomentum DynamicsState
    """
    ini_state = DynamicsState()
    com = phase.c_init
    if use_shift_com:
        com[2] += com_shift_z
    ini_state.com = com
    if not use_shift_com:
        ini_state.lmom = phase.dc_init * mass
        ini_state.lmomd = phase.ddc_init * mass
        ini_state.amom = phase.L_init
        ini_state.amomd = phase.dL_init
        #otherwise, set them to 0 (default)
    logger.debug("Initial CoM : %s", ini_state.com)
    logger.debug("Initial lmom : %s", ini_state.lmom)
    logger.debug("Initial lmomd : %s", ini_state.lmomd)
    logger.debug("Initial amom : %s", ini_state.amom)
    logger.debug("Initial amomd : %s", ini_state.amomd)

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
    :param planner_setting: the planner_setting from momemtumopt
    :param cs: the contactSequence object containing the centroidal trajectory (c, dc, and L)
    :param t_init: time at which the centroidal trajectory start in the CS
    :return: a pymomentum.KinematicsSequence
    """
    logger.info("Build kinematic sequence from CS ...")
    assert cs.haveAMtrajectories(), \
        "In momentumopt, the given contact sequence do not have Angular momentum trajectories set. " \
        "Check that  IK_store_centroidal = True"
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
        if id == len(states)-1:
            t = L_t.max()
        #state.com = c_t(t)
        #state.lmom = dc_t(t) * MASS
        state.amom = L_t(t)
    logger.info("Build kinematic sequence from CS done.")
    return kin_sequence

def isNewPhase(ds1, ds2):
    """
    Check if two dynamicsState have the same contacts
    :param ds1: the first phase
    :param ds2: the second phase
    :return: True if they have the same contacts, False otherwise
    """
    assert ds1.effNum() == ds2.effNum(), "The two dynamic states do not comes from the same model."

    for i in range(ds1.effNum()):
        if ds1.effActivation(i) != ds2.effActivation(i):
            return True
    return False


def CSfromMomentumopt(planner_setting, cs, init_state, dyn_states, t_init = 0., connect_goal = True):
    """
    Create a ContactSequence and fill it with the results from momentumopt
    :param planner_setting: the planner_setting object from momentumopt
    :param cs: a multicontact_api ContactSequence from which the contacts are copied
    :param init_state: initial state used by momentumopt
    :param dyn_states: the results of momentumopt
    :return: a multicontact_api ContactSequence with centroidal trajectories
    """
    logger.info("Start to convert result to mc-api ...")
    cs_com = ContactSequence(cs)
    MASS = planner_setting.get(mopt.PlannerDoubleParam_RobotMass)
    p_id = 0  # phase id in cs
    # dyn_states[0] is at t == dt , not t == 0 ! use init_state for t == 0
    p0 = cs_com.contactPhases[0]
    c_init = init_state.com
    dc_init = init_state.lmom / MASS
    ddc_init = init_state.lmomd / MASS
    L_init = init_state.amom
    dL_init = init_state.amomd
    p0.timeInitial = t_init

    # init arrays to store the discrete points. one value per columns
    c_t = c_init.reshape(3, 1)
    dc_t = dc_init.reshape(3, 1)
    ddc_t = ddc_init.reshape(3, 1)
    L_t = L_init.reshape(3, 1)
    dL_t = dL_init.reshape(3, 1)
    times = array(t_init)
    current_t = t_init
    # loop for all dynamicsStates
    for k, ds in enumerate(dyn_states):
        #extract states values from ds :
        c = ds.com # position
        dc = ds.lmom / MASS # velocity
        ddc = ds.lmomd / MASS  # acceleration
        L = ds.amom # angular momentum
        dL = ds.amomd  # angular momentum variation
        # stack the values in the arrays:
        c_t = append(c_t, c.reshape(3,1), axis = 1)
        dc_t = append(dc_t, dc.reshape(3,1), axis = 1)
        ddc_t = append(ddc_t, ddc.reshape(3,1), axis = 1)
        L_t = append(L_t, L.reshape(3,1), axis = 1)
        dL_t = append(dL_t, dL.reshape(3,1), axis = 1)
        current_t += ds.dt
        times = append(times, round(current_t, 6))

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
    setCOMtrajectoryFromPoints(phase, c_t, dc_t, ddc_t, times, overwriteFinal = not connect_goal)
    setAMtrajectoryFromPoints(phase, L_t, dL_t, times, overwriteFinal = not connect_goal)
    # set final time :
    phase.timeFinal = times[-1]
    logger.info("Converting results to mc-api done.")
    return cs_com


def generate_centroidal_momentumopt(cfg, cs, cs_initGuess=None, fullBody=None, viewer=None, first_iter = True):
    if cs_initGuess and first_iter:
        logger.warning("The initial guess is ignored. (TODO)")
    if not first_iter:
        if cs_initGuess is None or not cs_initGuess.haveCentroidalValues():
            raise RuntimeError("Centroidal.momentumopt called after a first iteration without a valid reference ContactSequence provided.")

    dict_ee_to_timeopt = {cfg.Robot.rfoot: EffId.right_foot.value(),
                          cfg.Robot.lfoot: EffId.left_foot.value(),
                          cfg.Robot.rhand: EffId.right_hand.value(),
                          cfg.Robot.lhand: EffId.left_hand.value()}

    tStart = time.time()
    # load planner settings from yaml:
    planner_setting = PlannerSetting()
    cfg_path = cfg.TIME_OPT_CONFIG_PATH + '/' + cfg.TIMEOPT_CONFIG_FILE
    logger.warning("Use configuration file for momentumopt : %s", cfg_path)
    planner_setting.initialize(cfg_path)
    planner_setting.set(mopt.PlannerIntParam_NumActiveEndeffectors, len(cs.getAllEffectorsInContact()))
    setDuration(planner_setting, cs, cfg.SOLVER_DT)
    contact_plan = contactPlanFromCS(planner_setting, cs, dict_ee_to_timeopt, cfg.SOLVER_DT)
    setFinalCOM(planner_setting, cs)

    if cfg.USE_WP_COST:
        addCOMviapoints(planner_setting, cs, viewer, cfg.DISPLAY_WP_COST)

    ini_state = initStateFromPhase(cs.contactPhases[0], cfg.TIME_SHIFT_COM > 0, cfg.COM_SHIFT_Z, dict_ee_to_timeopt,
                                   planner_setting.get(mopt.PlannerDoubleParam_RobotMass))
    if first_iter:
        kin_sequence = buildEmptyKinSequence(planner_setting)
    else:
        kin_sequence = buildKinSequenceFromCS(planner_setting, cs_initGuess, cfg.TIME_SHIFT_COM)

    # build the dynamic optimizer:
    dyn_opt = DynamicsOptimizer()
    dyn_opt.initialize(planner_setting)
    # optimize the motion
    logger.info("Start optimization ...")
    code = dyn_opt.optimize(ini_state, contact_plan, kin_sequence, not first_iter)
    logger.warning("Momentumopt internal solving time: " + str(dyn_opt.solveTime() / 1000.) + " s")
    if code != ExitCode.Optimal:
        logger.error("!! momentumopt exit with a non Optimal status: %s", code)

    # now build a new multicontact_api contactSequence from the results of momentumopt:
    cs_result = CSfromMomentumopt(planner_setting, cs, ini_state, dyn_opt.dynamicsSequence().dynamics_states,
                                  cs.contactPhases[0].timeInitial + cfg.TIME_SHIFT_COM,
                                  connect_goal = (cfg.DURATION_CONNECT_GOAL > 0.))

    if cfg.TIME_SHIFT_COM > 0:
        connectPhaseTrajToInitialState(cs_result.contactPhases[0], cfg.TIME_SHIFT_COM)
    if cfg.DURATION_CONNECT_GOAL > 0:
        # momentumopt solution is not guarantee to end at the desired final state.
        # so we add a final phase here, with a smooth motion from the final state atteined by timeopt to the desired one
        connectPhaseTrajToFinalState(cs_result.contactPhases[-1], cfg.DURATION_CONNECT_GOAL)

    tCentroidal = time.time() - tStart
    logger.warning("Centroidal total time : %f s.", tCentroidal)
    return cs_result



