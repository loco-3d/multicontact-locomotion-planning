import timeopt
import multicontact_api
import mlp.config as cfg
from multicontact_api import ContactSequence
import time
import mlp.viewer.display_tools as display
from mlp.utils.cs_tools import setCOMtrajectoryFromPoints, setAMtrajectoryFromPoints, connectPhaseTrajToFinalState, connectPhaseTrajToInitialState
import numpy as np
from numpy import array, append
from numpy.linalg import norm
from pinocchio import SE3
from mlp.utils.requirements import Requirements

multicontact_api.switchToNumpyArray()
CONTACT_ANKLE_LEVEL = False  # probably only required for hrp2, as the center of the feet is not the center of the flexibility ...

class Inputs(Requirements):
    timings = True
    consistentContacts = True
    COMvalues = True

class Outputs(Inputs):
    centroidalTrajectories = True


dict_ee_to_timeopt = {cfg.Robot.rfoot : timeopt.EndeffectorID.RF,
                    cfg.Robot.lfoot : timeopt.EndeffectorID.LF ,
                    cfg.Robot.rhand : timeopt.EndeffectorID.RH,
                    cfg.Robot.lhand : timeopt.EndeffectorID.LH}


## check if two given timeOpt adjacent indices belong to the same phase or not
# by checking the contact forces of each effector
def isNewPhase(tp, k0, k1):
    isNew = False
    for i in range(4):
        if norm(tp.getContactForce(i, k0)) > 1e-2 and norm(tp.getContactForce(i, k1)) <= 1e-3:
            isNew = True
        if norm(tp.getContactForce(i, k1)) > 1e-2 and norm(tp.getContactForce(i, k0)) <= 1e-3:
            isNew = True
    return isNew



def fillCSFromTimeopt(cs, cs_initGuess, tp, t_init = 0.):
    cs_com = ContactSequence(cs)

    # extract infos from tp to fill cs.contactPhases struct
    MASS = tp.getMass()
    p_id = 0  # phase id in cs
    # tp.getTime(0) == dt !! not 0
    p0 = cs_com.contactPhases[0]
    c_init = tp.getInitialCOM()
    p0.timeInitial = t_init
    # build column matrix of discretized points for the centroidal values of each phases
    c_t = c_init.reshape(3,1)
    dc_t = p0.dc_init.reshape(3,1)
    ddc_t = p0.ddc_init.reshape(3,1)
    L_t = p0.L_init.reshape(3,1)
    dL_t = p0.dL_init.reshape(3,1)
    times = array(t_init)
    for k in range(tp.getTrajectorySize()):
        #extract states values from tp :
        if k == 0:
            ddc = (tp.getLMOM(k) / MASS) / tp.getTime(k) # acceleration
            dL = tp.getAMOM(k) / (tp.getTime(k))  # angular momentum variation
        else:
            ddc = ((tp.getLMOM(k) / MASS) - (tp.getLMOM(k - 1) / MASS)) / (tp.getTime(k) - tp.getTime(k - 1))
            dL = (tp.getAMOM(k) - tp.getAMOM(k - 1)) / (tp.getTime(k) - tp.getTime(k - 1))
        c = tp.getCOM(k) # position
        dc = tp.getLMOM(k) / MASS # velocity
        L = tp.getAMOM(k)  # angular momentum
        # stack the values in the arrays:
        c_t = append(c_t, c.reshape(3,1), axis = 1)
        dc_t = append(dc_t, dc.reshape(3,1), axis = 1)
        ddc_t = append(ddc_t, dc.reshape(3,1), axis = 1)
        L_t = append(L_t, L.reshape(3,1), axis = 1)
        dL_t = append(dL_t, dL.reshape(3,1), axis = 1)
        times = append(times, tp.getTime(k) + t_init)

        if k > 0 and isNewPhase(tp, k - 1, k) and p_id < cs_com.size() - 1:
            #last k of current phase, first k of next one (same state_traj and time)
            # set the trajectories for the current phase from the arrays :
            print(" last point for phase " + str(p_id)+ " at t = "+ str(times[-1]))
            phase = cs_com.contactPhases[p_id]
            setCOMtrajectoryFromPoints(phase, c_t,dc_t,ddc_t,times, overwriteInit= (p_id > 0))
            setAMtrajectoryFromPoints(phase, L_t,dL_t,times)
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
    setCOMtrajectoryFromPoints(phase, c_t, dc_t, ddc_t, times, overwriteFinal = (cfg.DURATION_CONNECT_GOAL == 0))
    setAMtrajectoryFromPoints(phase, L_t, dL_t, times)
    # set final time :
    phase.timeFinal = times[-1]

    if cfg.DURATION_CONNECT_GOAL > 0:
        # timeopt solution is not guarantee to end at the desired final state.
        # so we add a final phase here, with a smooth motion from the final state atteined by timeopt to the desired one
        connectPhaseTrajToFinalState(cs_com.contactPhases[-1], cfg.DURATION_CONNECT_GOAL)
    return cs_com


def extractEffectorPhasesFromCS(cs, eeName):
    """
    extract a list of effector phase (t start, t end, placement) (as defined by timeopt) from a ContactSequence
    :param cs:
    :param eeName: the effector name (CS notation)
    :return:
    """
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
            ee_phases += [[t_start, t_end, previous_placement]]
        pid += 1
    return ee_phases



def addCOMviapoints(tp, cs, viewer=None):
    for pid in range(1, cs.size() - 1):
        phase = cs.contactPhases[pid]
        phase_previous = cs.contactPhases[pid - 1]
        phase_next = cs.contactPhases[pid + 1]
        if (phase_previous.numContacts() < phase.numContacts())  and (phase_next.numContacts() < phase.numContacts()):
            com = phase.c_init
            tp.setViapoint(phase.timeInitial, com)
            if viewer and cfg.DISPLAY_WP_COST:
                display.displaySphere(viewer, com.tolist())


def extractAllEffectorsPhasesFromCS(cs, eeNames):
    """
    Create a dict with the effectors phase (with timeopt format) of each effector
    :param cs: the Cntact sequence
    :param ee_ids: the list of effectors names used (CS notation)
    :return:
    """
    effectors_phases = {}
    size = 0
    for eeName in eeNames:
        #print "looking for phases for effector : ",ee
        ee_phases = extractEffectorPhasesFromCS(cs, eeName)
        #print ee_phases
        #print "num of phases : ",len(ee_phases)
        size += len(ee_phases)
        if len(ee_phases) > 0:
            effectors_phases.update({eeName: ee_phases})
    return effectors_phases, size



def generateCentroidalTrajectory(cs, cs_initGuess=None, fullBody=None, viewer=None):
    if cs_initGuess:
        print("WARNING : in current implementation of timeopt.generateCentroidalTrajectory"
              " the initial guess is ignored. (TODO)")
    eeNames = cs.getAllEffectorsInContact()
    timeopt_ee_names = [timeopt.EndeffectorID.RF, timeopt.EndeffectorID.LF, timeopt.EndeffectorID.RH, timeopt.EndeffectorID.LH]

    effectors_phases, size = extractAllEffectorsPhasesFromCS(cs, eeNames)
    #print "final dic : ",effectors_phases
    print("final number of phases : ", size)
    # initialize timeopt problem :
    tp = timeopt.problem(size)
    com_init = cs.contactPhases[0].c_init
    vel_init = cs.contactPhases[0].dc_init
    com_end = cs.contactPhases[-1].c_final
    com_init[2] += cfg.COM_SHIFT_Z
    tp.setInitialCOM(com_init)
    tp.setInitialLMOM(vel_init * cfg.MASS)
    tp.setFinalCOM(com_end)
    p0 = cs.contactPhases[0]
    for eeName in eeNames:
        timeopt_id = dict_ee_to_timeopt[eeName]
        placement = p0.contactPatch(eeName).placement
        tp.setInitialPose(True, placement.translation, placement.rotation, timeopt_id)
        timeopt_ee_names.remove(timeopt_id)
    placement = SE3.Identity()
    for ee_id in timeopt_ee_names:
        # id never used for contact (they need to be set anyway)
        tp.setInitialPose(False, placement.translation, placement.rotation, ee_id)

    tp.setMass(cfg.MASS)
    #FIXME
    tp.getMass()

    #add all effector phases to the problem :
    i = 0
    for eeName in effectors_phases.keys():
        timeopt_id = dict_ee_to_timeopt[eeName]
        for phase in effectors_phases[eeName]:
            tp.setPhase(i, timeopt.phase(timeopt_id, phase[0], phase[1], phase[2].translation, phase[2].rotation))
            i += 1
    if cfg.USE_WP_COST:
        addCOMviapoints(tp, cs, viewer)
    cfg_path = cfg.TIME_OPT_CONFIG_PATH + '/' + cfg.TIMEOPT_CONFIG_FILE
    print("set configuration file for time-optimization : ", cfg_path)
    tp.setConfigurationFile(cfg_path)
    tp.setTimeoptSolver(cfg_path)
    tStart = time.time()
    tp.solve()
    tTimeOpt = time.time() - tStart
    print("timeopt problem solved in : " + str(tTimeOpt) + " s")
    print("write results in cs")

    cs_result = fillCSFromTimeopt(cs, cs_initGuess, tp,cfg.TIME_SHIFT_COM )
    if cfg.TIME_SHIFT_COM > 0:
        connectPhaseTrajToInitialState(cs_result.contactPhases[0], cfg.TIME_SHIFT_COM)

    return cs_result
