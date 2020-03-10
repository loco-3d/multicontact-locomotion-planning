import numpy as np
import mlp.config as cfg
import multicontact_api
from multicontact_api import ContactPhase, ContactSequence
from curves import bezier, piecewise
from mlp.utils.util import createFullbodyStatesFromCS
from mlp.utils.cs_tools import resetCOMtrajectories
from mlp.utils.requirements import Requirements
multicontact_api.switchToNumpyArray()

class Inputs(Requirements):
    timings = True
    consistentContacts = True
    configurationValues = True

class Outputs(Inputs):
    COMtrajectories = True
    COMvalues = True


def setCOMfromCurve(phase, curve_normalized):
    """
    Initialise the phase c_t dc_t and ddc_t from the given curve,
    Also set the final values for the CoM from the final points of the curve
    Also set or increase the final time from the duration of the curve
    :param phase:
    :param curve_normalized: the curve is defined in [0,max], we need to shift it
    :return:
    """
    if phase.c_t is None:
        phase.c_t = piecewise()
        phase.dc_t = piecewise()
        phase.ddc_t = piecewise()
        t_min = phase.timeInitial
        # set initial values
        phase.c_init = curve_normalized(0.)
        phase.dc_init = curve_normalized.derivate(0.,1)
        phase.ddc_init = curve_normalized.derivate(0.,2)
    else:
        t_min = phase.c_t.max()
    t_max = t_min + curve_normalized.max()
    # shift the curve to the correct initial time :
    curve = bezier(curve_normalized.waypoints(), t_min, t_max)
    # set the trajectory in the phase :
    phase.c_t.append(curve)
    phase.dc_t.append(curve.compute_derivate(1))
    phase.ddc_t.append(curve.compute_derivate(2))
    # set the new final values :
    phase.timeFinal = t_max
    phase.c_final = phase.c_t(t_max)
    phase.dc_final = phase.dc_t(t_max)
    phase.ddc_final = phase.ddc_t(t_max)



# Not generic .... assume that there is always 2 contact phases for each state in fullBody
def generateCentroidalTrajectory(cs, cs_initGuess=None, fb=None, viewer=None, first_iter = True):
    if cs_initGuess:
        print("WARNING : in centroidal.croc, initial guess is ignored.")
    if not fb:
        raise ValueError("CROC called without fullBody object.")
    if not first_iter:
        print("WARNING: in centroidal.croc, it is useless to iterate several times.")
    beginId, endId = createFullbodyStatesFromCS(cs, fb)
    print("beginid = ", beginId)
    print("endId   = ", endId)
    cs_result = ContactSequence(cs)
    resetCOMtrajectories(cs_result) # erase all pre existing CoM trajectories

    if (2 * (endId - beginId) + 1) != cs.size():
        raise NotImplemented("Current implementation of CROC require to have 2 contact phases per States in fullBody")
    # for each phase in the cs, create a corresponding FullBody State and call CROC,
    # then fill the cs struct
    id_phase = 0
    current_t = cs.contactPhases[0].timeInitial
    for id_state in range(beginId, endId):
        print("id_state = ", str(id_state))
        print("id_phase = ", str(id_phase))
        # First, compute the bezier curves between the states :
        pid = fb.isDynamicallyReachableFromState(id_state, id_state + 1, True, numPointsPerPhases=0)
        if len(pid) != 4:
            print("# ERROR : Cannot compute CROC between states " + str(id_state) + " , " + str(id_state + 1))
            return cs
        c1 = fb.getPathAsBezier(int(pid[1])) # curve before contact break
        c2 = fb.getPathAsBezier(int(pid[2])) # curve for swing phase
        c3 = fb.getPathAsBezier(int(pid[3])) # curve after contact creation
        # NOTE : this bezier are defined in [0,tmax] ! We need to switch the time intervals

        # fill the phases with the curves from CROC :
        # c1 :
        phase = cs_result.contactPhases[id_phase]
        setCOMfromCurve(phase,c1)
        current_t = phase.timeFinal
        # c2
        id_phase += 1
        phase = cs_result.contactPhases[id_phase]
        phase.timeInitial = current_t
        setCOMfromCurve(phase,c2)
        current_t = phase.timeFinal
        # c3
        id_phase += 1
        phase = cs_result.contactPhases[id_phase]
        phase.timeInitial = current_t
        setCOMfromCurve(phase,c3)
        # pid should not increase here, as the next c1 trajectory will be added to the same phase as the current c3 one.


    return cs_result
