import numpy as np
import mlp.config as cfg
import multicontact_api
from multicontact_api import ContactPhase, ContactSequence
from mlp.utils.requirements import Requirements
from mlp.utils.util import genCOMTrajFromPhaseStates, genAMTrajFromPhaseStates
from mlp.utils.cs_tools import computePhasesCOMValues
import math
VERBOSE = False

multicontact_api.switchToNumpyArray()

class Inputs(Requirements):
    timings = True
    consistentContacts = True

class Outputs(Inputs):
    centroidalTrajectories = True
    COMvalues = True


def generateCentroidalTrajectory(cs, cs_initGuess=None, fullBody=None, viewer=None, first_iter = True):
    """
    Generate straight line trajectories from the center of the support polygon of one phase
    to the center in the next phase.
    Compute trajectories as Quintic splines, starting and ending with null acceleration and velocity at each phase change
    Add a null angular momentum.
    :param cs:
    :param cs_initGuess:
    :param fullBody:
    :param viewer:
    :return:
    """
    if cs_initGuess:
        print("WARNING : in centroidal.geometric, initial guess is ignored.")
    if not first_iter:
        print("WARNING : in centroidal.geometric, it is useless to iterate several times.")

    if cs.haveCOMvalues():
        # do not overwrite com values in input sequence
        cs_result = ContactSequence(cs)
        computePhasesCOMValues(cs_result, cfg.Robot.DEFAULT_COM_HEIGHT, overwrite= True)
    else:
        # add them in output sequence if not present
        computePhasesCOMValues(cs, cfg.Robot.DEFAULT_COM_HEIGHT)
        cs_result = ContactSequence(cs)

    for pid,phase in enumerate(cs_result.contactPhases):
        genCOMTrajFromPhaseStates(phase)
        genAMTrajFromPhaseStates(phase)


    return cs_result
