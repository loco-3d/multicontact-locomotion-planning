import pinocchio as pin
from pinocchio import SE3, Quaternion
from pinocchio.utils import *
import inspect
import mlp.config as cfg
import multicontact_api
from multicontact_api import ContactPhase, ContactSequence, ContactPatch
from mlp.utils.util import quatFromConfig
from mlp.utils.cs_tools import createPhaseFromConfig, copyPhaseInitToFinal
from mlp.utils.requirements import Requirements
from numpy import array
import enum
import importlib
multicontact_api.switchToNumpyArray()

VERBOSE = True


class Outputs(Requirements):
    consistentContacts = True
    COMvalues = True
    configurationValues = True

class VariationType(enum.Enum):
    NONE = 0
    BROKEN = 1
    CREATED = 2
    REPOSITIONNED = 3


def contactPlacementFromRBPRMState(fb, stateId, eeName):
    # get limbName from effector name :
    limbId = list(fb.dict_limb_joint.keys())[list(fb.dict_limb_joint.values()).index(eeName)]
    [p, n] = fb.clientRbprm.rbprm.computeCenterOfContactAtStateForLimb(stateId, False, limbId)
    placement = SE3.Identity()
    placement.translation = array(p)
    if fb.cType == "_3_DOF":
        normal = np.array(n)
        quat = Quaternion.FromTwoVectors(np.array(fb.dict_normal[eeName]), normal)
    else:
        q_r = fb.getJointPosition(eeName)
        quat = quatFromConfig(q_r)
    placement.rotation = quat.matrix()
    return placement


def createPhaseFromRBPRMState(fb, stateId, t_init = -1):
    q = fb.getConfigAtState(stateId)
    limbs_contact = fb.getAllLimbsInContact(stateId)
    return createPhaseFromConfig(fb, q, limbs_contact, t_init)

def getContactVariationBetweenStates(fb, s1, s2):
    # Determine the contact changes which are going to occur :
    variations = fb.getContactsVariations(s1, s2)
    if VERBOSE:
        print("Contact variations : ", variations)
    assert len(variations) <= 1, "Several changes of contacts in adjacent states, not implemented yet !"
    variationValue = VariationType.NONE.value
    if len(variations) == 0:
        if VERBOSE:
            print("no variations !")
        movingLimb = None
    else:
        movingLimb = variations[0]
        if fb.isLimbInContact(movingLimb, s1):
            variationValue += VariationType.BROKEN.value
        if fb.isLimbInContact(movingLimb, s2):
            variationValue += VariationType.CREATED.value
        # if both if are true, it's a repositionning (it is defined by the sum of the enum types)
    variationType = VariationType(variationValue)
    if VERBOSE:
        print("movingLimb = ", movingLimb)
        print("variation type :", end ="")
        print(variationType.name)
    return fb.dict_limb_joint[movingLimb],variationType

def setPhaseInitialValues(fb, stateId, phase):
    # get q, c, dc and ddc from planning for the current state :
    q = fb.getConfigAtState(stateId)
    fb.setCurrentConfig(q)
    com = array(fb.getCenterOfMass())
    vel = array(q[-6:-3])
    acc = array(q[-3:])
    # set them in the phase :
    phase.q_init = array(q)
    phase.c_init = com
    phase.dc_init = vel
    phase.ddc_init = acc

def setPhaseFinalValues(fb, stateId, phase):
    # get q, c, dc and ddc from planning for the current state :
    q = fb.getConfigAtState(stateId)
    fb.setCurrentConfig(q)
    com = array(fb.getCenterOfMass())
    vel = array(q[-6:-3])
    acc = array(q[-3:])
    # set them in the phase :
    phase.q_final = array(q)
    phase.c_final = com
    phase.dc_final = vel
    phase.ddc_final = acc


def runRBPRMScript():
    #the following script must produce a sequence of configurations in contact (configs)
    # with exactly one contact change between each configurations
    # It must also initialise a FullBody object name fullBody and optionnaly a Viewer object named V
    if hasattr(cfg, 'SCRIPT_ABSOLUTE_PATH'):
        scriptName = cfg.SCRIPT_ABSOLUTE_PATH
    else:
        scriptName = 'scenarios.' + cfg.SCRIPT_PATH + '.' + cfg.DEMO_NAME
    print("Run RBPRM script : ", scriptName)
    cp = importlib.import_module(scriptName)
    if hasattr(cp, 'beginId'):
        beginId = cp.beginId
    else:
        beginId = 0
    if hasattr(cp, 'endId'):
        endId = cp.endId
    else:
        endId = len(cp.configs) - 1
    return cp.fullBody, cp.v, beginId, endId


def contactSequenceFromRBPRMConfigs(fb, beginId, endId):
    print("generate contact sequence from planning : ")
    n_states = endId - beginId + 1
    # There could be either contact break, creation or repositionning between each adjacent states.
    # But there should be only contacts break or creation between each adjacent contactPhases
    cs = ContactSequence(0)
    prev_phase = None
    phaseId = 0

    # create initial ContactPhase
    cs.append(createPhaseFromRBPRMState(fb, beginId))
    for stateId in range(beginId + 1, endId + 1): #from the second state to the last one
        if VERBOSE:
            print("current state id = ", stateId)
        previous_phase = cs.contactPhases[-1]
        eeName, variationType = getContactVariationBetweenStates(fb,stateId-1, stateId)

        if eeName is not None:
            if variationType == VariationType.REPOSITIONNED:
                # in case of repositionning, the centroidal motion will happend in the next intermediate phase,
                # and thus the previous_phase will not move :
                copyPhaseInitToFinal(previous_phase)
            else:
                # set the final values of the previous phase to be the current one :
                setPhaseFinalValues(fb, stateId, previous_phase)

            if variationType == VariationType.BROKEN:
                # remove the given contact :
                cs.breakContact(eeName)
            else:
                # get placement of the new contact from the planning :
                contact_placement = contactPlacementFromRBPRMState(fb,stateId,eeName)
            if variationType == VariationType.CREATED:
                # create a new contact :
                cs.createContact(eeName, ContactPatch(contact_placement))
            elif variationType == VariationType.REPOSITIONNED:
                # move existing contact to new placement :
                cs.moveEffectorToPlacement(eeName,contact_placement)
                # set the initial values for the current phase from planning :
                setPhaseInitialValues(fb, stateId, cs.contactPhases[-1])
                # set the same values as the final ones for the intermediate state created :
                setPhaseFinalValues(fb, stateId, cs.contactPhases[-2])
        # else : no contact changes, ignore this state
    setPhaseFinalValues(fb, endId, cs.contactPhases[-1])
    return cs


def generateContactSequence():
    fb, viewer, beginId, endId = runRBPRMScript()
    cs = contactSequenceFromRBPRMConfigs(fb, beginId, endId)
    return cs, fb, viewer

