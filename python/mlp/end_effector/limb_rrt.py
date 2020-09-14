import numpy as np
from mlp.utils.util import createStateFromPhase, discretizeCurve
from mlp.utils.trajectories import HPPEffectorTrajectory
from mlp.utils.requirements import Requirements
import logging
logging.basicConfig(format='[%(name)-12s] %(levelname)-8s: %(message)s')
logger = logging.getLogger("limb-rrt")
logger.setLevel(logging.ERROR) #DEBUG, INFO or WARNING

class EffectorInputsLimbrrt(Requirements):
    timings = True
    configurationValues = True

class EffectorOutputsLimbrrt(EffectorInputsLimbrrt):
    effectorTrajectories = True

DISPLAY_RRT_PATH = True
DISPLAY_JOINT_LEVEL = True
HPP_DT = 0.01 # dt used to discretize trajectory given to hpp


def effectorCanRetry():
    return True


def generateLimbRRTPath(q_init, q_end, phase_previous, phase, phase_next, fullBody):
    assert fullBody and "Cannot use limb-rrt method as fullBody object is not defined."
    extraDof = int(fullBody.client.robot.getDimensionExtraConfigSpace())
    q_init = q_init.tolist() + [0] * extraDof
    q_end = q_end.tolist() + [0] * extraDof
    # create nex states in fullBody corresponding to given configuration and set of contacts
    s0 = createStateFromPhase(fullBody, phase_previous, q_init)
    s1 = createStateFromPhase(fullBody, phase_next, q_end)
    if not fullBody.isConfigValid(q_init)[0]:
        logger.error("q_init invalid in limb-rrt : %s", q_init)
        raise ValueError("init config is invalid in limb-rrt.")
    if not fullBody.isConfigValid(q_end)[0]:
        logger.error("q_end invalid in limb-rrt : %s", q_end)
        raise ValueError("goal config is invalid in limb-rrt.")

    logger.debug("New state added, q_init = %s", q_init)
    logger.debug("New state added, q_end = %s", q_end)
    if logger.getEffectiveLevel() >= logging.DEBUG:
        contacts = fullBody.getAllLimbsInContact(s0)
        fullBody.setCurrentConfig(fullBody.getConfigAtState(s0))
        logger.debug("contact at init state : %s", contacts)
        for contact in contacts:
            effName = fullBody.dict_limb_joint[contact]
            logger.debug("contact position for joint %s = %s",effName, fullBody.getJointPosition(effName)[0:3])
        contacts = fullBody.getAllLimbsInContact(s1)
        fullBody.setCurrentConfig(fullBody.getConfigAtState(s1))
        logger.debug("contact at end  state : %s", contacts)
        for contact in contacts:
            effName = fullBody.dict_limb_joint[contact]
            logger.debug("contact position for joint %s = %s",effName, fullBody.getJointPosition(effName)[0:3])

    # create a path in hpp corresponding to the discretized trajectory in phase :
    dt = HPP_DT
    c_t = discretizeCurve(phase.c_t, dt)[0]
    v_t = discretizeCurve(phase.dc_t, dt)[0][:,:-1]
    a_t = discretizeCurve(phase.ddc_t, dt)[0][:,:-1]
    logger.debug("c shape : %s", c_t.shape)
    logger.debug("v shape : %s", v_t.shape)
    logger.debug("a shape : %s", a_t.shape)
    logger.debug("c_t = %s", c_t.T.tolist())
    logger.debug("dc_t = %s", v_t.T.tolist())
    logger.debug("ddc_t = %s", a_t.T.tolist())

    fullBody.setCurrentConfig(fullBody.getConfigAtState(s0))
    com0_fb = fullBody.getCenterOfMass()
    fullBody.setCurrentConfig(fullBody.getConfigAtState(s1))
    com1_fb = fullBody.getCenterOfMass()

    ## TEST, FIXME (force com path to start/end in the com position found from q_init and q_end. :
    c_t[:, 0] = np.array(com0_fb)
    c_t[:, -1] = np.array(com1_fb)
    com0 = c_t[:,0].tolist()
    com1 = c_t[:,-1].tolist()
    logger.debug("init com : %s", com0_fb)
    logger.debug("init ref : %s", com0)
    logger.debug("end  com : %s", com1_fb)
    logger.debug("end  ref : %s", com1)

    path_com_id = fullBody.generateComTraj(c_t.T.tolist(), v_t.T.tolist(), a_t.T.tolist(), dt)
    logger.info("add com reference as hpp path with id : %d", path_com_id)

    # project this states to the new COM position in phase :
    """
    successProj=fullBody.projectStateToCOM(s0,com0)
    assert successProj and "Error during projection of state"+str(s0)+" to com position : "+str(com0)
    successProj=fullBody.projectStateToCOM(s1,com1)
    assert successProj and "Error during projection of state"+str(s1)+" to com position : "+str(com1)
    q_init = fullBody.getConfigAtState(s0)
    q_end = fullBody.getConfigAtState(s1)
    if extraDof: 
        q_init[-extraDof:] = [0]*extraDof #TODO : fix this in the projection method
        q_end[-extraDof:] = [0]*extraDof
        fullBody.setConfigAtState(s0,q_init)
        fullBody.setConfigAtState(s1,q_end)
    """

    # run limb-rrt in hpp :

    logger.info("start limb-rrt ... ")
    paths_rrt_ids = fullBody.comRRTOnePhase(s0, s1, path_com_id, 10)
    logger.info("Limb-rrt returned path(s) : %s", paths_rrt_ids)
    path_rrt_id = int(paths_rrt_ids[0])

    return path_rrt_id


def generate_effector_trajectory_limb_rrt(cfg,
                        time_interval,
                        placement_init,
                        placement_end,
                        numTry,
                        q_t,
                        phase_previous=None,
                        phase=None,
                        phase_next=None,
                        fullBody=None,
                        eeName=None,
                        viewer=None):

    q_init = q_t(time_interval[0])
    q_end = q_t(time_interval[1])
    pathId = generateLimbRRTPath(q_init, q_end, phase_previous, phase, phase_next, fullBody)

    if viewer and cfg.DISPLAY_FEET_TRAJ and DISPLAY_RRT_PATH:
        from hpp.gepetto import PathPlayer
        pp = PathPlayer(viewer)
        pp.displayPath(pathId, jointName=fullBody.getLinkNames(eeName)[0])

    return HPPEffectorTrajectory(eeName, fullBody, fullBody.client.problem, pathId)

