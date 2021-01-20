import pinocchio as pin
from pinocchio import Force
try:
    import tsid
except ImportError:
    message = "ERROR: Cannot import TSID python library.\n"
    message += "Did you correctly installed it?\n"
    message +="See https://github.com/stack-of-tasks/tsid"
    raise ImportError(message)
import numpy as np
from numpy.linalg import norm as norm
import os
from rospkg import RosPack
import time
from multicontact_api import ContactPhase, ContactSequence, ContactType
from curves import piecewise, piecewise_SE3, polynomial
from mlp.utils.computation_tools import shiftZMPtoFloorAltitude
import mlp.viewer.display_tools as display_tools
import math
from mlp.utils.util import constantSE3curve, SE3toVec, MotiontoVec, buildRectangularContactPoints
from mlp.utils.requirements import Requirements
import eigenpy
from mlp.utils.cs_tools import deleteAllTrajectories, deletePhaseWBtrajectories, deletePhaseTrajectories,\
    updateContactPlacement, setPreviousFinalValues, deleteEffectorsTrajectories
import logging
logging.basicConfig(format='[%(name)-12s] %(levelname)-8s: %(message)s')
logger = logging.getLogger("tsid")
logger.setLevel(logging.ERROR) #DEBUG, INFO or WARNING

eigenpy.switchToNumpyArray()



class WholebodyInputsTsid(Requirements):
    consistentContacts = True
    friction = True
    contactModel = True
    timings = True
    centroidalTrajectories = True
    effectorTrajectories = True
    rootTrajectories = True

class WholebodyOutputsTsid(Requirements):
    consistentContacts = True
    timings = True
    jointsTrajectories = True


def getCurrentEffectorPosition(robot, data, eeName):
    """
    Get the position of the effector with the current robot Data
    :param robot: a RobotWrapper instance
    :param data: the Data used
    :param eeName: the name of the joint (or frame)
    :return: the position of the desired effector, as a pinocchio.SE3
    """
    id = robot.model().getJointId(eeName)
    if id < len(data.oMi):
        return robot.position(data, id)
    else:
        id = robot.model().getFrameId(eeName)
        return robot.framePosition(data, id)


def getCurrentEffectorVelocity(robot, data, eeName):
    """
    Get the velocity of the effector with the current robot Data
    :param robot: a RobotWrapper instance
    :param data: the Data used
    :param eeName: the name of the joint (or frame)
    :return: the position of the desired effector, as a pinocchio.Motion
    """
    id = robot.model().getJointId(eeName)
    if id < len(data.oMi):
        return robot.velocity(data, id)
    else:
        id = robot.model().getFrameId(eeName)
        return robot.frameVelocity(data, id)


def getCurrentEffectorAcceleration(robot, data, eeName):
    """
    Get the acceleration of the effector with the current robot Data
    :param robot: a RobotWrapper instance
    :param data: the Data used
    :param eeName: the name of the joint (or frame)
    :return: the position of the desired effector, as a pinocchio.Motion
    """
    id = robot.model().getJointId(eeName)
    if id < len(data.oMi):
        return robot.acceleration(data, id)
    else:
        id = robot.model().getFrameId(eeName)
        return robot.frameAcceleration(data, id)


def createContactForEffector(cfg, invdyn, robot, eeName, patch, use_force_reg = True):
    """
    Add a contact task in invdyn for the given effector, at it's current placement
    :param invdyn:
    :param robot:
    :param eeName: name of the effector
    :param patch: the ContactPatch object to use. Take friction coefficient and placement for the contact from this object
    :param use_force_reg: if True, use the cfg.w_forceRef_init otherwise use cfg.w_forceRef_end
    :return: the contact task
    """
    contactNormal = np.array(cfg.Robot.dict_normal[eeName])
    contactNormal = cfg.Robot.dict_offset[eeName].rotation @ contactNormal  # apply offset transform
    logger.info("create contact for effector %s", eeName)
    logger.info("contact placement : %s", patch.placement)
    logger.info("contact_normal : %s", contactNormal)
    if patch.contact_model.contact_type == ContactType.CONTACT_POINT:
        contact = tsid.ContactPoint("contact_" + eeName, robot, eeName, contactNormal, patch.friction, cfg.fMin, cfg.fMax)
        mask = np.ones(3)
        force_ref = np.zeros(3)
        contact.useLocalFrame(False)
        logger.info("create contact point")
    elif patch.contact_model.contact_type == ContactType.CONTACT_PLANAR:
        contact_points = patch.contact_model.contact_points_positions
        contact = tsid.Contact6d("contact_" + eeName, robot, eeName, contact_points, contactNormal, patch.friction, cfg.fMin,
                                 cfg.fMax)
        mask = np.ones(6)
        force_ref = np.zeros(6)
        logger.info("create rectangular contact")
        logger.info("contact points : \n %s", contact_points)
    else:
        raise RuntimeError("Unknown contact type : ", patch.contact_model.contact_type)
    contact.setKp(cfg.kp_contact * mask)
    contact.setKd(2.0 * np.sqrt(cfg.kp_contact) * mask)
    contact.setReference(patch.placement)
    contact.setForceReference(force_ref)
    if use_force_reg:
        w_forceRef = cfg.w_forceRef_init
    else:
        w_forceRef = cfg.w_forceRef_end
    invdyn.addRigidContact(contact, w_forceRef)
    return contact


def createEffectorTasksDic(cfg, effectorsNames, robot):
    """
    Build a dic with keys = effector names, value = Effector tasks objects
    :param effectorsNames:
    :param robot:
    :return: the dict
    """
    res = {}
    for eeName in effectorsNames:
        # build effector task object
        effectorTask = tsid.TaskSE3Equality("task-" + eeName, robot, eeName)
        mask = np.ones(6)
        if cfg.Robot.cType == "_3_DOF":
            mask[3:6] = np.zeros(3) # ignore rotation for contact points
        effectorTask.setMask(mask)
        effectorTask.setKp(cfg.kp_Eff * mask)
        effectorTask.setKd(2.0 * np.sqrt(cfg.kp_Eff) * mask)
        res.update({eeName: effectorTask})
    return res



def curvesToTSID(curves,t):
    """
    Build a tsid.TrajectorySample from the given curve and time index
    :param curves: a list of curves for the position, velocity, and optionally acceleration
    :param t: the time index
    :return: a tsid.TrajectorySample object with the values from the curves
    """
    # adjust t to bounds, required due to precision issues:
    if curves[0].min() > t > curves[0].min() - 1e-3:
        t = curves[0].min()
    if curves[0].max() < t < curves[0].max() + 1e-3:
        t = curves[0].max()
    sample = tsid.TrajectorySample(curves[0].dim())
    sample.pos(curves[0](t))
    sample.vel(curves[1](t))
    if len(curves) == 3:
        sample.acc(curves[2](t))
    return sample

def curveSE3toTSID(curve,t, computeAcc = False):
    """
    Build a tsid.TrajectorySample from the given SE3 curve and time index
    :param curve: an SE3 curve
    :param t: the time index
    :param computeAcc: if True, derivate twice the given curve and set velocity and acceleration,
    if False only derivate once and set the velocity
    :return: a tsid.TrajectorySample object with the values from the curves
    """
    # adjust t to bounds, required due to precision issues:
    if curve.min() > t > curve.min() - 1e-3:
        t = curve.min()
    if curve.max() < t < curve.max() + 1e-3:
        t = curve.max()
    sample = tsid.TrajectorySample(12,6)
    placement = curve.evaluateAsSE3(t)
    vel = curve.derivateAsMotion(t,1)
    sample.pos(SE3toVec(placement))
    sample.vel(MotiontoVec(vel))
    if computeAcc:
        acc = curve.derivateAsMotion(t, 2)
        sample.acc(MotiontoVec(acc))
    return sample

def adjustEndEffectorTrajectoryIfNeeded(cfg, phase, robot, data, eeName, effectorMethod):
    """
    Check that the reference trajectory correctly start close enough to the current effector position
    and adjust it if required to start at the current position
    :param phase: the ContactPhase used
    :param robot: RobotWrapper instance
    :param data: Robotwrapper.data instance
    :param eeName: Name of the effector
    :param effectorMethod: Pointer to the method used to generate end-effector trajectory for one phase
    """
    current_placement = getCurrentEffectorPosition(robot, data, eeName)
    ref_placement = phase.effectorTrajectory(eeName).evaluateAsSE3(phase.timeInitial)
    if not current_placement.isApprox(ref_placement, 1e-2):
        logger.warning("- End effector trajectory need to be adjusted.")
        placement_end = phase.effectorTrajectory(eeName).evaluateAsSE3(phase.timeFinal)
        ref_traj = effectorMethod(cfg, [phase.timeInitial, phase.timeFinal], current_placement, placement_end, 0)
        phase.addEffectorTrajectory(eeName, ref_traj)


def generate_wholebody_tsid(cfg, cs_ref, fullBody=None, viewer=None, robot=None, queue_qt = None):
    """
    Generate the whole body motion corresponding to the given contactSequence
    :param cs: Contact sequence containing the references,
     it will only be modified if the end effector trajectories are not valid.
     New references will be generated and added to the cs
    :param fullBody: Required to compute collision free end effector trajectories
    :param viewer: If provided, and the settings are enabled, display the end effector trajectories and the last step computed
    :param robot: a tsid.RobotWrapper instance. If None, a new one is created from the urdf files defined in cfg
    :param queue_qt: If not None, the joint trajectories are send to this multiprocessing.Queue during computation
        The queue take a tuple: [q_t (a Curve object), ContactPhase (may be None), Bool (True mean that this is the
         last trajectory of the motion)]
    :return: a new ContactSequence object, containing the wholebody trajectories,
    and the other trajectories computed from the wholebody motion request with cfg.IK_STORE_* and a robotWrapper instance
    """

    # define nested functions used in control loop #

    def appendJointsValues(first_iter_for_phase = False):
        """
        Append the current q value to the current phase.q_t trajectory
        :param first_iter_for_phase: if True, set the current phase.q_init value
        and initialize a new Curve for phase.q_t
        """
        if first_iter_for_phase:
            phase.q_init = q
            phase.q_t = piecewise(polynomial(q.reshape(-1,1), t, t))
            #phase.root_t = piecewise_SE3(constantSE3curve(SE3FromConfig(q) ,t))
        else:
            phase.q_t.append(q, t)
            #phase.root_t.append(SE3FromConfig(q), t)
        if queue_qt:
            queue_qt.put([phase.q_t.curve_at_index(phase.q_t.num_curves()-1),
                          phase.dq_t.curve_at_index(phase.dq_t.num_curves()-1),
                          None,
                          False])

    def appendJointsDerivatives(first_iter_for_phase=False):
        """
        Append the current v and dv value to the current phase.dq_t and phase.ddq_t trajectory
        :param first_iter_for_phase: if True, initialize a new Curve for phase.dq_t and phase.ddq_t
        """
        if first_iter_for_phase:
            phase.dq_t = piecewise(polynomial(v.reshape(-1, 1), t, t))
            phase.ddq_t = piecewise(polynomial(dv.reshape(-1, 1), t, t))
        else:
            phase.dq_t.append(v, t)
            phase.ddq_t.append(dv, t)

    def appendTorques(first_iter_for_phase = False):
        """
        Append the current tau value to the current phase.tau_t trajectory
        :param first_iter_for_phase: if True, initialize a new Curve for phase.tau_t
        """
        tau = invdyn.getActuatorForces(sol)
        if first_iter_for_phase:
            phase.tau_t = piecewise(polynomial(tau.reshape(-1,1), t, t))
        else:
            phase.tau_t.append(tau, t)

    def appendCentroidal(first_iter_for_phase = False):
        """
        Compute the values of the CoM position, velocity, acceleration, the anuglar momentum and it's derivative
        from the wholebody data and append them to the current phase trajectories
        :param first_iter_for_phase: if True, set the initial values for the current phase
        and initialize the centroidal trajectories
        """
        pcom, vcom, acom = pinRobot.com(q, v, dv)
        L = pinRobot.centroidalMomentum(q, v).angular
        dL = pin.computeCentroidalMomentumTimeVariation(pinRobot.model, pinRobot.data, q, v, dv).angular
        if first_iter_for_phase:
            phase.c_init = pcom
            phase.dc_init = vcom
            phase.ddc_init = acom
            phase.L_init = L
            phase.dL_init = dL
            phase.c_t = piecewise(polynomial(pcom.reshape(-1,1), t , t))
            phase.dc_t = piecewise(polynomial(vcom.reshape(-1,1), t, t))
            phase.ddc_t = piecewise(polynomial(acom.reshape(-1,1), t, t))
            phase.L_t = piecewise(polynomial(L.reshape(-1,1), t, t))
            phase.dL_t = piecewise(polynomial(dL.reshape(-1,1), t, t))
        else:
            phase.c_t.append(pcom, t)
            phase.dc_t.append(vcom, t)
            phase.ddc_t.append(acom, t)
            phase.L_t.append(L, t)
            phase.dL_t.append(dL, t)

    def appendZMP(first_iter_for_phase = False):
        """
        Compute the zmp from the current wholebody data and append it to the current phase
        :param first_iter_for_phase: if True, initialize a new Curve for phase.zmp_t
        """
        tau = pin.rnea(pinRobot.model, pinRobot.data, q, v, dv)
        # tau without external forces, only used for the 6 first
        # res.tau_t[:6,k_t] = tau[:6]
        phi0 = pinRobot.data.oMi[1].act(Force(tau[:6]))
        wrench = phi0.vector
        zmp = shiftZMPtoFloorAltitude(cs, t, phi0, cfg.Robot)
        if first_iter_for_phase:
            phase.zmp_t = piecewise(polynomial(zmp.reshape(-1,1), t, t))
            phase.wrench_t = piecewise(polynomial(wrench.reshape(-1,1), t, t))
        else:
            phase.zmp_t.append(zmp, t)
            phase.wrench_t.append(wrench, t)

    def appendEffectorsTraj(first_iter_for_phase = False):
        """
        Append the current position of the effectors not in contact to the current phase trajectories
        :param first_iter_for_phase: if True, initialize a new Curve for the phase effector trajectories
        """
        if first_iter_for_phase and phase_prev:
            for eeName in phase_prev.effectorsWithTrajectory():
                if t > phase_prev.effectorTrajectory(eeName).max():
                    placement = getCurrentEffectorPosition(robot, invdyn.data(), eeName)
                    phase_prev.effectorTrajectory(eeName).append(placement, t)

        for eeName in phase.effectorsWithTrajectory():
            placement = getCurrentEffectorPosition(robot, invdyn.data(), eeName)
            if first_iter_for_phase:
                phase.addEffectorTrajectory(eeName, piecewise_SE3(constantSE3curve(placement, t)))
            else:
                phase.effectorTrajectory(eeName).append(placement, t)


    def appendContactForcesTrajs(first_iter_for_phase = False):
        """
        Append the current contact force value to the current phase, for all the effectors in contact
        :param first_iter_for_phase: if True, initialize a new Curve for the phase contact trajectories
        """
        for eeName in phase.effectorsInContact():
            contact = dic_contacts[eeName]
            if invdyn.checkContact(contact.name, sol):
                contact_forces = invdyn.getContactForce(contact.name, sol)
                contact_normal_force = np.array(contact.getNormalForce(contact_forces))
            else:
                logger.warning("invdyn check contact returned false while the reference contact is active !")
                contact_normal_force = np.zeros(1)
                if cfg.Robot.cType == "_3_DOF":
                    contact_forces = np.zeros(3)
                else:
                    contact_forces = np.zeros(12)
            if first_iter_for_phase:
                phase.addContactForceTrajectory(eeName, piecewise(polynomial(contact_forces.reshape(-1,1), t, t)))
                phase.addContactNormalForceTrajectory(eeName, piecewise(polynomial(contact_normal_force.reshape(1,1), t, t)))
            else:
                phase.contactForce(eeName).append(contact_forces, t)
                phase.contactNormalForce(eeName).append(contact_normal_force.reshape(1), t)


    def storeData(first_iter_for_phase = False):
        """
        Append all the required data (selected in the configuration file) to the current ContactPhase
        :param first_iter_for_phase: if True, set the initial values for the current phase
        and correctly initiliaze empty trajectories for this phase
        """
        if cfg.IK_store_joints_derivatives:
            appendJointsDerivatives(first_iter_for_phase)
        appendJointsValues(first_iter_for_phase)
        if cfg.IK_store_joints_torque:
            appendTorques(first_iter_for_phase)
        if cfg.IK_store_centroidal:
            appendCentroidal(first_iter_for_phase)
        if cfg.IK_store_zmp:
            appendZMP(first_iter_for_phase)
        if cfg.IK_store_effector:
            appendEffectorsTraj(first_iter_for_phase)
        if cfg.IK_store_contact_forces:
            appendContactForcesTrajs(first_iter_for_phase)


    def printIntermediate():
        """
        Print the current state: active contacts, tracking errors, computed joint acceleration and velocity
        :return:
        """
        if logger.isEnabledFor(logging.INFO):
            print("Time %.3f" % (t))
            for eeName, contact in dic_contacts.items():
                if invdyn.checkContact(contact.name, sol):
                    f = invdyn.getContactForce(contact.name, sol)
                    print("\tnormal force %s: %.1f" % (contact.name.ljust(20, '.'), contact.getNormalForce(f)))

            print("\ttracking err %s: %.3f" % (comTask.name.ljust(20, '.'), norm(comTask.position_error, 2)))
            for eeName in phase.effectorsWithTrajectory():
                task = dic_effectors_tasks[eeName]
                error = task.position_error
                if cfg.Robot.cType == "_3_DOF":
                    error = error[0:3]
                print("\ttracking err %s: %.3f" % (task.name.ljust(20, '.'), norm(error, 2)))
            print("\t||v||: %.3f\t ||dv||: %.3f" % (norm(v, 2), norm(dv)))

    def checkDiverge():
        """
        Check if either the joint velocity or acceleration is over a treshold or is NaN, and raise an error
        """
        if norm(dv) > 1e6 or norm(v) > 1e6:
            logger.error("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            logger.error("/!\ ABORT : controler unstable at t = %f  /!\ ", t)
            logger.error("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            raise ValueError("ABORT : controler unstable at t = " + str(t))
        if math.isnan(norm(dv)) or math.isnan(norm(v)):
            logger.error("!!!!!!    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            logger.error("/!\ ABORT : nan   at t = %f   /!\ ", t)
            logger.error("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            raise ValueError("ABORT : controler unstable at t = " + str(t))

    def stopHere():
        """
        Set the current data as final values for the current phase, resize the contact sequence if needed and return
        :return: [The current ContactSequence, the RobotWrapper]
        """
        setPreviousFinalValues(phase_prev, phase, cfg)
        if cfg.WB_ABORT_WHEN_INVALID:
            # cut the sequence up to the last phase
            cs.resize(pid)
            return cs, robot
        elif cfg.WB_RETURN_INVALID:
            # cut the sequence up to the current phase
            cs.resize(pid+1)
            return cs, robot

    ### End of nested functions definitions ###
    if not viewer:
        logger.warning("No viewer linked, cannot display end_effector trajectories.")
    logger.warning("Start TSID ... ")


    # copy the given contact sequence to keep it as reference :
    cs = ContactSequence(cs_ref)
    # delete all the 'reference' trajectories from result (to leave room for the real trajectories stored)
    deleteAllTrajectories(cs)

    # Create a robot wrapper

    if robot is None or cfg.IK_store_centroidal or cfg.IK_store_zmp:
        rp = RosPack()
        package_path = rp.get_path(cfg.Robot.packageName)
        urdf = package_path + '/urdf/' + cfg.Robot.urdfName + cfg.Robot.urdfSuffix + '.urdf'
    if robot is None:
        logger.info("load robot : %s", urdf)
        robot = tsid.RobotWrapper(urdf, pin.StdVec_StdString(), pin.JointModelFreeFlyer(), False)
    else:
        logger.info("Use given robot in tsid.")
    logger.info("robot loaded in tsid.")
    if cfg.IK_store_centroidal or cfg.IK_store_zmp:
        logger.info("load pinocchio robot ...")
        # FIXME : tsid robotWrapper don't have all the required methods, only pinocchio have them
        pinRobot = pin.RobotWrapper.BuildFromURDF(urdf, package_path, pin.JointModelFreeFlyer())
        logger.info("pinocchio robot loaded.")

    # get the selected end effector trajectory generation method
    effectorMethod, effectorCanRetry = cfg.get_effector_method()

    # get the selected simulator method
    Simulator = cfg.get_simulator_class()
    simulator = Simulator(cfg.IK_dt, robot.model())

    ### Define initial state of the robot ###
    phase0 = cs.contactPhases[0]
    q = phase0.q_init[:robot.nq].copy()
    if not q.any():
        raise RuntimeError("The contact sequence doesn't contain an initial whole body configuration")
    t = phase0.timeInitial
    if cs_ref.contactPhases[0].dq_t and cs_ref.contactPhases[0].dq_t.min() <= t <= cs_ref.contactPhases[0].dq_t.max():
        v = cs_ref.contactPhases[0].dq_t(t)
    else:
        v = np.zeros(robot.nv)
    logger.debug("V_init used in tsid : %s", v)

    invdyn = tsid.InverseDynamicsFormulationAccForce("tsid", robot, False)
    invdyn.computeProblemData(t, q, v)
    simulator.init(q, v)

    # add initial contacts :
    dic_contacts = {}
    for eeName in cs.contactPhases[0].effectorsInContact():
        # replace the initial contact patch placements if needed to match exactly the current position in the problem:
        updateContactPlacement(cs, 0, eeName,
                               getCurrentEffectorPosition(robot, invdyn.data(), eeName),
                               cfg.Robot.cType == "_6_DOF")
        # create the contacts :
        contact = createContactForEffector(cfg, invdyn, robot, eeName, phase0.contactPatch(eeName), False)
        dic_contacts.update({eeName: contact})

    if cfg.EFF_CHECK_COLLISION:  # initialise object needed to check the motion
        from mlp.utils import check_path
        validator = check_path.PathChecker(fullBody, cfg.CHECK_DT, logger.isEnabledFor(logging.INFO))

    ### Initialize all task used  ###
    logger.info("initialize tasks : ")
    if cfg.w_com > 0. :
        comTask = tsid.TaskComEquality("task-com", robot)
        comTask.setKp(cfg.kp_com * np.ones(3))
        comTask.setKd(2.0 * np.sqrt(cfg.kp_com) * np.ones(3))
        invdyn.addMotionTask(comTask, cfg.w_com, cfg.level_com, 0.0)
    else:
        comTask = None

    if cfg.w_am > 0.:
        amTask = tsid.TaskAMEquality("task-am", robot)
        amTask.setKp(cfg.kp_am * np.array([1., 1., 0.]))
        amTask.setKd(2.0 * np.sqrt(cfg.kp_am * np.array([1., 1., 0.])))
        invdyn.addMotionTask(amTask, cfg.w_am, cfg.level_am, 0.)
    else:
        amTask = None

    if cfg.w_posture > 0.:
        postureTask = tsid.TaskJointPosture("task-joint-posture", robot)
        postureTask.setKp(cfg.kp_posture * cfg.gain_vector)
        postureTask.setKd(2.0 * np.sqrt(cfg.kp_posture * cfg.gain_vector))
        postureTask.mask(cfg.masks_posture)
        invdyn.addMotionTask(postureTask, cfg.w_posture, cfg.level_posture, 0.0)
        q_ref = cfg.IK_REFERENCE_CONFIG
        samplePosture = tsid.TrajectorySample(q_ref.shape[0] - 7)
        samplePosture.pos(q_ref[7:]) # -7 because we remove the freeflyer part
    else :
        postureTask = None

    if cfg.w_rootOrientation > 0. :
        orientationRootTask = tsid.TaskSE3Equality("task-orientation-root", robot, 'root_joint')
        mask = np.ones(6)
        mask[0:3] = 0
        mask[5] = cfg.YAW_ROT_GAIN
        orientationRootTask.setMask(mask)
        orientationRootTask.setKp(cfg.kp_rootOrientation * mask)
        orientationRootTask.setKd(2.0 * np.sqrt(cfg.kp_rootOrientation * mask))
        invdyn.addMotionTask(orientationRootTask, cfg.w_rootOrientation, cfg.level_rootOrientation, 0.0)
    else:
        orientationRootTask = None

    # init effector task objects :
    usedEffectors = cs.getAllEffectorsInContact()
    dic_effectors_tasks = createEffectorTasksDic(cfg, usedEffectors, robot)

    # Add bounds tasks if required:
    if cfg.w_torque_bounds > 0.:
        tau_max = cfg.scaling_torque_bounds*robot.model().effortLimit[-robot.na:]
        tau_min = -tau_max
        actuationBoundsTask = tsid.TaskActuationBounds("task-actuation-bounds", robot)
        actuationBoundsTask.setBounds(tau_min, tau_max)
        invdyn.addActuationTask(actuationBoundsTask, cfg.w_torque_bounds, 0, 0.0)
    if cfg.w_joint_bounds > 0.:
        jointBoundsTask = tsid.TaskJointBounds("task-joint-bounds", robot, cfg.IK_dt)
        v_max = cfg.scaling_vel_bounds * robot.model().velocityLimit[-robot.na:]
        v_min = -v_max
        print("v_max : ", v_max)
        jointBoundsTask.setVelocityBounds(v_min, v_max)
        invdyn.addMotionTask(jointBoundsTask, cfg.w_joint_bounds, 0, 0.0)

    solver = tsid.SolverHQuadProg("qp solver")
    solver.resize(invdyn.nVar, invdyn.nEq, invdyn.nIn)

    # time check
    dt = cfg.IK_dt
    logger.info("dt : %f", dt)
    logger.info("tsid initialized, start control loop")
    #raw_input("Enter to start the motion (motion displayed as it's computed, may be slower than real-time)")
    time_start = time.time()

    # For each phases, create the necessary task and references trajectories :
    for pid in range(cs.size()):
        logger.info("## for phase : %d", pid)
        logger.info("t = %f", t)
        # phase_ref contains the reference trajectories and should not be modified exept for new effector trajectories
        # when the first ones was not collision free
        phase_ref = cs_ref.contactPhases[pid]
        # phase de not contains trajectories (exept for the end effectors) and should be modified with the new values computed
        phase = cs.contactPhases[pid]
        if pid > 0:
            phase_prev = cs.contactPhases[pid - 1]
        else:
            phase_prev = None
        if pid < cs.size() - 1:
            phase_next = cs.contactPhases[pid + 1]
        else:
            phase_next = None

        time_interval = [phase_ref.timeInitial, phase_ref.timeFinal]

        logger.info("time_interval %s", time_interval)

        # take CoM and AM trajectory from the phase, with their derivatives
        com_traj = [phase_ref.c_t, phase_ref.dc_t, phase_ref.ddc_t]
        am_traj = [phase_ref.L_t, phase_ref.L_t, phase_ref.dL_t]

        # add root's orientation ref from reference config :
        root_traj = phase_ref.root_t


        # add se3 tasks for end effector when required
        for eeName in phase.effectorsWithTrajectory():
                logger.info("add se3 task for %s", eeName)
                task = dic_effectors_tasks[eeName]
                invdyn.addMotionTask(task, cfg.w_eff, cfg.level_eff, 0.)
                adjustEndEffectorTrajectoryIfNeeded(cfg, phase_ref, robot, invdyn.data(), eeName, effectorMethod)
                logger.info("t interval : %s", time_interval)

        # add newly created contacts :
        new_contacts_names = [] # will store the names of the contact tasks created at this phase
        for eeName in usedEffectors:
            if phase_prev and phase_ref.isEffectorInContact(eeName) and not phase_prev.isEffectorInContact(eeName):
                invdyn.removeTask(dic_effectors_tasks[eeName].name, 0.0)  # remove pin task for this contact
                logger.info("remove se3 effector task : %s", dic_effectors_tasks[eeName].name)
                if logger.isEnabledFor(logging.DEBUG):
                    current_placement = getCurrentEffectorPosition(robot, invdyn.data(), eeName)
                    logger.debug("Current   effector placement : %s", current_placement)
                    logger.debug("Reference effector placement : %s", cs.contactPhases[pid].contactPatch(eeName).placement)
                updateContactPlacement(cs, pid, eeName,
                                       getCurrentEffectorPosition(robot, invdyn.data(), eeName),
                                       cfg.Robot.cType == "_6_DOF")
                contact = createContactForEffector(cfg, invdyn, robot, eeName, phase.contactPatch(eeName))
                new_contacts_names += [contact.name]
                dic_contacts.update({eeName: contact})
                logger.info("Create contact for : %s", eeName)

        # start removing the contact that will be broken in the next phase :
        # (This tell the solver that it should start minimizing the contact force on this contact, and ideally get to 0 at the given time)
        for eeName, contact in dic_contacts.items():
            if phase_next and phase.isEffectorInContact(eeName) and not phase_next.isEffectorInContact(eeName):
                transition_time = phase.duration + dt/2.
                logger.info("\nTime %.3f Start breaking contact %s. transition time : %.3f\n",
                            t, contact.name, transition_time)
                exist = invdyn.removeRigidContact(contact.name, transition_time)
                assert exist, "Try to remove a non existing contact !"

        # Remove all effectors not in contact at this phase,
        # This is required as the block above may not remove the contact exactly at the desired time
        # FIXME: why is it required ? Numerical approximation in the transition_time ?
        for eeName, contact in dic_contacts.items():
            if not phase.isEffectorInContact(eeName):
                exist = invdyn.removeRigidContact(contact.name, 0.)
                if exist:
                    logger.warning("Contact "+eeName+" was not remove after the given transition time.")

        if cfg.WB_STOP_AT_EACH_PHASE:
            input('start simulation')

        # save values at the beginning of the current phase
        q_begin = q.copy()
        v_begin = v.copy()
        phase.q_init = q_begin
        if phase_prev:
            phase_prev.q_final = q_begin
        phaseValid = False
        iter_for_phase = -1
        # iterate until a valid motion for this phase is found (ie. collision free and which respect joint-limits)
        while not phaseValid:
            deletePhaseWBtrajectories(phase) # clean previous invalid trajectories
            t = phase.timeInitial
            k_t = 0
            if iter_for_phase >= 0:
                # reset values to their value at the beginning of the current phase
                q = q_begin.copy()
                v = v_begin.copy()
                simulator.q = q
                simulator.v = v
            iter_for_phase += 1
            logger.info("Start computation for phase %d , t = %f, try number :  %d", pid, t, iter_for_phase)
            # loop to generate states (q,v,a) for the current contact phase :
            while t < phase.timeFinal - (dt / 2.):

                # set traj reference for current time :
                # com
                if comTask:
                    sampleCom = curvesToTSID(com_traj,t)
                    comTask.setReference(sampleCom)

                # am
                if amTask:
                    if cfg.IK_trackAM:
                        sampleAM =  curvesToTSID(am_traj,t)
                    else:
                        sampleAM = tsid.TrajectorySample(3)
                    amTask.setReference(sampleAM)


                # posture
                #print "postural task ref : ",samplePosture.pos()
                if postureTask:
                    postureTask.setReference(samplePosture)

                # root orientation :
                if orientationRootTask:
                    sampleRoot = curveSE3toTSID(root_traj,t)
                    orientationRootTask.setReference(sampleRoot)

                # update weight of regularization tasks for the new contacts:
                if len(new_contacts_names) > 0 :
                    # linearly decrease the weight of the tasks for the newly created contacts
                    u_w_force = (t - phase.timeInitial) / (phase.duration * cfg.w_forceRef_time_ratio)
                    if u_w_force <= 1.:
                        current_w_force = cfg.w_forceRef_init * (1. - u_w_force) + cfg.w_forceRef_end * u_w_force
                        for contact_name in new_contacts_names:
                            success = invdyn.updateRigidContactWeights(contact_name, current_w_force)
                            assert success, "Unable to change the weight of the force regularization task for contact "+contact_name

                logger.debug("### references given : ###")
                logger.debug("com  pos : %s", sampleCom.pos())
                logger.debug("com  vel : %s", sampleCom.vel())
                logger.debug("com  acc : %s", sampleCom.acc())
                if amTask:
                    logger.debug("AM   pos : %s", sampleAM.pos())
                    logger.debug("AM   vel : %s", sampleAM.vel())
                logger.debug("root pos : %s", sampleRoot.pos())
                logger.debug("root vel : %s", sampleRoot.vel())

                # end effector (if they exists)
                for eeName, traj in phase_ref.effectorTrajectories().items():
                    sampleEff = curveSE3toTSID(traj,t,True)
                    dic_effectors_tasks[eeName].setReference(sampleEff)
                    logger.debug("effector %s, pos = %s", eeName, sampleEff.pos())
                    logger.debug("effector %s, vel = %s", eeName, sampleEff.vel())

                logger.debug("previous q = %s", q)
                logger.debug("previous v = %s", v)
                # solve HQP for the current time
                HQPData = invdyn.computeProblemData(t, q, v)
                if t < phase.timeInitial + dt:
                    logger.info("final data for phase %d", pid)
                    if logger.isEnabledFor(logging.INFO):
                        HQPData.print_all()
                sol = solver.solve(HQPData)
                dv = invdyn.getAccelerations(sol)

                storeData(k_t == 0)
                q, v = simulator.simulate(dv)
                t += dt
                k_t += 1
                if t >= phase.timeFinal - (dt / 2.):
                    t = phase.timeFinal # avoid numerical imprecisions

                logger.debug("v = %s", v)
                logger.debug("dv = %s", dv)

                if int(t / dt) % cfg.IK_PRINT_N == 0:
                    printIntermediate()
                try:
                    checkDiverge()
                except ValueError:
                    return stopHere()


            # end while t \in phase_t (loop for the current contact phase)
            if len(phase.effectorsWithTrajectory()) > 0 and cfg.EFF_CHECK_COLLISION:
                phaseValid, t_invalid = validator.check_motion(phase.q_t)
                #if iter_for_phase < 3 :# debug only, force limb-rrt
                #    phaseValid = False
                #    t_invalid = (phase.timeInitial + phase.timeFinal) / 2.
                if not phaseValid:
                    if iter_for_phase == 0:
                        # save the first q_t trajectory computed, for limb-rrt
                        first_q_t = phase.q_t
                    logger.warning("Phase %d not valid at t = %f", pid, t_invalid)
                    logger.info("First invalid q : %s", phase.q_t(t_invalid))
                    if t_invalid <= (phase.timeInitial + cfg.EFF_T_PREDEF) \
                            or t_invalid >= (phase.timeFinal - cfg.EFF_T_PREDEF):
                        logger.error("Motion is invalid during predefined phases, cannot change this.")
                        return stopHere()
                    if effectorCanRetry():
                        logger.warning("Try new end effector trajectory.")
                        try:
                            for eeName, ref_traj in phase_ref.effectorTrajectories().items():
                                placement_init = ref_traj.evaluateAsSE3(phase.timeInitial)
                                placement_end = ref_traj.evaluateAsSE3(phase.timeFinal)
                                traj = effectorMethod(cfg, time_interval, placement_init, placement_end,
                                                                   iter_for_phase + 1, first_q_t, phase_prev,
                                                                   phase_ref, phase_next, fullBody, eeName, viewer)
                                # save the new trajectory in the phase with the references
                                phase_ref.addEffectorTrajectory(eeName,traj)
                                if cfg.DISPLAY_ALL_FEET_TRAJ and logger.isEnabledFor(logging.INFO):
                                    color = fullBody.dict_limb_color_traj[eeName]
                                    color[-1] = 0.6 # set small transparency
                                    display_tools.displaySE3Traj(phase_ref.effectorTrajectory(eeName),
                                                       viewer.client.gui,
                                                       viewer.sceneName,
                                                       eeName + "_traj_" + str(pid) + "_" + str(iter_for_phase),
                                                       color,
                                                       [phase.timeInitial, phase.timeFinal],
                                                       fullBody.dict_offset[eeName])
                        except ValueError as e:
                            logging.error("ERROR in generateEndEffectorTraj :", exc_info=e)
                            return stopHere()
                    else:
                        logging.error("End effector method choosen do not allow retries, abort here.")
                        return stopHere()
            else:  # no effector motions, phase always valid (or bypass the check)
                phaseValid = True
                logging.info("Phase %d valid.", pid)
            if phaseValid:
                setPreviousFinalValues(phase_prev, phase, cfg)
                # display the progress by moving the robot at the last configuration computed
                if viewer and cfg.IK_SHOW_PROGRESS:
                    display_tools.displayWBconfig(viewer,q)
        #end while not phaseValid
    # end for all phases
    # store the data of the last point
    phase_prev = phase
    phase = ContactPhase(phase_prev)
    storeData(True)
    setPreviousFinalValues(phase_prev, phase, cfg)
    time_end = time.time() - time_start
    logger.warning("Whole body motion generated in : %f s.", time_end)
    logger.info("\nFinal COM Position  %s", robot.com(invdyn.data()))
    logger.info("Desired COM Position %s", cs.contactPhases[-1].c_final)
    if queue_qt:
        queue_qt.put([phase.q_t.curve_at_index(phase.q_t.num_curves() - 1), None, True])
    return cs, robot
