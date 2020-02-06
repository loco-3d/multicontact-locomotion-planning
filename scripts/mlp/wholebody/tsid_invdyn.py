import pinocchio as pin
from pinocchio import SE3, Quaternion, Force, Motion
import tsid
import numpy as np
from numpy.linalg import norm as norm
import os
from rospkg import RosPack
import time
import subprocess
import gepetto.corbaserver
import mlp.config as cfg
import multicontact_api
from multicontact_api import WrenchCone, SOC6, ContactPatch, ContactPhase, ContactSequence
from curves import SE3Curve, piecewise
from mlp.utils.computation_tools import shiftZMPtoFloorAltitude
import mlp.viewer.display_tools as display_tools
import math
from mlp.utils.wholebody_result import Result
from mlp.utils.util import *
from mlp.end_effector import generateEndEffectorTraj, effectorCanRetry
import eigenpy
from mlp.utils.cs_tools import deleteAllTrajectories, deletePhaseWBtrajectories
from mlp.utils.requirements import Requirements
eigenpy.switchToNumpyArray()

class Inputs(Requirements):
    consistentContacts = True
    timings = True
    centroidalTrajectories = True
    effectorTrajectories = True
    rootTrajectories = True

class Outputs(Requirements):
    consistentContacts = True
    timings = True
    jointsTrajectories = True
    torqueTrajectories = True
    if cfg.IK_store_centroidal:
        centroidalTrajectories = True
    if cfg.IK_store_effector:
        effectorTrajectories = True
    if cfg.IK_store_contact_forces:
        contactForcesTrajectories = False


def buildRectangularContactPoints(eeName):
    size = cfg.IK_eff_size[eeName]
    transform = cfg.Robot.dict_offset[eeName]
    # build matrices with corners of the feet
    lxp = size[0] / 2. + transform.translation[0]  # foot length in positive x direction
    lxn = size[0] / 2. - transform.translation[0]  # foot length in negative x direction
    lyp = size[1] / 2. + transform.translation[1]  # foot length in positive y direction
    lyn = size[1] / 2. - transform.translation[1]  # foot length in negative y direction
    lz = transform.translation[2]  # foot sole height with respect to ankle joint
    contact_Point = np.ones((3, 4))
    contact_Point[0, :] = [-lxn, -lxn, lxp, lxp]
    contact_Point[1, :] = [-lyn, lyp, -lyn, lyp]
    contact_Point[2, :] = [lz] * 4
    return contact_Point


def getCurrentEffectorPosition(robot, data, eeName):
    id = robot.model().getJointId(eeName)
    if id < len(data.oMi):
        return robot.position(data, id)
    else:
        id = robot.model().getFrameId(eeName)
        return robot.framePosition(data, id)


def getCurrentEffectorVelocity(robot, data, eeName):
    id = robot.model().getJointId(eeName)
    if id < len(data.oMi):
        return robot.velocity(data, id)
    else:
        id = robot.model().getFrameId(eeName)
        return robot.frameVelocity(data, id)


def getCurrentEffectorAcceleration(robot, data, eeName):
    id = robot.model().getJointId(eeName)
    if id < len(data.oMi):
        return robot.acceleration(data, id)
    else:
        id = robot.model().getFrameId(eeName)
        return robot.frameAcceleration(data, id)


def createContactForEffector(invdyn, robot, eeName):
    """
    Add a contact task in invdyn for the given effector, at it's current placement
    :param invdyn:
    :param robot:
    :param eeName: name of the effector
    :return: the contact task
    """
    contactNormal = np.array(cfg.Robot.dict_normal[eeName])
    contactNormal = cfg.Robot.dict_offset[eeName].rotation @ contactNormal  # apply offset transform
    if cfg.Robot.cType == "_3_DOF":
        contact = tsid.ContactPoint("contact_" + eeName, robot, eeName, contactNormal, cfg.MU, cfg.fMin, cfg.fMax)
        mask = np.ones(3)
        contact.useLocalFrame(False)
    else:
        contact_Points = buildRectangularContactPoints(eeName)
        contact = tsid.Contact6d("contact_" + eeName, robot, eeName, contact_Points, contactNormal, cfg.MU, cfg.fMin,
                                 cfg.fMax)
        mask = np.ones(6)
    contact.setKp(cfg.kp_contact * mask)
    contact.setKd(2.0 * np.sqrt(cfg.kp_contact) * mask)
    ref = getCurrentEffectorPosition(robot, invdyn.data(), eeName)
    contact.setReference(ref)
    invdyn.addRigidContact(contact, cfg.w_forceRef)
    if cfg.WB_VERBOSE:
        print("create contact for effector ", eeName)
        if cfg.Robot.cType == "_3_DOF":
            print("create contact point")
        else:
            print("create rectangular contact")
            print("contact points : \n", contact_Points)
        print("contact placement : ", ref)
        print("contact_normal : ", contactNormal)
    return contact


def createEffectorTasksDic(effectorsNames, robot):
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
        effectorTask.setKp(cfg.kp_Eff * mask)
        effectorTask.setKd(2.0 * np.sqrt(cfg.kp_Eff) * mask)
        res.update({eeName: effectorTask})
    return res



def curvesToTSID(curves,t):
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

def adjustEndEffectorTrajectoryIfNeeded(phase, robot, data, eeName):
    """
    Check that the reference trajectory correctly start close enough to the current effector position
    and adjust it if required to start at the current position
    :param phase_ref:
    :param robot:
    :param data:
    :param eeName:
    :return:
    """
    current_placement = getCurrentEffectorPosition(robot, data, eeName)
    ref_placement = phase.effectorTrajectory(eeName).evaluateAsSE3(phase.timeInitial)
    if not current_placement.isApprox(ref_placement, 1e-4):
        if cfg.WB_VERBOSE:
            print("End effector trajectory need to be adjusted")

    placement_end = phase.effectorTrajectory(eeName).evaluateAsSE3(phase.timeFinal)
    ref_traj = generateEndEffectorTraj([phase.timeInitial, phase.timeFinal], current_placement, placement_end, 0)
    phase.addEffectorTrajectory(eeName, ref_traj)


def generateWholeBodyMotion(cs_ref, fullBody=None, viewer=None):
    """
    Generate the whole body motion corresponding to the given contactSequence
    :param cs: Contact sequence containing the references,
     it will only be modified if the end effector trajectories are not valid.
     New references will be generated and added to the cs
    :param fullBody:
    :param viewer:
    :return: a new ContactSequence object, containing the wholebody trajectories,
    and the other trajectories computed from the wholebody motion request with cfg.IK_STORE_*
    """

    ### define nested functions used in control loop ###
    def appendWBStateValues():
        if phase.q_t is None:
            pol_q = polynomial(q_begin,q,t-dt,t)
            phase.q_t = piecewise(pol_q)
            pol_v = polynomial(v_begin, v, t - dt, t)
            phase.dq_t = piecewise(pol_v)
        else:
            phase.q_t.append(q, t)
            phase.dq_t.append(v, t)


    def appendWBControlValues(use_previous_phase = False):
        if use_previous_phase:
            if phase_prev is not None and t > phase_prev.ddq_t.max():
                phase_prev.ddq_t.append(dv, t)
                phase_prev.tau_t.append(tau, t)
        elif phase.ddq_t is None:
            pol_dv = polynomial(dv0, dv, t - dt, t)
            pol_tau = polynomial(tau0, tau, t - dt, t)
            phase.ddq_t = piecewise(pol_dv)
            phase.tau_t = piecewise(pol_tau)
        else:
            phase.ddq_t.append(dv, t)
            phase.tau_t.append(tau, t)


    def storeData():
        """
        # store current state
        res.q_t[:, k_t] = q
        res.dq_t[:, k_t] = v
        res.ddq_t[:, k_t] = dv
        res.tau_t[:, k_t] = invdyn.getActuatorForces(sol)  # actuator forces, with external forces (contact forces)
        #store contact info (force and status)
        for eeName, contact in dic_contacts.items():
            if invdyn.checkContact(contact.name, sol):
                res.contact_activity[eeName][:, k_t] = 1
                if cfg.IK_store_contact_forces:
                    contact_forces = invdyn.getContactForce(contact.name, sol)
                    if cfg.Robot.cType == "_3_DOF":
                        contact_forces = np.vstack([contact_forces] * 4)
                    res.contact_forces[eeName][:, k_t] = contact_forces
                    res.contact_normal_force[eeName][:, k_t] = contact.getNormalForce(res.contact_forces[eeName][:,
                                                                                                                 k_t])
        # store centroidal info (real one and reference) :
        if cfg.IK_store_reference_centroidal:
            res.c_reference[:, k_t] = sampleCom.pos()
            res.dc_reference[:, k_t] = sampleCom.vel()
            res.ddc_reference[:, k_t] = sampleCom.acc()
            res.L_reference[:, k_t] = sampleAM.pos()
            res.dL_reference[:, k_t] = sampleAM.vel()
            if cfg.IK_store_zmp:
                Mcom = SE3.Identity()
                Mcom.translation = sampleCom.pos()
                Fcom = Force.Zero()
                Fcom.linear = cfg.MASS * (sampleCom.acc() - cfg.GRAVITY)
                Fcom.angular = sampleAM.vel()
                F0 = Mcom.act(Fcom)
                res.wrench_reference[:, k_t] = F0.vector
                res.zmp_reference[:, k_t] = shiftZMPtoFloorAltitude(cs, res.t_t[k_t], F0, cfg.EXPORT_OPENHRP)
        if cfg.IK_store_centroidal:
            pcom, vcom, acom = pinRobot.com(q, v, dv)
            res.c_t[:, k_t] = pcom
            res.dc_t[:, k_t] = vcom
            res.ddc_t[:, k_t] = acom
            res.L_t[:, k_t] = pinRobot.centroidalMomentum(q, v).angular
            #res.dL_t[:,k_t] = pinRobot.centroidalMomentumVariation(q,v,dv)
            # FIXME : replace both lines below by the commented line above once it is fixed in pinocchio
            # https://github.com/stack-of-tasks/pinocchio/issues/1035
            pin.dccrba(pinRobot.model, pinRobot.data, q, v)
            res.dL_t[:, k_t] = Force(pinRobot.data.Ag.dot(dv) + pinRobot.data.dAg.dot(v)).angular
            if cfg.IK_store_zmp:
                tau = pin.rnea(pinRobot.model, pinRobot.data, q, v, dv)
                # tau without external forces, only used for the 6 first
                #res.tau_t[:6,k_t] = tau[:6]
                phi0 = pinRobot.data.oMi[1].act(Force(tau[:6]))
                res.wrench_t[:, k_t] = phi0.vector
                res.zmp_t[:, k_t] = shiftZMPtoFloorAltitude(cs, res.t_t[k_t], phi0, cfg.EXPORT_OPENHRP)
        if cfg.IK_store_effector:
            for eeName in usedEffectors:  # real position (not reference)
                res.effector_trajectories[eeName][:, k_t] = SE3toVec(
                    getCurrentEffectorPosition(robot, invdyn.data(), eeName))
                res.d_effector_trajectories[eeName][:, k_t] = MotiontoVec(
                    getCurrentEffectorVelocity(robot, invdyn.data(), eeName))
                res.dd_effector_trajectories[eeName][:, k_t] = MotiontoVec(
                    getCurrentEffectorAcceleration(robot, invdyn.data(), eeName))

        return res
        """

    def printIntermediate():
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
        if norm(dv) > 1e6 or norm(v) > 1e6:
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print("/!\ ABORT : controler unstable at t = " + str(t) + "  /!\ ")
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            raise ValueError("ABORT : controler unstable at t = " + str(t))
        if math.isnan(norm(dv)) or math.isnan(norm(v)):
            print("!!!!!!    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print("/!\ ABORT : nan   at t = " + str(t) + "  /!\ ")
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            raise ValueError("ABORT : controler unstable at t = " + str(t))

    def stopHere():
        if cfg.WB_ABORT_WHEN_INVALID:
            return cs, pinRobot
        elif cfg.WB_RETURN_INVALID:
            return cs, pinRobot

    ### End of nested functions definitions ###

    if not viewer:
        print("No viewer linked, cannot display end_effector trajectories.")
    print("Start TSID ... ")

    # copy the given contact sequence to keep it as reference :
    cs = ContactSequence(cs_ref)
    # delete all the 'reference' trajectories from result (to leave room for the real trajectories stored)
    deleteAllTrajectories(cs)

    # Create a robot wrapper
    rp = RosPack()
    package_path = rp.get_path(cfg.Robot.packageName)
    urdf = package_path + '/urdf/' + cfg.Robot.urdfName + cfg.Robot.urdfSuffix + '.urdf'
    if cfg.WB_VERBOSE:
        print("load robot : ", urdf)
    #srdf = "package://" + package + '/srdf/' +  cfg.Robot.urdfName+cfg.Robot.srdfSuffix + '.srdf'
    robot = tsid.RobotWrapper(urdf, pin.StdVec_StdString(), pin.JointModelFreeFlyer(), False)
    if cfg.WB_VERBOSE:
        print("robot loaded in tsid.")
    # FIXME : tsid robotWrapper don't have all the required methods, only pinocchio have them
    pinRobot = pin.RobotWrapper.BuildFromURDF(urdf, package_path, pin.JointModelFreeFlyer(), cfg.WB_VERBOSE == 2)
    if cfg.WB_VERBOSE:
        print("pinocchio robot loaded from urdf.")

    ### Define initial state of the robot ###
    q = cs.contactPhases[0].q_init[:robot.nq].copy()
    if not q.any():
        raise RuntimeError("The contact sequence doesn't contain an initial whole body configuration")
    v = np.zeros(robot.nv)
    dv0 = np.zeros(robot.nv)
    tau0 = np.zeros(robot.nv)
    t = cs.contactPhases[0].timeInitial

    # init states list with initial state (assume joint velocity is null for t=0)
    invdyn = tsid.InverseDynamicsFormulationAccForce("tsid", robot, False)
    invdyn.computeProblemData(t, q, v)

    # add initial contacts :
    dic_contacts = {}
    for eeName in cs.contactPhases[0].effectorsInContact():
        contact = createContactForEffector(invdyn, robot, eeName)
        dic_contacts.update({eeName: contact})

    if cfg.EFF_CHECK_COLLISION:  # initialise object needed to check the motion
        from mlp.utils import check_path
        validator = check_path.PathChecker(fullBody, cfg.CHECK_DT, cfg.WB_VERBOSE)

    ### Initialize all task used  ###
    if cfg.WB_VERBOSE:
        print("initialize tasks : ")
    comTask = tsid.TaskComEquality("task-com", robot)
    comTask.setKp(cfg.kp_com * np.ones(3))
    comTask.setKd(2.0 * np.sqrt(cfg.kp_com) * np.ones(3))
    invdyn.addMotionTask(comTask, cfg.w_com, cfg.level_com, 0.0)

    amTask = tsid.TaskAMEquality("task-am", robot)
    amTask.setKp(cfg.kp_am * np.array([1., 1., 0.]))
    amTask.setKd(2.0 * np.sqrt(cfg.kp_am * np.array([1., 1., 0.])))
    invdyn.addTask(amTask, cfg.w_am, cfg.level_am)

    postureTask = tsid.TaskJointPosture("task-joint-posture", robot)
    postureTask.setKp(cfg.kp_posture * cfg.gain_vector)
    postureTask.setKd(2.0 * np.sqrt(cfg.kp_posture * cfg.gain_vector))
    postureTask.mask(cfg.masks_posture)
    invdyn.addMotionTask(postureTask, cfg.w_posture, cfg.level_posture, 0.0)
    q_ref = cfg.IK_REFERENCE_CONFIG
    samplePosture = tsid.TrajectorySample(q_ref.shape[0] - 7)
    samplePosture.pos(q_ref[7:]) # -7 because we remove the freeflyer part

    orientationRootTask = tsid.TaskSE3Equality("task-orientation-root", robot, 'root_joint')
    mask = np.ones(6)
    mask[0:3] = 0
    mask[5] = cfg.YAW_ROT_GAIN
    orientationRootTask.setKp(cfg.kp_rootOrientation * mask)
    orientationRootTask.setKd(2.0 * np.sqrt(cfg.kp_rootOrientation * mask))
    invdyn.addMotionTask(orientationRootTask, cfg.w_rootOrientation, cfg.level_rootOrientation, 0.0)

    # init effector task objects :
    usedEffectors = cs.getAllEffectorsInContact()
    dic_effectors_tasks = createEffectorTasksDic(usedEffectors, robot)


    solver = tsid.SolverHQuadProg("qp solver")
    solver.resize(invdyn.nVar, invdyn.nEq, invdyn.nIn)

    # time check
    dt = cfg.IK_dt
    if cfg.WB_VERBOSE:
        print("dt : ", dt)
    if cfg.WB_VERBOSE:
        print("tsid initialized, start control loop")
        #raw_input("Enter to start the motion (motion displayed as it's computed, may be slower than real-time)")
    time_start = time.time()

    # For each phases, create the necessary task and references trajectories :
    for pid in range(cs.size()):
        if cfg.WB_VERBOSE:
            print("## for phase : ", pid)
            print("t = ", t)
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

        if cfg.WB_VERBOSE:
            print("time_interval ", time_interval)

        # take CoM and AM trajectory from the phase, with their derivatives
        com_traj = [phase_ref.c_t, phase_ref.dc_t, phase_ref.ddc_t]
        am_traj = [phase_ref.L_t, phase_ref.dL_t]

        # add root's orientation ref from reference config :
        root_traj = phase_ref.root_t

        # add newly created contacts :
        for eeName in usedEffectors:
            if phase_prev is not None and phase_ref.isEffectorInContact(eeName) and not phase_prev.isEffectorInContact(eeName):
                invdyn.removeTask(dic_effectors_tasks[eeName].name, 0.0)  # remove pin task for this contact
                if cfg.WB_VERBOSE:
                    print("remove se3 effector task : " + dic_effectors_tasks[eeName].name)
                contact = createContactForEffector(invdyn, robot, eeName)
                dic_contacts.update({eeName: contact})
                if cfg.WB_VERBOSE:
                    print("Create contact for : " + eeName)

        # add se3 tasks for end effector when required
        for eeName in phase.effectorsWithTrajectory():
                if cfg.WB_VERBOSE:
                    print("add se3 task for " + eeName)
                task = dic_effectors_tasks[eeName]
                invdyn.addMotionTask(task, cfg.w_eff, cfg.level_eff, 0.0)
                adjustEndEffectorTrajectoryIfNeeded(phase, robot, invdyn.data(), eeName)
                if cfg.WB_VERBOSE:
                    print("t interval : ", time_interval)


        # start removing the contact that will be broken in the next phase :
        # (This tell the solver that it should start minimizing the contact force on this contact, and ideally get to 0 at the given time)
        for eeName, contact in dic_contacts.items():
            if phase_next is not None and phase.isEffectorInContact(eeName) and not phase_next.isEffectorInContact(eeName):
                transition_time = phase.timeFinal - t - dt / 2.
                if cfg.WB_VERBOSE:
                    print("\nTime %.3f Start breaking contact %s. transition time : %.3f\n" %
                          (t, contact.name, transition_time))
                invdyn.removeRigidContact(contact.name, transition_time)

        if cfg.WB_STOP_AT_EACH_PHASE:
            input('start simulation')

        # save values at the beginning of the current phase
        q_begin = q.copy()
        v_begin = v.copy()
        phase.q_init = q_begin
        if phase_prev is not None:
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
            iter_for_phase += 1
            if cfg.WB_VERBOSE:
                print("Start computation for phase " + str(pid) + ", try number :  " + str(iter_for_phase))
            # loop to generate states (q,v,a) for the current contact phase :
            while t < phase.timeFinal - (dt / 2.):

                # set traj reference for current time :
                # com
                sampleCom = curvesToTSID(com_traj,t)
                comTask.setReference(sampleCom)

                # am
                sampleAM =  curvesToTSID(am_traj,t)
                amTask.setReference(sampleAM)

                # posture
                #print "postural task ref : ",samplePosture.pos()
                postureTask.setReference(samplePosture)

                # root orientation :
                sampleRoot = curveSE3toTSID(root_traj,t)
                orientationRootTask.setReference(sampleRoot)

                if cfg.WB_VERBOSE == 2:
                    print("### references given : ###")
                    print("com  pos : ", sampleCom.pos())
                    print("com  vel : ", sampleCom.vel())
                    print("com  acc : ", sampleCom.acc())
                    print("AM   pos : ", sampleAM.pos())
                    print("AM   vel : ", sampleAM.vel())
                    print("root pos : ", sampleRoot.pos())
                    print("root vel : ", sampleRoot.vel())

                # end effector (if they exists)
                for eeName, traj in phase.effectorTrajectories().items():
                    sampleEff = curveSE3toTSID(traj,t,True)
                    dic_effectors_tasks[eeName].setReference(sampleEff)
                    if cfg.WB_VERBOSE == 2:
                        print("effector " + str(eeName) + " pos : " + str(sampleEff.pos()))
                        print("effector " + str(eeName) + " vel : " + str(sampleEff.vel()))

                # solve HQP for the current time
                HQPData = invdyn.computeProblemData(t, q, v)
                if cfg.WB_VERBOSE and t < phase.timeInitial + dt:
                    print("final data for phase ", pid)
                    HQPData.print_all()

                sol = solver.solve(HQPData)
                dv = invdyn.getAccelerations(sol)
                tau = invdyn.getActuatorForces(sol)
                if k_t == 0:
                    dv0 = dv
                    tau0 = tau
                appendWBControlValues(k_t == 0) # save the control computed inside the ContactPhase
                storeData() # TODO

                # update state by integrating the acceleration computed
                v_mean = v + 0.5 * dt * dv
                v += dt * dv
                q = pin.integrate(robot.model(), q, dt * v_mean)
                t += dt
                k_t += 1
                if t >= phase.timeFinal - (dt / 2.):
                    t = phase.timeFinal # avoid numerical imprecisions
                appendWBStateValues() # save the new states inside the ContactPhase

                if cfg.WB_VERBOSE == 2:
                    print("v = ", v)
                    print("dv = ", dv)

                if cfg.WB_VERBOSE and int(t / dt) % cfg.IK_PRINT_N == 0:
                    printIntermediate()
                try:
                    checkDiverge()
                except ValueError:
                    return stopHere()

            if iter_for_phase == 0:
                # set the final value for q :
                phase.q_final = q

            # end while t \in phase_t (loop for the current contact phase)
            if len(phase.effectorsWithTrajectory()) > 0 and cfg.EFF_CHECK_COLLISION:
                phaseValid, t_invalid = validator.check_motion(phase.q_t)
                #if iter_for_phase < 3 :# debug only, force limb-rrt
                #    phaseValid = False
                if not phaseValid:
                    if iter_for_phase == 0:
                        # save the first q_t trajectory computed, for limb-rrt
                        first_q_t = phase.q_t.copy()
                    # t_invalid is the time since the beginning of the phase
                    print("Phase " + str(pid) + " not valid at t = " + str(t_invalid))
                    if t_invalid <= cfg.EFF_T_PREDEF \
                            or t_invalid >= (phase.duration - cfg.EFF_T_PREDEF):
                        print("Motion is invalid during predefined phases, cannot change this.")
                        return stopHere()
                    if effectorCanRetry():
                        print("Try new end effector trajectory.")
                        try:
                            for eeName, ref_traj in phase.effectorTrajectories().items():
                                placement_init = ref_traj.evaluateAsSE3(phase.timeInitial)
                                placement_end = ref_traj.evaluateAsSE3(phase.timeFinal)
                                traj = generateEndEffectorTraj(time_interval, placement_init, placement_end,
                                                                   iter_for_phase + 1, first_q_t, phase_prev,
                                                                   phase, phase_next, fullBody, eeName, viewer)
                                # save the new trajectory in the phase (the wb and the reference one)
                                phase_ref.addEffectorTrajectory(eeName,traj)
                                phase.addEffectorTrajectory(eeName,traj)
                        except ValueError as e:
                            print("ERROR in generateEndEffectorTraj :")
                            print(e)
                            return stopHere()
                    else:
                        print("End effector method choosen do not allow retries, abort here.")
                        return stopHere()
            else:  # no effector motions, phase always valid (or bypass the check)
                phaseValid = True
                if cfg.WB_VERBOSE:
                    print("Phase " + str(pid) + " valid.")
            if phaseValid:
                # display the progress by moving the robot at the last configuration computed
                if viewer:
                    display_tools.displayWBconfig(viewer,q)
        #end while not phaseValid
    # end for all phases
    time_end = time.time() - time_start
    print("Whole body motion generated in : " + str(time_end) + " s.")
    if cfg.WB_VERBOSE:
        print("\nFinal COM Position  ", robot.com(invdyn.data()))
        print("Desired COM Position", cs.contactPhases[-1].c_final)

    return cs, pinRobot
