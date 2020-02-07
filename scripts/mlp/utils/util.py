import numpy as np
from numpy import cross
from numpy.linalg import norm
import pinocchio
from pinocchio import SE3, Quaternion, Motion
from pinocchio.utils import rpyToMatrix, rotate
import mlp.config as cfg #TODO : remove cfg from here and only take it as argument when required
from curves import polynomial, SE3Curve, SO3Linear
import math
import types
pinocchio.switchToNumpyArray()


def quatFromConfig(q):
    return Quaternion(q[6], q[3], q[4], q[5])


def distPointLine(p_l, x1_l, x2_l):
    p = np.matrix(p_l)
    x1 = np.matrix(x1_l)
    x2 = np.matrix(x2_l)
    return norm(cross(p - x1, p - x2)) / norm(x2 - x1)



def SE3toVec(M):
    v = np.zeros(12)
    for j in range(3):
        v[j] = M.translation[j]
        v[j + 3] = M.rotation[j, 0]
        v[j + 6] = M.rotation[j, 1]
        v[j + 9] = M.rotation[j, 2]
    return v


def MotiontoVec(M):
    v = np.zeros(6)
    for j in range(3):
        v[j] = M.linear[j]
        v[j + 3] = M.angular[j]
    return v


def SE3FromVec(vect):
    if vect.shape[0] != 12 or vect.shape[1] != 1:
        raise ValueError("SE3FromVect take as input a vector of size 12")
    placement = SE3.Identity()
    placement.translation = vect[0:3]
    rot = placement.rotation
    # depend if eigenpy.switchToNumpyArray() have been called, FIXME : there should be a better way to check this
    if len( rot[:, 0].shape ) == 1:
        rot[:, 0] = np.asarray(vect[3:6]).reshape(-1)
        rot[:, 1] = np.asarray(vect[6:9]).reshape(-1)
        rot[:, 2] = np.asarray(vect[9:12]).reshape(-1)
    else:
        rot[:, 0] = vect[3:6]
        rot[:, 1] = vect[6:9]
        rot[:, 2] = vect[9:12]
    placement.rotation = rot
    return placement


def MotionFromVec(vect):
    if vect.shape[0] != 6 or vect.shape[1] != 1:
        raise ValueError("MotionFromVec take as input a vector of size 6")
    m = Motion.Zero()
    m.linear = np.array(vect[0:3])
    m.angular = np.array(vect[3:6])
    return m


def stdVecToMatrix(std_vector):
    if len(std_vector) == 0:
        raise Exception("std_vector is Empty")
    vec_l = []
    for vec in std_vector:
        vec_l.append(vec)

    res = np.vstack(tuple(vec_l)).T
    return res


## helper method to deal with StateVectorState and other exposed c++ vectors :
# replace vec[i] with value if it already exist or append the value
def appendOrReplace(vec, i, value):
    assert len(vec) >= i, "There is an uninitialized gap in the vector."
    if i < len(vec):
        vec[i] = value
    else:
        vec.append(value)


def numpy2DToList(m):
    """
    Convert a numpy array of shape (n,m) in a list of list.
    First list is of length m and contains list of length n
    :param m:
    :return:
    """
    l = []
    for i in range(m.shape[1]):
        p = m[:, i]
        if len(p.shape) == 1:  # array
            l += [p.tolist()]  # TODO : check this
        else:  # matrix
            l += [p.tolist()]
    return l


# assume that q.size >= 7 with root pos and quaternion(x,y,z,w)
def SE3FromConfig(q):
    if isinstance(q, list):
        q = np.array(q)
    placement = SE3.Identity()
    tr = np.array(q[0:3])
    placement.translation = tr
    r = Quaternion(q[6], q[3], q[4], q[5])
    placement.rotation = r.matrix()
    return placement


# rotate the given placement of 'angle' (in radian) along axis 'axis'
# axis : either 'x' , 'y' or 'z'
def rotatePlacement(placement, axis, angle):
    T = rotate(axis, angle)
    placement.rotation = placement.rotation @ T
    return placement


def rotateFromRPY(placement, rpy):
    trans = SE3.Identity()
    trans.rotation = rpyToMatrix(rpy)
    return placement.act(trans)


# get the joint position for the given phase with the given effector name
# Note that if the effector is not in contact the phase placement may be uninitialized (==Identity)
def JointPatchForEffector(phase, eeName, Robot=None):
    if not Robot:
        Robot = cfg.Robot
    patch = phase.contactPatch(eeName)
    patch.placement = patch.placement.act(Robot.dict_offset[eeName].inverse())
    return patch


def JointPlacementForEffector(phase, eeName, Robot=None):
    return JointPatchForEffector(phase, eeName, Robot).placement




def effectorPositionFromHPPPath(fb, problem, eeName, pid, t):
    q = problem.configAtParam(pid, t)
    # compute effector pos from q :
    fb.setCurrentConfig(q)
    p = fb.getJointPosition(eeName)[0:3]
    return np.array(p)


def genAMTrajFromPhaseStates(phase, constraintVelocity = True):
    if constraintVelocity:
        am_traj = polynomial(phase.L_init, phase.dL_init, phase.L_final, phase.dL_final,
                              phase.timeInitial, phase.timeFinal)
    else:
        am_traj = polynomial(phase.L_init, phase.L_final, phase.timeInitial, phase.timeFinal)
    phase.L_t = am_traj
    phase.dL_t = am_traj.compute_derivate(1)


def genCOMTrajFromPhaseStates(phase, constraintVelocity = True, constraintAcceleration = True):
    if constraintAcceleration and not constraintVelocity:
        raise ValueError("Cannot constraints acceleration if velocity is not constrained.")
    if constraintAcceleration:
        com_traj = polynomial(phase.c_init, phase.dc_init, phase.ddc_init,
                              phase.c_final, phase.dc_final, phase.ddc_final,phase.timeInitial, phase.timeFinal)
    elif constraintVelocity:
        com_traj = polynomial(phase.c_init, phase.dc_init, phase.c_final, phase.dc_final,
                              phase.timeInitial, phase.timeFinal)
    else:
        com_traj = polynomial(phase.c_init, phase.c_final, phase.timeInitial, phase.timeFinal)
    phase.c_t = com_traj
    phase.dc_t = com_traj.compute_derivate(1)
    phase.ddc_t = com_traj.compute_derivate(2)



def copyPhaseContacts(phase_in, phase_out):
    phase_out.RF_patch = phase_in.RF_patch
    phase_out.LF_patch = phase_in.LF_patch
    phase_out.RH_patch = phase_in.RH_patch
    phase_out.LH_patch = phase_in.LH_patch



def createStateFromPhase(fullBody, phase, q=None):
    if q is None:
        q = hppConfigFromMatrice(fullBody.client.robot, phase.q_init)
    effectorsInContact = phase.effectorsInContact()
    contacts = [] # contacts should contains the limb names, not the effector names
    list_effector = list(fullBody.dict_limb_joint.values())
    for eeName in effectorsInContact:
        contacts += [list(fullBody.dict_limb_joint.keys())[list_effector.index(eeName)]]
    # FIXME : check if q is consistent with the contacts, and project it if not.
    return fullBody.createState(q, contacts)


def hppConfigFromMatrice(robot, q_matrix):
    q = q_matrix.tolist()
    extraDof = robot.getConfigSize() - q_matrix.shape[0]
    assert extraDof >= 0, "Changes in the robot model happened."
    if extraDof > 0:
        q += [0] * extraDof
    return q


def phasesHaveSameConfig(p0, p1):
    return np.array_equal(p0.q_init, p1.q_init)


def computeEffectorTranslationBetweenStates(cs, pid):
    """
    Compute the distance travelled by the effector (suppose a straight line) between
    it's contact placement in pid+1 and it's previous contact placement
    :param cs:
    :param pid:
    :return:
    """
    phase = cs.contactPhases[pid]
    next_phase = cs.contactPhases[pid+1]
    eeNames = phase.getContactsCreated(next_phase)
    if len(eeNames) > 1:
        raise NotImplementedError("Several effectors are moving during the same phase.")
    if len(eeNames) == 0 :
        # no effectors motions in this phase
        return 0.
    eeName = eeNames[0]
    i = pid
    while not cs.contactPhases[i].isEffectorInContact(eeName) and i >= 0:
        i -= 1
    if i < 0:
        # this is the first phase where this effector enter in contact
        # TODO what should we do here ?
        return 0.

    d = next_phase.contactPatch(eeName).placement.translation -  cs.contactPhases[i].contactPatch(eeName).placement.translation
    return norm(d)


def computeEffectorRotationBetweenStates(cs, pid):
    """
    Compute the rotation applied to the effector  between
    it's contact placement in pid+1 and it's previous contact placement
    :param cs:
    :param pid:
    :return:
    """
    phase = cs.contactPhases[pid]
    next_phase = cs.contactPhases[pid + 1]
    eeNames = phase.getContactsCreated(next_phase)
    if len(eeNames) > 1:
        raise NotImplementedError("Several effectors are moving during the same phase.")
    if len(eeNames) == 0:
        # no effectors motions in this phase
        return 0.
    eeName = eeNames[0]
    i = pid
    while not cs.contactPhases[pid].isEffectorInContact(eeName) and i >= 0:
        i -= 1
    if i < 0:
        # this is the first phase where this effector enter in contact
        # TODO what should we do here ?
        return 0.

    P = next_phase.contactPatch(eeName).placement.rotation
    Q = cs.contactPhases[i].contactPatch(eeName).placement.rotation
    R = P.dot(Q.T)
    tR = R.trace()
    try:
        res = abs(math.acos((tR - 1.) / 2.))
    except ValueError as e:
        print("WARNING : when computing rotation between two contacts, got error : ", e)
        print("With trace value = ", tR)
        res = 0.
    return res


def fullBodyStatesExists(cs, fb):
    lastId = fb.createState([0] * fb.getConfigSize(), []) - 1
    if lastId <= 0:
        return 0
    else:
        # TODO check with cs if all the states belong to the contact sequence and adjust lastId if necessary
        return lastId


def createFullbodyStatesFromCS(cs, fb):
    #lastId = fullBodyStatesExists(cs, fb)
    #if lastId > 0:
    #    print("States already exist in fullBody instance. endId = ", lastId)
    #    return 0, lastId
    phase_prev = cs.contactPhases[0]
    beginId = createStateFromPhase(fb, phase_prev)
    lastId = beginId
    print("CreateFullbodyStateFromCS ##################")
    print("beginId = ", beginId)
    for pid, phase in enumerate(cs.contactPhases[1:]):
        if not phasesHaveSameConfig(phase_prev, phase):
            lastId = createStateFromPhase(fb, phase)
            print("add phase " + str(pid) + " at state index : " + str(lastId))
            phase_prev = phase
    return beginId, lastId



def computeContactNormal(placement):
    z_up = np.array([0., 0., 1.])
    contactNormal = placement.rotation @ z_up
    return contactNormal


def computeContactNormalForPhase(phase, eeName):
    return computeContactNormal(getContactPlacement(phase, eeName))


def effectorStatePositionFromWB(Robot, wb_result, id, eeName):
    placement = SE3FromVec(wb_result.effector_references[eeName][:, id])
    pos = placement.act(Robot.dict_offset[eeName]).translation
    vel = wb_result.d_effector_references[eeName][0:3, id]
    acc = wb_result.dd_effector_references[eeName][0:3, id]
    return np.vstack([pos, vel, acc])


def getPhaseEffTrajectoryByName(phase, eeName, Robot):
    if eeName == Robot.rfoot:
        return phase.RF_trajectory
    if eeName == Robot.lfoot:
        return phase.LF_trajectory
    if eeName == Robot.rhand:
        return phase.RH_trajectory
    if eeName == Robot.lhand:
        return phase.LH_trajectory
    raise ValueError("Unknown effector name : " + eeName)



def rootOrientationFromFeetPlacement(phase_prev, phase, phase_next):
    """
    Compute an initial and final root orientation for the ContactPhase
    The initial orientation is a mean between both feet contact position in the current (or previous) phase
    the final orientation is with considering the newt contact position of the feet
    :param phase_prev:
    :param phase:
    :param phase_next:
    :return:
    """
    #FIXME : extract only the yaw rotation
    qr = None
    ql = None
    patchR = None
    patchL = None
    if phase.isEffectorInContact(cfg.Robot.rfoot):
        patchR = phase.contactPatch(cfg.Robot.rfoot)
    elif phase_prev is not None and phase_prev.isEffectorInContact(cfg.Robot.rfoot):
        patchR = phase_prev.contactPatch(cfg.Robot.rfoot)
    if patchR is not None:
        qr = Quaternion(patchR.placement.rotation)
        qr.x = 0
        qr.y = 0
        qr.normalize()
    if phase.isEffectorInContact(cfg.Robot.lfoot):
        patchL = phase.contactPatch(cfg.Robot.lfoot)
    elif phase_prev is not None and phase_prev.isEffectorInContact(cfg.Robot.lfoot):
        patchL = phase_prev.contactPatch(cfg.Robot.lfoot)
    if patchL is not None:
        ql = Quaternion(patchL.placement.rotation)
        ql.x = 0
        ql.y = 0
        ql.normalize()
    if ql is not None and qr is not None:
        q_rot = qr.slerp(0.5, ql)
    elif qr is not None:
        q_rot = qr
    elif ql is not None:
        q_rot = ql
    else:
        raise RuntimeError("In rootOrientationFromFeetPlacement, cannot deduce feet initial contacts positions.")
    placement_init = SE3.Identity()
    placement_init.rotation = q_rot.matrix()

    # compute the final orientation :
    if phase_next:
        if not phase.isEffectorInContact(cfg.Robot.rfoot) and phase_next.isEffectorInContact(cfg.Robot.rfoot):
            qr = Quaternion(phase_next.contactPatch(cfg.Robot.rfoot).placement.rotation)
            qr.x = 0
            qr.y = 0
            qr.normalize()
        if not phase.isEffectorInContact(cfg.Robot.lfoot) and phase_next.isEffectorInContact(cfg.Robot.lfoot):
            ql = Quaternion(phase_next.contactPatch(cfg.Robot.lfoot).placement.rotation)
            ql.x = 0
            ql.y = 0
            ql.normalize()
    if ql is not None and qr is not None:
        q_rot = qr.slerp(0.5, ql)
    elif qr is not None:
        q_rot = qr
    elif ql is not None:
        q_rot = ql
    else:
        raise RuntimeError("In rootOrientationFromFeetPlacement, cannot deduce feet initial contacts positions.")
    placement_end = SE3.Identity()
    placement_end.rotation = q_rot.matrix()
    return placement_init, placement_end



def discretizeCurve(curve,dt):
    """
    Discretize the given curve at the given dt
    return the result as an array (one column per discret point)
    In case where the time interval of the curve is not a multiple of dt, the last point is still included
    This mean that the timestep between the two last points may be less than dt
    :param curve: a curve object, require operator (), min() and max()
    :param dt: the discretization step
    :return: an array of shape (curve.dim(), numPoints) and an array corresponding to the timeline
    """
    numPoints = math.ceil((curve.max() - curve.min()) / dt )
    res = np.zeros([curve.dim(), numPoints])
    timeline = np.zeros(numPoints)
    t = curve.min()
    for i in range(numPoints):
        res[:,i] = curve(t)
        timeline[i] = t
        t += dt
        if t > curve.max():
            t = curve.max()
    return res, timeline


def discretizeDerivateCurve(curve,dt, order):
    """
    Discretize the derivative of the given curve at the given dt
    return the result as an array (one column per discret point)
    In case where the time interval of the curve is not a multiple of dt, the last point is still included
    This mean that the timestep between the two last points may be less than dt
    :param curve: a curve object, require operator (), min() and max()
    :param dt: the discretization step
    :return: an array of shape (curve.dim(), numPoints) and an array corresponding to the timeline
    """
    numPoints = math.ceil((curve.max() - curve.min()) / dt )
    res = np.zeros([curve.dim(), numPoints])
    timeline = np.zeros(numPoints)
    t = curve.min()
    for i in range(numPoints):
        res[:,i] = curve.derivate(t, order)
        timeline[i] = t
        t += dt
        if t > curve.max():
            t = curve.max()
    return res, timeline


def discretizeSE3CurveTranslation(curve,dt):
    """
    Discretize the given curve at the given dt
    return the result as an array (one column per discret point)
    In case where the time interval of the curve is not a multiple of dt, the last point is still included
    This mean that the timestep between the two last points may be less than dt
    :param curve: a SE3 curve object, require operator (), min() and max() and translation()
    :param dt: the discretization step
    :return: an array of shape (3, numPoints) and an array corresponding to the timeline
    """
    numPoints = math.ceil((curve.max() - curve.min()) / dt )
    res = np.zeros([3, numPoints])
    timeline = np.zeros(numPoints)
    t = curve.min()
    for i in range(numPoints):
        res[:,i] = curve.translation(t)
        timeline[i] = t
        t += dt
        if t > curve.max():
            t = curve.max()
    return res, timeline

def discretizeSE3CurveToVec(curve,dt):
    """
    Discretize the given curve at the given dt
    return the result as an array (one column per discret point)
    In case where the time interval of the curve is not a multiple of dt, the last point is still included
    This mean that the timestep between the two last points may be less than dt
    :param curve: a SE3 curve object, require operator (), min() and max()
    :param dt: the discretization step
    :return: an array of shape (12, numPoints) and an array corresponding to the timeline
    """
    numPoints = math.ceil((curve.max() - curve.min()) / dt )
    res = np.zeros([12, numPoints])
    timeline = np.zeros(numPoints)
    t = curve.min()
    for i in range(numPoints):
        res[:,i] = SE3toVec(curve(t))
        timeline[i] = t
        t += dt
        if t > curve.max():
            t = curve.max()
    return res, timeline

def constantSE3curve(placement, t):
    rot = SO3Linear(placement.rotation, placement.rotation, t, t)
    trans = polynomial(placement.translation.reshape(-1,1), t, t)
    return SE3Curve(trans, rot)