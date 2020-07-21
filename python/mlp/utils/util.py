import numpy as np
from numpy import cross
from numpy.linalg import norm
import pinocchio
from pinocchio import SE3, Quaternion, Motion
from pinocchio.utils import  rotate
from curves import polynomial, SE3Curve, SO3Linear
from hpp.corbaserver.rbprm.rbprmstate import State, StateHelper
from random import uniform
import signal, time
from abc import ABCMeta, abstractmethod
from math import isnan
pinocchio.switchToNumpyArray()


def distPointLine(p_l, x1_l, x2_l):
    """
    Compute the distance between a point and a line defined by two points
    :param p_l: the point
    :param x1_l: one extremity of the line
    :param x2_l: the other extremity of the line
    :return: the orthogonal distance between the point and the line
    """
    p = np.array(p_l)
    x1 = np.array(x1_l)
    x2 = np.array(x2_l)
    return norm(cross(p - x1, p - x2)) / norm(x2 - x1)



def SE3toVec(M):
    """
    Convert a pinocchio.SE3 as a vector 12:
    [ translation ; rotation[:,0] ; rotation[:,1] ; rotation[:,2] ].T
    :param M: a SE3 object
    :return: a numpy array of size 12
    """
    v = np.zeros(12)
    for j in range(3):
        v[j] = M.translation[j]
        v[j + 3] = M.rotation[j, 0]
        v[j + 6] = M.rotation[j, 1]
        v[j + 9] = M.rotation[j, 2]
    return v


def MotiontoVec(M):
    """
    Convert a pinocchio.Motion as a vector of size 6:
    [Linear ; angular ].T
    :param M: a Motion object
    :return: a numpy array of size 6
    """
    v = np.zeros(6)
    for j in range(3):
        v[j] = M.linear[j]
        v[j + 3] = M.angular[j]
    return v


def SE3FromVec(vect):
    """
    Convert a vector of size 12 to a pinocchio.SE3 object. See SE3toVec()
    :param vect: a numpy array or matrix of size 12
    :return: a SE3 object
    """
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
    """
    Convert a vector of size 6 to a pinocchio.Motion object. See MotiontoVec()
    :param vect: a numpy array or matrix of size 6
    :return: a Motion object
    """
    if vect.shape[0] != 6 or vect.shape[1] != 1:
        raise ValueError("MotionFromVec take as input a vector of size 6")
    m = Motion.Zero()
    m.linear = np.array(vect[0:3])
    m.angular = np.array(vect[3:6])
    return m



def numpy2DToList(m):
    """
    Convert a numpy array of shape (n,m) to a list of list.
    First list is of length m and contain lists of length n
    :param m: the numpy array
    :return: a list of list with the elements from m
    """
    l = []
    for i in range(m.shape[1]):
        p = m[:, i]
        if len(p.shape) == 1:  # array
            l += [p.tolist()]  # TODO : check this
        else:  # matrix
            l += [p.tolist()]
    return l


def SE3FromConfig(q):
    """
    Convert a vector of size >=7 to a pinocchio.SE3 object.
    Assume that the first 3 values of the vector are the translation part, followed by a quaternion(x,y,z,w)
    :param q: a list or a numpy array of size >=7
    :return: a SE3 object
    """
    if isinstance(q, list):
        q = np.array(q)
    placement = SE3.Identity()
    tr = np.array(q[0:3])
    placement.translation = tr
    r = Quaternion(q[6], q[3], q[4], q[5])
    placement.rotation = r.matrix()
    return placement


def rotatePlacement(placement, axis, angle):
    """
    Rotate the given placement of the desired angle along the given axis
    :param placement: a pinocchio.SE3 object
    :param axis: either 'x' , 'y' or 'z'
    :param angle: desired rotation (in radian)
    :return: the updated placement
    """
    T = rotate(axis, angle)
    placement.rotation = placement.rotation @ T
    return placement


def effectorPositionFromHPPPath(fb, problem, eeName, pid, t):
    """
    Get the effector position in an HPP joint trajectory at the given time
    :param fb: an rbprm.FullBody instance
    :param problem: an hpp.corbaserver.ProblemServer instance containing the path
    :param eeName: the joint/frame name as defined in fullbody
    :param pid: the Id of the path in the problem instance
    :param t: the time index along the path
    :return: a numpy array of size 3 with the translation of the effecor expressed in the world frame
    """
    q = problem.configAtParam(pid, t)
    # compute effector pos from q :
    fb.setCurrentConfig(q)
    p = fb.getJointPosition(eeName)[0:3]
    return np.array(p)


def hppConfigFromMatrice(robot, q_matrix):
    """
    Convert a numpy array to a list, if required fill the list with 0 at the end to match the dimension defined in robot
    :param robot:
    :param q_matrix:
    :return: a list a length robot.configSize() where the head is the values in q_matrix and the tail are zero
    """
    q = q_matrix.tolist()
    extraDof = robot.getConfigSize() - q_matrix.shape[0]
    assert extraDof >= 0, "Changes in the robot model happened."
    if extraDof > 0:
        q += [0] * extraDof
    return q


def perturbateContactNormal(fb, state_id, epsilon = 1e-2):
    """
    Add a small variation (+- epsilon) to the contact normals of the given state
    :param fb:
    :param state_id:
    :param epsilon:
    :return: the new state ID, -1 if fail
    """
    state = State(fb, state_id)
    for name in state.getLimbsInContact():
        p, n = state.getCenterOfContactForLimb(name)
        n[2] += uniform(-epsilon, epsilon)
        n = np.array(n)
        state, success = StateHelper.addNewContact(state,name, p, n.tolist())
        if not success:
            return -1
    return state.sId


def computeContactNormal(placement):
    """
    Compute the contact normal assuming that it's orthogonal to the contact orientation
    :param placement: the contact placement
    :return: the normal, as a numpy array of size 3
    """
    z_up = np.array([0., 0., 1.])
    contactNormal = placement.rotation @ z_up
    return contactNormal




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
    numPoints = round((curve.max() - curve.min()) / dt ) + 1
    res = np.zeros([curve.dim(), numPoints])
    timeline = np.zeros(numPoints)
    t = curve.min() + 0.0001 # add an epsilon to be sure to be AFTER the discontinuities at each phase changes
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
    numPoints = round((curve.max() - curve.min()) / dt ) + 1
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
    numPoints = round((curve.max() - curve.min()) / dt ) + 1
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


def discretizeSE3CurveQuaternion(curve,dt):
    """
    Discretize the given curve at the given dt
    return the result as an array (one column per discret point)
    In case where the time interval of the curve is not a multiple of dt, the last point is still included
    This mean that the timestep between the two last points may be less than dt
    :param curve: a SE3 curve object, require operator (), min() and max() and rotation()
    :param dt: the discretization step
    :return: an array of shape (3, numPoints) and an array corresponding to the timeline
    """
    numPoints = round((curve.max() - curve.min()) / dt ) + 1
    res = np.zeros([4, numPoints])
    timeline = np.zeros(numPoints)
    t = curve.min()
    for i in range(numPoints):
        res[:,i] = Quaternion(curve.rotation(t)).coeffs()
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
    numPoints = round((curve.max() - curve.min()) / dt ) +1
    res = np.zeros([12, numPoints])
    timeline = np.zeros(numPoints)
    t = curve.min()
    for i in range(numPoints):
        res[:,i] = SE3toVec(curve.evaluateAsSE3(t))
        timeline[i] = t
        t += dt
        if t > curve.max():
            t = curve.max()
    return res, timeline

def constantSE3curve(placement, t_min, t_max = None):
    """
    Create a constant SE3_curve at the given placement for the given duration
    :param placement: the placement
    :param t_min: the initial time
    :param t_max: final time, if not provided the curve will have a duration of 0
    :return: the constant curve
    """
    if t_max is None:
        t_max = t_min
    rot = SO3Linear(placement.rotation, placement.rotation, t_min, t_max)
    trans = polynomial(placement.translation.reshape(-1,1), t_min, t_max)
    return SE3Curve(trans, rot)


def buildRectangularContactPoints(size, transform):
    """
    Build Array at the corners of the feet
    :param size: list of len 2 : size of the rectangle along x and y
    :param transform: an SE3 object: transform applied to all vectors
    :return: a 3x4 Array, with the 3D position of one contact point per columns
    """
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


def build_fullbody(Robot, genLimbsDB=True, context = None):
    """
    Build an rbprm FullBody instance
    :param Robot: The class of the robot
    :param genLimbsDB: if true, generate the limbs database
    :param context: An optional string that give a name to a corba context instance
    :return: a fullbody instance and a problemsolver containing this fullbody
    """
    # Local import, as they are optional dependencies
    from hpp.corbaserver import createContext, loadServerPlugin, Client, ProblemSolver
    from hpp.corbaserver.rbprm import Client as RbprmClient
    if context:
        createContext(context)
        loadServerPlugin(context, 'rbprm-corba.so')
        loadServerPlugin(context, 'affordance-corba.so')
        hpp_client = Client(context=context)
        hpp_client.problem.selectProblem(context)
        rbprm_client = RbprmClient(context=context)
    else:
        hpp_client = None
        rbprm_client = None
    fullBody = Robot(client = hpp_client, clientRbprm = rbprm_client)
    fullBody.client.robot.setDimensionExtraConfigSpace(6)
    fullBody.setJointBounds("root_joint", [-100, 100, -100, 100, -100, 100])
    fullBody.client.robot.setExtraConfigSpaceBounds([-100, 100, -100, 100, -100, 100, -100, 100, -100, 100, -100, 100])
    fullBody.setReferenceConfig(fullBody.referenceConfig[::] + [0] * 6)
    fullBody.setPostureWeights(fullBody.postureWeights[::] + [0] * 6)
    try:
        if genLimbsDB:
            fullBody.loadAllLimbs("static", nbSamples=100)
        else:
            fullBody.loadAllLimbs("static", nbSamples=1)
    except AttributeError:
        print("WARNING initScene : fullBody do not have loadAllLimbs, some scripts may fails.")
    ps = ProblemSolver(fullBody)
    fullBody.setCurrentConfig(fullBody.referenceConfig[::] + [0] * 6)
    return fullBody, ps


def computeCenterOfSupportPolygonFromState(s):
    """
    Compute the center of the support polygon from the current state and the height defined in fullbody.DEFAULT_COM_HEIGHT
    :param s: a rbprm.State defining the current contacts
    :return: a list of size 3
    """
    com = np.zeros(3)
    numContacts = float(len(s.getLimbsInContact()))
    for limbId in s.getLimbsInContact():
        com += np.array(s.getCenterOfContactForLimb(limbId)[0])
    com /= numContacts
    com[2] += s.fullBody.DEFAULT_COM_HEIGHT
    return com.tolist()



def projectCoMInSupportPolygon(s):
    """
    Project the given state CoM to the center of it's support polygon, with an height defined in fullbody.DEFAULT_COM_HEIGHT
    :param s: a rbprm.State defining the current contacts
    :return: a boolean indicating the success of the projection
    """
    desiredCOM = computeCenterOfSupportPolygonFromState(s)
    # print "try to project state to com position : ",desiredCOM
    success = False
    maxIt = 20
    #print "project state to com : ", desiredCOM
    q_save = s.q()[::]
    while not success and maxIt > 0:
        success = s.fullBody.projectStateToCOM(s.sId, desiredCOM, maxNumSample=0)
        maxIt -= 1
        desiredCOM[2] -= 0.005
    #print "success = ", success
    #print "result = ", s.q()
    if success and isnan(s.q()[0]):  # FIXME why does it happen ?
        success = False
        s.setQ(q_save)
    return success


class Loop(metaclass=ABCMeta):
    """
    Astract Class to allow users to execute self.loop at a given frequency
    with a timer while self.run can do something else.
    """
    def __init__(self, period):
        self.period = period
        signal.signal(signal.SIGALRM, self.loop)
        signal.setitimer(signal.ITIMER_REAL, period, period)
        self.run()

    def stop(self):
        signal.setitimer(signal.ITIMER_REAL, 0)
        raise KeyboardInterrupt  # our self.run is waiting for this.

    def run(self):
        # Default implementation: don't do anything
        try:
            time.sleep(1e9)
        except KeyboardInterrupt:
            pass

    @abstractmethod
    def loop(self, signum, frame):
        ...
