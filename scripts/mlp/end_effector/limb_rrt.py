import mlp.config as cfg
from pinocchio import SE3
import numpy.linalg
import numpy as np
import math
from mlp.utils.util import stdVecToMatrix, createStateFromPhase, effectorPositionFromHPPPath, discretizeCurve
import hpp_bezier_com_traj
import hpp_bezier_com_traj as bezier_com
from curves import bezier, piecewise_bezier, SE3Curve, piecewise_SE3
import quadprog
from mlp.end_effector.bezier_predef import generatePredefBeziers, generateSmoothBezierTraj
from mlp.utils.trajectories import HPPEffectorTrajectory
from mlp.utils.requirements import Requirements
hpp_bezier_com_traj.switchToNumpyArray()

class Inputs(Requirements):
    timings = True
    configurationValues = True

class Outputs(Inputs):
    effectorTrajectories = True

VERBOSE = 1
DISPLAY_RRT_PATH = True
DISPLAY_JOINT_LEVEL = True
HPP_DT = 0.01 # dt used to discretize trajectory given to hpp

# order to try weight values and number of variables :
weights_vars = [[0.5, bezier_com.ConstraintFlag.ONE_FREE_VAR, 1], [0.75, bezier_com.ConstraintFlag.ONE_FREE_VAR, 1],
                [0.85, bezier_com.ConstraintFlag.ONE_FREE_VAR, 1], [0.90, bezier_com.ConstraintFlag.ONE_FREE_VAR, 1],
                [0.95, bezier_com.ConstraintFlag.ONE_FREE_VAR, 1], [1., bezier_com.ConstraintFlag.ONE_FREE_VAR, 1],
                [0.5, bezier_com.ConstraintFlag.THREE_FREE_VAR,
                 3], [0.75, bezier_com.ConstraintFlag.THREE_FREE_VAR, 3],
                [0.85, bezier_com.ConstraintFlag.THREE_FREE_VAR,
                 3], [0.90, bezier_com.ConstraintFlag.THREE_FREE_VAR, 3],
                [0.95, bezier_com.ConstraintFlag.THREE_FREE_VAR, 3], [1., bezier_com.ConstraintFlag.THREE_FREE_VAR, 3],
                [0.5, bezier_com.ConstraintFlag.FIVE_FREE_VAR, 5], [0.75, bezier_com.ConstraintFlag.FIVE_FREE_VAR, 5],
                [0.85, bezier_com.ConstraintFlag.FIVE_FREE_VAR, 5], [0.90, bezier_com.ConstraintFlag.FIVE_FREE_VAR, 5],
                [0.95, bezier_com.ConstraintFlag.FIVE_FREE_VAR, 5], [1., bezier_com.ConstraintFlag.FIVE_FREE_VAR, 5]]
# if numTry is equal to a number in this list, recompute the the limb-rrt path. This list is made such that low weight/num vars are tried for several limb-rrt path before trying higher weight/num variables
recompute_rrt_at_tries = [1, 4, 7, 10, 16, 23]
for i in range(20):
    recompute_rrt_at_tries += [23 + i * len(weights_vars)]
# store the last limbRRT path ID computed
current_limbRRT_id = None


def effectorCanRetry():
    return True


#min (1/2)x' P x + q' x
#subject to  G x <= h
#subject to  C x  = d
def quadprog_solve_qp(P, q, G=None, h=None, C=None, d=None):
    #~ qp_G = .5 * (P + P.T)   # make sure P is symmetric
    qp_G = .5 * (P + P.T)  # make sure P is symmetric
    qp_a = -q
    if C is not None:
        if G is not None:
            qp_C = -vstack([C, G]).T
            qp_b = -hstack([d, h])
        else:
            qp_C = -C.transpose()
            qp_b = -d
        meq = C.shape[0]
    else:  # no equality constraint
        qp_C = -G.T
        qp_b = -h
        meq = 0

    if VERBOSE > 1:
        print("quadprog matrix size : ")
        print("G : ", qp_G.shape)
        print("a : ", qp_a.shape)
        print("C : ", qp_C.shape)
        print("b : ", qp_b.shape)
        print("meq = ", meq)

    qp_G = np.array(qp_G)
    qp_a = np.array(qp_a).flatten()
    qp_C = np.array(qp_C)
    qp_b = np.array(qp_b).flatten()
    if VERBOSE > 1:
        print("quadprog array size : ")
        print("G : ", qp_G.shape)
        print("a : ", qp_a.shape)
        print("C : ", qp_C.shape)
        print("b : ", qp_b.shape)
        print("meq = ", meq)

    return quadprog.solve_qp(qp_G, qp_a, qp_C, qp_b, meq)[0]


def generateLimbRRTPath(q_init, q_end, phase_previous, phase, phase_next, fullBody):
    assert fullBody and "Cannot use limb-rrt method as fullBody object is not defined."
    extraDof = int(fullBody.client.robot.getDimensionExtraConfigSpace())
    q_init = q_init.tolist() + [0] * extraDof
    q_end = q_end.tolist() + [0] * extraDof
    # create nex states in fullBody corresponding to given configuration and set of contacts
    s0 = createStateFromPhase(fullBody, phase_previous, q_init)
    s1 = createStateFromPhase(fullBody, phase_next, q_end)
    if not fullBody.isConfigValid(q_init)[0]:
        print("q_init invalid in limb-rrt : ", q_init)
        raise ValueError("init config is invalid in limb-rrt.")
    if not fullBody.isConfigValid(q_end)[0]:
        print("q_end invalid in limb-rrt : ", q_end)
        raise ValueError("goal config is invalid in limb-rrt.")
    if VERBOSE > 1:
        print("New state added, q_init = ", q_init)
        print("New state added, q_end = ", q_end)
        contacts = fullBody.getAllLimbsInContact(s0)
        fullBody.setCurrentConfig(fullBody.getConfigAtState(s0))
        print("contact at init state : ", contacts)
        for contact in contacts:
            effName = cfg.Robot.dict_limb_joint[contact]
            print("contact position for joint " + str(effName) + " = " + str(fullBody.getJointPosition(effName)[0:3]))
        contacts = fullBody.getAllLimbsInContact(s1)
        fullBody.setCurrentConfig(fullBody.getConfigAtState(s1))
        print("contact at end  state : ", contacts)
        for contact in contacts:
            effName = cfg.Robot.dict_limb_joint[contact]
            print("contact position for joint " + str(effName) + " = " + str(fullBody.getJointPosition(effName)[0:3]))

    # create a path in hpp corresponding to the discretized trajectory in phase :
    dt = HPP_DT
    c_t = discretizeCurve(phase.c_t, dt)
    v_t = discretizeCurve(phase.dc_t, dt)[:,:-1]
    a_t = discretizeCurve(phase.ddc_t, dt)[:,:-1]
    if VERBOSE > 1:
        print ("c shape : ", c_t.shape)
        print ("v shape : ", v_t.shape)
        print ("a shape : ", a_t.shape)

    fullBody.setCurrentConfig(fullBody.getConfigAtState(s0))
    com0_fb = fullBody.getCenterOfMass()
    fullBody.setCurrentConfig(fullBody.getConfigAtState(s1))
    com1_fb = fullBody.getCenterOfMass()

    ## TEST, FIXME (force com path to start/end in the com position found from q_init and q_end. :
    c_t[:, 0] = np.array(com0_fb)
    c_t[:, -1] = np.array(com1_fb)
    com0 = c_t[:,0].tolist()
    com1 = c_t[:,-1].tolist()
    if VERBOSE > 1:
        print("init com : ", com0_fb)
        print("init ref : ", com0)
        print("end  com : ", com1_fb)
        print("end  ref : ", com1)

    path_com_id = fullBody.generateComTraj(c_t.T.tolist(), v_t.T.tolist(), a_t.T.tolist(), dt)
    if VERBOSE:
        print("add com reference as hpp path with id : ", path_com_id)

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
    if VERBOSE:
        print("start limb-rrt ... ")
    paths_rrt_ids = fullBody.comRRTOnePhase(s0, s1, path_com_id, 10)
    if VERBOSE:
        print("Limb-rrt returned path(s) : ", paths_rrt_ids)
    path_rrt_id = int(paths_rrt_ids[0])

    return path_rrt_id


def generateLimbRRTTraj(time_interval,
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


def computeDistanceCostMatrices(fb, pathId, pData, T, eeName, numPoints=50):
    problem = fb.client.problem
    # build a matrice 3xnumPoints by sampling the given path :
    step = problem.pathLength(pathId) / (numPoints - 1)
    pts = np.zeros([3, numPoints])
    for i in range(numPoints):
        p = effectorPositionFromHPPPath(fb, problem, eeName, pathId, float(step * i))
        pts[:, i] = p
    return bezier_com.computeEndEffectorDistanceCost(pData, T, numPoints, pts)


def generateLimbRRTOptimizedTraj(time_interval,
                                 placement_init,
                                 placement_end,
                                 numTry,
                                 q_t=None,
                                 phase_previous=None,
                                 phase=None,
                                 phase_next=None,
                                 fullBody=None,
                                 eeName=None,
                                 viewer=None):
    if numTry == 0:
        return generateSmoothBezierTraj(time_interval, placement_init, placement_end)
    else:
        if q_t is None or phase_previous is None or phase is None or phase_next is None or not fullBody or not eeName:
            raise ValueError("Cannot compute LimbRRTOptimizedTraj for try >= 1 without optionnal arguments")
    if cfg.EFF_T_PREDEF > 0:
        predef_curves = generatePredefBeziers(time_interval, placement_init, placement_end)
    else:
        predef_curves = generateSmoothBezierTraj(time_interval, placement_init, placement_end)
    id_middle = int(math.floor(predef_curves.num_curves() / 2.))
    predef_middle = predef_curves.curve_at_index(id_middle).translation_curve()
    pos_init = predef_middle(predef_middle.min())
    pos_end = predef_middle(predef_middle.max())
    if VERBOSE:
        print("generateLimbRRTOptimizedTraj, try number " + str(numTry))
        print("bezier takeoff end : ", pos_init)
        print("bezier landing init : ", pos_end)
    t_begin = predef_middle.min()
    t_end = predef_middle.max()
    t_middle = t_end - t_begin
    if VERBOSE:
        print("t begin : ", t_begin)
        print("t end   : ", t_end)
    q_init = q_t(t_begin)
    q_end = q_t(t_end)
    global current_limbRRT_id
    # compute new limb-rrt path if needed:
    if not current_limbRRT_id or (numTry in recompute_rrt_at_tries):
        current_limbRRT_id = generateLimbRRTPath(q_init, q_end, phase_previous, phase, phase_next, fullBody)
        if viewer and cfg.DISPLAY_FEET_TRAJ and DISPLAY_RRT_PATH:
            from hpp.gepetto import PathPlayer
            pp = PathPlayer(viewer)
            pp.displayPath(current_limbRRT_id,
                           jointName=fullBody.getLinkNames(eeName)[0],
                           offset=cfg.Robot.dict_offset[eeName].translation.tolist())

    # find weight and number of variable to use from the numTry :
    for offset in reversed(recompute_rrt_at_tries):
        if numTry >= offset:
            id = numTry - offset
            break
    if VERBOSE:
        print("weights_var id = ", id)
    if id >= len(weights_vars):
        raise ValueError("Max number of try allow to find a collision-end effector trajectory reached.")
    weight = weights_vars[id][0]
    varFlag = weights_vars[id][1]
    numVars = weights_vars[id][2]
    if VERBOSE:
        print("use weight " + str(weight) + " with num free var = " + str(numVars))
    # compute constraints for the end effector trajectories :
    pData = bezier_com.ProblemData()
    pData.c0_ = predef_middle(predef_middle.min())
    pData.dc0_ = predef_middle.derivate(predef_middle.min(), 1)
    pData.ddc0_ = predef_middle.derivate(predef_middle.min(), 2)
    pData.j0_ = predef_middle.derivate(predef_middle.min(), 3)
    pData.c1_ = predef_middle(predef_middle.max())
    pData.dc1_ = predef_middle.derivate(predef_middle.max(), 1)
    pData.ddc1_ = predef_middle.derivate(predef_middle.max(), 2)
    pData.j1_ = predef_middle.derivate(predef_middle.max(), 3)
    pData.constraints_.flag_ = bezier_com.ConstraintFlag.INIT_POS \
                               | bezier_com.ConstraintFlag.INIT_VEL \
                               | bezier_com.ConstraintFlag.INIT_ACC \
                               | bezier_com.ConstraintFlag.END_ACC \
                               | bezier_com.ConstraintFlag.END_VEL \
                               | bezier_com.ConstraintFlag.END_POS \
                               | bezier_com.ConstraintFlag.INIT_JERK \
                               | bezier_com.ConstraintFlag.END_JERK \
                               | varFlag
    Constraints = bezier_com.computeEndEffectorConstraints(pData, t_middle)
    Cost_smooth = bezier_com.computeEndEffectorVelocityCost(pData, t_middle)
    Cost_distance = computeDistanceCostMatrices(fullBody, current_limbRRT_id, pData, t_middle, eeName)

    # formulate QP matrices :
    # _ prefix = previous notation (in bezier_com_traj)
    # min        x' H x + 2 g' x
    # subject to A*x <= b
    _A = Constraints.A
    _b = Constraints.b
    _H = ((1. - weight) * Cost_smooth.A + weight * Cost_distance.A)
    _g = ((1. - weight) * Cost_smooth.b + weight * Cost_distance.b)
    if VERBOSE > 1:
        print("A = ", _A)
        print("b = ", _b)
        print("H = ", _H)
        print("h = ", _g)
    """  
    _A = np.array(_A)
    _b = np.array(_b)
    _H = np.array(_H)
    _g = np.array(_g)
    """

    # quadprog notation :
    #min (1/2)x' P x + q' x
    #subject to  G x <= h
    #subject to  C x  = d
    G = _A
    h = _b.flatten()  # remove the transpose when working with array
    P = _H * 2.
    q = (_g * 2.).flatten()

    if VERBOSE > 1:
        print("G = ", G)
        print("h = ", h)
        print("P = ", P)
        print("q = ", q)
        print("Shapes : ")
        print("G : ", G.shape)
        print("h : ", h.shape)
        print("P : ", P.shape)
        print("q : ", q.shape)

    # solve the QP :
    solved = False
    try:
        res = quadprog_solve_qp(P, q, G, h)
        solved = True
    except ValueError as e:
        print("Quadprog error : ")
        print(e)
        raise ValueError(
            "Quadprog failed to solve QP for optimized limb-RRT end-effector trajectory, for try number " +
            str(numTry))
    if VERBOSE:
        print("Quadprog solved.")

    # build a bezier curve from the result of quadprog :
    vars = np.split(res, numVars)
    wps = bezier_com.computeEndEffectorConstantWaypoints(pData, t_middle)  # one wp per column
    if VERBOSE:
        print("Constant waypoints computed.")
    id_firstVar = 4  # depend on the flag defined above, but for end effector we always use this ones ...
    i = id_firstVar
    for x in vars:
        wps[:, i] = np.array(x)
        i += 1
    if VERBOSE:
        print("Variables waypoints replaced by quadprog results.")
    bezier_middle = bezier(wps,t_begin, t_end)
    # create concatenation with takeoff/landing
    pBezier = piecewise_SE3()
    for ci in range(predef_curves.num_curves()):
        if ci == id_middle:
            pBezier.append(SE3Curve(bezier_middle,placement_init.rotation, placement_end.rotation))
        else:
            pBezier.append(predef_curves.curve_at_index(ci))

    if VERBOSE:
        print("time interval     = ", time_interval[1] - time_interval[0])
    return pBezier
