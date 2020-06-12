from .limb_rrt import DISPLAY_RRT_PATH, generateLimbRRTPath
from .limb_rrt import EffectorInputsLimbrrt, EffectorOutputsLimbrrt
import hpp_bezier_com_traj as bezier_com
from mlp.end_effector.bezier_predef import generatePredefBeziers, generateSmoothBezierTraj
from curves import bezier, SE3Curve, piecewise_SE3
from mlp.utils.util import effectorPositionFromHPPPath
import quadprog
import math
import numpy as np
import logging
logging.basicConfig(format='[%(name)-12s] %(levelname)-8s: %(message)s')
logger = logging.getLogger("limb-rrt-opti")
logger.setLevel(logging.WARNING) #DEBUG, INFO or WARNING

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

    logger.debug("quadprog matrix size : ")
    logger.debug("G : %s", qp_G.shape)
    logger.debug("a : %s", qp_a.shape)
    logger.debug("C : %s", qp_C.shape)
    logger.debug("b : %s", qp_b.shape)
    logger.debug("meq = %d", meq)

    qp_G = np.array(qp_G)
    qp_a = np.array(qp_a).flatten()
    qp_C = np.array(qp_C)
    qp_b = np.array(qp_b).flatten()
    logger.debug("quadprog array size : ")
    logger.debug("G : %s", qp_G.shape)
    logger.debug("a : %s", qp_a.shape)
    logger.debug("C : %s", qp_C.shape)
    logger.debug("b : %s", qp_b.shape)
    logger.debug("meq = %d", meq)

    return quadprog.solve_qp(qp_G, qp_a, qp_C, qp_b, meq)[0]


def computeDistanceCostMatrices(fb, pathId, pData, T, eeName, numPoints=50):
    problem = fb.client.problem
    # build a matrice 3xnumPoints by sampling the given path :
    step = problem.pathLength(pathId) / (numPoints - 1)
    pts = np.zeros([3, numPoints])
    for i in range(numPoints):
        p = effectorPositionFromHPPPath(fb, problem, eeName, pathId, float(step * i))
        pts[:, i] = p
    return bezier_com.computeEndEffectorDistanceCost(pData, T, numPoints, pts)




def generate_effector_trajectory_limb_rrt_optimized(cfg,
                                 time_interval,
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
        return generateSmoothBezierTraj(cfg, time_interval, placement_init, placement_end)
    else:
        if q_t is None or phase_previous is None or phase is None or phase_next is None or not fullBody or not eeName:
            raise ValueError("Cannot compute LimbRRTOptimizedTraj for try >= 1 without optionnal arguments")
    if cfg.EFF_T_PREDEF > 0:
        predef_curves = generatePredefBeziers(cfg, time_interval, placement_init, placement_end)
    else:
        predef_curves = generateSmoothBezierTraj(cfg, time_interval, placement_init, placement_end)
    id_middle = int(math.floor(predef_curves.num_curves() / 2.))
    predef_middle = predef_curves.curve_at_index(id_middle).translation_curve()
    pos_init = predef_middle(predef_middle.min())
    pos_end = predef_middle(predef_middle.max())

    logger.warning("generateLimbRRTOptimizedTraj, try number %d", numTry)
    logger.info("bezier takeoff end : %s", pos_init)
    logger.info("bezier landing init : %s", pos_end)
    t_begin = predef_middle.min()
    t_end = predef_middle.max()
    t_middle = t_end - t_begin
    logger.info("t begin : %f", t_begin)
    logger.info("t end   : %f", t_end)
    q_init = q_t(t_begin)
    q_end = q_t(t_end)
    global current_limbRRT_id
    # compute new limb-rrt path if needed:
    if not current_limbRRT_id or (numTry in recompute_rrt_at_tries):
        logger.warning("Compute new limb-rrt path ...")
        current_limbRRT_id = generateLimbRRTPath(q_init, q_end, phase_previous, phase, phase_next, fullBody)
        if viewer and cfg.DISPLAY_FEET_TRAJ and DISPLAY_RRT_PATH:
            from hpp.gepetto import PathPlayer
            pp = PathPlayer(viewer)
            pp.displayPath(current_limbRRT_id,
                           jointName=fullBody.getLinkNames(eeName)[0])

    # find weight and number of variable to use from the numTry :
    for offset in reversed(recompute_rrt_at_tries):
        if numTry >= offset:
            id = numTry - offset
            break
    logger.info("weights_var id = %d", id)
    if id >= len(weights_vars):
        raise ValueError("Max number of try allow to find a collision-end effector trajectory reached.")
    weight = weights_vars[id][0]
    varFlag = weights_vars[id][1]
    numVars = weights_vars[id][2]
    logger.warning("use weight %f with num free var = %d", weight, numVars)
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
    logger.debug("A = %s", _A)
    logger.debug("b = %s", _b)
    logger.debug("H = %s", _H)
    logger.debug("h = %s", _g)
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

    logger.debug("G = %s", G)
    logger.debug("h = %s", h)
    logger.debug("P = %s", P)
    logger.debug("q = %s", q)
    logger.debug("Shapes : ")
    logger.debug("G : %s", G.shape)
    logger.debug("h : %s", h.shape)
    logger.debug("P : %s", P.shape)
    logger.debug("q : %s", q.shape)

    # solve the QP :
    solved = False
    try:
        res = quadprog_solve_qp(P, q, G, h)
        solved = True
    except ValueError as e:
        logger.error("Quadprog error : ", exc_info=e)
        raise ValueError(
            "Quadprog failed to solve QP for optimized limb-RRT end-effector trajectory, for try number " +
            str(numTry))
    logger.info("Quadprog solved.")

    # build a bezier curve from the result of quadprog :
    vars = np.split(res, numVars)
    wps = bezier_com.computeEndEffectorConstantWaypoints(pData, t_middle)  # one wp per column
    logger.debug("Constant waypoints computed.")
    id_firstVar = 4  # depend on the flag defined above, but for end effector we always use this ones ...
    i = id_firstVar
    for x in vars:
        wps[:, i] = np.array(x)
        logger.debug("waypoint number %d : %s", i, wps[:, i])
        i += 1

    logger.debug("Variables waypoints replaced by quadprog results.")
    bezier_middle = bezier(wps,t_begin, t_end)
    # create concatenation with takeoff/landing
    pBezier = piecewise_SE3()
    for ci in range(predef_curves.num_curves()):
        if ci == id_middle:
            pBezier.append(SE3Curve(bezier_middle,placement_init.rotation, placement_end.rotation))
        else:
            pBezier.append(predef_curves.curve_at_index(ci))

    logger.info("time interval     = [%f ; %f]", pBezier.min(), pBezier.max())
    return pBezier
