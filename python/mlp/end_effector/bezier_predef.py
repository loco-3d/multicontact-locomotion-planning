import pinocchio
from pinocchio import SE3, Quaternion
import numpy.linalg
import numpy as np
from curves import bezier, piecewise_bezier, SE3Curve, piecewise_SE3
import hpp_bezier_com_traj as bezier_com
import math
from mlp.utils.requirements import Requirements
from mlp.utils.cs_tools import generate_effector_trajectories_for_sequence
pinocchio.switchToNumpyArray()

class EffectorInputsBezier(Requirements):
    consistentContacts = True
    timings = True

class EffectorOutputsBezier(EffectorInputsBezier):
    effectorTrajectories = True


def effectorCanRetry():
    return False


class Empty:
    None


def computeConstantsWithDDJerk(ddjerk, t):
    a = (1. / 6.) * ddjerk * t * t * t
    v = (1. / 24.) * ddjerk * t * t * t * t
    p = (1. / 120.) * ddjerk * t * t * t * t * t
    return p, v, a


def computePosOffset(t_predef, t_total, p_max):
    timeMid = (t_total - (2. * t_predef)) / 2.
    if t_predef > 0:
        p = p_max / (1. + 4. * timeMid / t_predef + 6. * timeMid * timeMid / (t_predef * t_predef) -
                         (timeMid * timeMid * timeMid) / (t_predef * t_predef * t_predef))
    else:
        p = p_max
    if p < 0:  # FIXME : why/when does it happen ? eg. with t_total = 3.4 and t_predef = 0.2
        p = abs(p)
    return p, 0., 0.


def computePredefConstants(cfg, t):
    #return computeConstantsWithDDJerk(250.,)
    return computePosOffset(cfg.EFF_T_PREDEF, t, cfg.p_max)


def buildPredefinedInitTraj(cfg, placement, t_total,t_min,t_max):
    p_off, v_off, a_off = computePredefConstants(cfg, t_total)
    normal = placement.rotation @ np.array([0, 0, 1])
    #print "normal used for takeoff : ",normal.T
    #print "offset used : ",p_off
    c0 = placement.translation.copy()
    c1 = placement.translation.copy()
    c1 += p_off * normal
    #print "takeoff part, c0 : ",c0.T
    #print "takeoff part, c1 : ",c1.T
    dc0 = np.zeros(3)
    #dc1 = v_off * normal
    ddc0 = np.zeros(3)
    #ddc1 = a_off * normal
    #create wp :
    n = 4.
    wps = np.zeros(([3, int(n + 1)]))
    T = t_max - t_min
    # constrained init pos and final pos. Init vel, acc and jerk = 0
    # c0
    wps[:, 0] = (c0)
    #dc0
    wps[:, 1] = ((dc0 * T / n) + c0)
    #ddc0 // * T because derivation make a T appear
    wps[:, 2] = ((n * n * c0 - n * c0 + 2. * n * dc0 * T - 2. * dc0 * T + ddc0 * T * T) / (n * (n - 1.)))
    #j0 = 0
    wps[:, 3] = ((n * n * c0 - n * c0 + 3. * n * dc0 * T - 3. * dc0 * T + 3. * ddc0 * T * T) / (n * (n - 1.)))
    #c1
    wps[:, 4] = (c1)
    return bezier(wps,t_min,t_max)


def buildPredefinedFinalTraj(cfg, placement, t_total,t_min,t_max):
    p_off, v_off, a_off = computePredefConstants(cfg, t_total)
    normal = placement.rotation @ np.array([0, 0, 1])
    #print "normal used for landing : ",normal.T
    #print "offset used : ",p_off
    c0 = placement.translation.copy()
    c1 = placement.translation.copy()
    c0 += p_off * normal
    #print "landing part, c0 : ",c0.T
    #print "landing part, c1 : ",c1.T
    dc1 = np.zeros(3)
    #dc0 = v_off * normal
    ddc1 = np.zeros(3)
    #ddc0 = a_off * normal
    #create wp :
    n = 4.
    wps = np.zeros(([3, int(n + 1)]))
    T = t_max - t_max
    # constrained init pos and final pos. final vel, acc and jerk = 0
    #c0
    wps[:, 0] = (c0)
    # j1
    wps[:, 1] = ((n * n * c1 - n * c1 - 3 * n * dc1 * T + 3 * dc1 * T + 3 * ddc1 * T * T) / (n * (n - 1)))
    #ddc1 * T ??
    wps[:, 2] = ((n * n * c1 - n * c1 - 2 * n * dc1 * T + 2 * dc1 * T + ddc1 * T * T) / (n * (n - 1)))
    #dc1
    wps[:, 3] = ((-dc1 * T / n) + c1)
    #c1
    wps[:, 4] = (c1)
    return bezier(wps, t_min, t_max)


# build a bezier curve of degree 7 that connect exactly the two given bezier up to order 3
def generatePredefMiddle(bezier_takeoff, bezier_landing, t_min,t_max):
    T = t_max - t_min
    c0 = bezier_takeoff(bezier_takeoff.max())
    dc0 = bezier_takeoff.derivate(bezier_takeoff.max(), 1)
    ddc0 = bezier_takeoff.derivate(bezier_takeoff.max(), 2)
    j0 = bezier_takeoff.derivate(bezier_takeoff.max(), 3)
    c1 = bezier_landing(bezier_landing.min())
    dc1 = bezier_landing.derivate(bezier_landing.min(), 1)
    ddc1 = bezier_landing.derivate(bezier_landing.min(), 2)
    j1 = bezier_landing.derivate(bezier_landing.min(), 3)
    n = 7
    wps = np.zeros(([3, int(n + 1)]))
    # c0
    wps[:, 0] = (c0)
    #dc0
    wps[:, 1] = ((dc0 * T / n) + c0)
    #ddc0 // * T because derivation make a T appear
    wps[:, 2] = ((n * n * c0 - n * c0 + 2. * n * dc0 * T - 2. * dc0 * T + ddc0 * T * T) / (n * (n - 1.)))
    #j0
    wps[:, 3] = ((n * n * c0 - n * c0 + 3. * n * dc0 * T - 3. * dc0 * T + 3. * ddc0 * T * T + j0 * T * T * T /
                  (n - 2)) / (n * (n - 1.)))
    # j1
    wps[:, 4] = ((n * n * c1 - n * c1 - 3 * n * dc1 * T + 3 * dc1 * T + 3 * ddc1 * T * T - j1 * T * T * T / (n - 2)) /
                 (n * (n - 1)))
    #ddc1 * T ??
    wps[:, 5] = ((n * n * c1 - n * c1 - 2 * n * dc1 * T + 2 * dc1 * T + ddc1 * T * T) / (n * (n - 1)))
    #dc1
    wps[:, 6] = ((-dc1 * T / n) + c1)
    #c1
    wps[:, 7] = (c1)
    return bezier(wps,t_min,t_max)


def generatePredefBeziers(cfg, time_interval, placement_init, placement_end):
    t_total = time_interval[1] - time_interval[0] - 2 * cfg.EFF_T_DELAY
    #print "Generate Bezier Traj :"
    #print "placement Init = ",placement_init
    #print "placement End  = ",placement_end
    #print "time interval  = ",time_interval
    # generate two curves for the takeoff/landing :
    t_takeoff_min = time_interval[0] + cfg.EFF_T_DELAY
    t_takeoff_max = t_takeoff_min + cfg.EFF_T_PREDEF
    t_landing_max = time_interval[1] - cfg.EFF_T_DELAY
    t_landing_min = t_landing_max - cfg.EFF_T_PREDEF
    bezier_takeoff = buildPredefinedInitTraj(cfg, placement_init, t_total,t_takeoff_min,t_takeoff_max)
    bezier_landing = buildPredefinedFinalTraj(cfg, placement_end, t_total,t_landing_min,t_landing_max)
    t_middle = (t_total - (2. * cfg.EFF_T_PREDEF))
    assert t_middle >= 0.1 and "Duration of swing phase too short for effector motion. Change the values of predef motion for effector or the duration of the contact phase. "
    bezier_middle = generatePredefMiddle(bezier_takeoff, bezier_landing, t_takeoff_max,t_landing_min)
    curves = piecewise_SE3()
    # create polybezier with concatenation of the 3 (or 5) curves :
    # create constant curve at the beginning and end for the delay :
    if cfg.EFF_T_DELAY > 0:
        bezier_init_zero = bezier(bezier_takeoff(bezier_takeoff.min()).reshape([-1,1]), time_interval[0], t_takeoff_min)
        # Create SE3 curves with translation and duration defined from the bezier and constant orientation:
        curves.append(SE3Curve(bezier_init_zero,placement_init.rotation, placement_init.rotation))
    curves.append(SE3Curve(bezier_takeoff, placement_init.rotation, placement_init.rotation))
    curves.append(SE3Curve(bezier_middle, placement_init.rotation, placement_end.rotation))
    curves.append(SE3Curve(bezier_landing, placement_end.rotation, placement_end.rotation))
    if cfg.EFF_T_DELAY > 0:
        bezier_end_zero = bezier(bezier_landing(bezier_landing.max()).reshape([-1,1]), t_landing_max,time_interval[1])
        # Create SE3 curves with translation and duration defined from the bezier and constant orientation:
        curves.append(SE3Curve(bezier_end_zero,placement_end.rotation,placement_end.rotation))
    return curves


def generateSmoothBezierTrajWithPredef(cfg, time_interval, placement_init, placement_end):
    predef_curves = generatePredefBeziers(cfg, time_interval, placement_init, placement_end)
    id_middle = int(math.floor(predef_curves.num_curves() / 2.))
    bezier_takeoff = predef_curves.curve_at_index(id_middle-1).translation_curve()
    bezier_landing = predef_curves.curve_at_index(id_middle+1).translation_curve()
    # update mid curve to minimize velocity along the curve:
    # set problem data for mid curve :
    pData = bezier_com.ProblemData()
    pData.c0_ = bezier_takeoff(bezier_takeoff.max())
    pData.dc0_ = bezier_takeoff.derivate(bezier_takeoff.max(), 1)
    pData.ddc0_ = bezier_takeoff.derivate(bezier_takeoff.max(), 2)
    pData.j0_ = bezier_takeoff.derivate(bezier_takeoff.max(), 3)
    pData.c1_ = bezier_landing(bezier_landing.min())
    pData.dc1_ = bezier_landing.derivate(bezier_landing.min(), 1)
    pData.ddc1_ = bezier_landing.derivate(bezier_landing.min(), 2)
    pData.j1_ = bezier_landing.derivate(bezier_landing.min(), 3)
    pData.constraints_.flag_ = bezier_com.ConstraintFlag.INIT_POS \
                               | bezier_com.ConstraintFlag.INIT_VEL \
                               | bezier_com.ConstraintFlag.INIT_ACC \
                               | bezier_com.ConstraintFlag.END_ACC \
                               | bezier_com.ConstraintFlag.END_VEL \
                               | bezier_com.ConstraintFlag.END_POS \
                               | bezier_com.ConstraintFlag.INIT_JERK \
                               | bezier_com.ConstraintFlag.END_JERK
    t_min_middle = predef_curves.curve_at_index(id_middle).min()
    t_max_middle = predef_curves.curve_at_index(id_middle).max()
    t_middle = t_max_middle - t_min_middle
    res = bezier_com.computeEndEffector(pData, t_middle)
    wp_middle = res.c_of_t.waypoints()
    bezier_middle = bezier(wp_middle,t_min_middle,t_max_middle)
    # create a new piecewise-bezier, with the predef curves except for bezier_middle :
    pBezier = piecewise_SE3()
    for i in range(predef_curves.num_curves()):
        if i == id_middle:
            pBezier.append(SE3Curve(bezier_middle, placement_init.rotation, placement_end.rotation))
        else:
            pBezier.append(predef_curves.curve_at_index(i))
    return pBezier


def generateSmoothBezierTrajWithoutPredef(cfg, time_interval, placement_init, placement_end):
    t_tot = time_interval[1] - time_interval[0]
    wps = np.zeros([3, 9])
    for i in range(4):  # init position. init vel,acc and jerk == 0
        wps[:, i] = placement_init.translation.copy()
    # compute mid point (average and offset along z)
    wps[:, 4] = (placement_init.translation + placement_end.translation) / 2.
    wps[2, 4] += cfg.p_max
    for i in range(5, 9):  # final position. final vel,acc and jerk == 0
        wps[:, i] = placement_end.translation.copy()
    translation = bezier(wps, time_interval[0], time_interval[1])
    pBezier = piecewise_SE3(SE3Curve(translation, placement_init.rotation, placement_end.rotation))
    return pBezier


def generateSmoothBezierTraj(cfg,
                             time_interval,
                             placement_init,
                             placement_end,
                             numTry=None,
                             q_t=None,
                             phase_previous=None,
                             phase=None,
                             phase_next=None,
                             fullBody=None,
                             eeName=None,
                             viewer=None):
    if numTry is not None and numTry > 0:
        raise ValueError(
            "generateSmoothBezierTraj will always produce the same trajectory, cannot be called with numTry > 0 ")
    if cfg.EFF_T_PREDEF > 0:
        return generateSmoothBezierTrajWithPredef(cfg, time_interval, placement_init, placement_end)
    else:
        return generateSmoothBezierTrajWithoutPredef(cfg, time_interval, placement_init, placement_end)

def generate_effector_trajectories_for_sequence_bezier(cfg, cs,  fullBody = None):
    return generate_effector_trajectories_for_sequence(cfg, cs, generateSmoothBezierTraj, fullBody)

"""

placement_init = SE3.Identity()
placement_init.translation = np.matrix([0.1,0.3,0]).T
placement_end = SE3.Identity()
placement_end.translation = np.matrix([0.6,0.22,0]).T
placement_end.rotation = Quaternion(0.9800666,0.1986693,0, 0).matrix()
t_total = 1.2
time_interval = [1,1+t_total]

"""
