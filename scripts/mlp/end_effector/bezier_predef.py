import mlp.config as cfg
import time
import os
from mlp.utils.polyBezier import *
import pinocchio as pin
from pinocchio import SE3, Quaternion
from pinocchio.utils import *
import numpy.linalg
from multicontact_api import WrenchCone, SOC6, ContactSequenceHumanoid
import numpy as np
from tools.disp_bezier import *
import curves
from curves import bezier
import hpp_bezier_com_traj as bezier_com
from mlp.utils import trajectories
from mlp.utils.util import stdVecToMatrix
import math


def effectorCanRetry():
    return False


class Empty:
    None


def computeConstantsWithDDJerk(ddjerk, t):
    a = (1. / 6.) * ddjerk * t * t * t
    v = (1. / 24.) * ddjerk * t * t * t * t
    p = (1. / 120.) * ddjerk * t * t * t * t * t
    return p, v, a


def computePosOffset(t_predef, t_total):
    timeMid = (t_total - (2. * t_predef)) / 2.
    if t_predef > 0:
        p = cfg.p_max / (1. + 4. * timeMid / t_predef + 6. * timeMid * timeMid / (t_predef * t_predef) -
                         (timeMid * timeMid * timeMid) / (t_predef * t_predef * t_predef))
    else:
        p = cfg.p_max
    if p < 0:  # FIXME : why/when does it happen ? eg. with t_total = 3.4 and t_predef = 0.2
        p = abs(p)
    return p, 0., 0.


def computePredefConstants(t):
    #return computeConstantsWithDDJerk(250.,cfg.EFF_T_PREDEF)
    return computePosOffset(cfg.EFF_T_PREDEF, t)


def buildPredefinedInitTraj(placement, t_total):
    p_off, v_off, a_off = computePredefConstants(t_total)
    normal = placement.rotation * np.matrix([0, 0, 1]).T
    #print "normal used for takeoff : ",normal.T
    #print "offset used : ",p_off
    c0 = placement.translation.copy()
    c1 = placement.translation.copy()
    c1 += p_off * normal
    #print "takeoff part, c0 : ",c0.T
    #print "takeoff part, c1 : ",c1.T
    dc0 = np.matrix(np.zeros(3)).T
    #dc1 = v_off * normal
    ddc0 = np.matrix(np.zeros(3)).T
    #ddc1 = a_off * normal
    #create wp :
    n = 4.
    wps = np.matrix(np.zeros(([3, int(n + 1)])))
    T = cfg.EFF_T_PREDEF
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
    return bezier(wps,0., T)


def buildPredefinedFinalTraj(placement, t_total):
    p_off, v_off, a_off = computePredefConstants(t_total)
    normal = placement.rotation * np.matrix([0, 0, 1]).T
    #print "normal used for landing : ",normal.T
    #print "offset used : ",p_off
    c0 = placement.translation.copy()
    c1 = placement.translation.copy()
    c0 += p_off * normal
    #print "landing part, c0 : ",c0.T
    #print "landing part, c1 : ",c1.T
    dc1 = np.matrix(np.zeros(3)).T
    #dc0 = v_off * normal
    ddc1 = np.matrix(np.zeros(3)).T
    #ddc0 = a_off * normal
    #create wp :
    n = 4.
    wps = np.matrix(np.zeros(([3, int(n + 1)])))
    T = cfg.EFF_T_PREDEF
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
    return bezier(wps,0., T)


# build a bezier curve of degree 7 that connect exactly the two given bezier up to order 3
def generatePredefMiddle(bezier_takeoff, bezier_landing, T):
    c0 = bezier_takeoff(bezier_takeoff.max())
    dc0 = bezier_takeoff.derivate(bezier_takeoff.max(), 1)
    ddc0 = bezier_takeoff.derivate(bezier_takeoff.max(), 2)
    j0 = bezier_takeoff.derivate(bezier_takeoff.max(), 3)
    c1 = bezier_landing(0)
    dc1 = bezier_landing.derivate(0, 1)
    ddc1 = bezier_landing.derivate(0, 2)
    j1 = bezier_landing.derivate(0, 3)
    n = 7
    wps = np.matrix(np.zeros(([3, int(n + 1)])))
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
    return bezier(wps,0., T)


def generatePredefBeziers(time_interval, placement_init, placement_end):
    t_total = time_interval[1] - time_interval[0] - 2 * cfg.EFF_T_DELAY
    #print "Generate Bezier Traj :"
    #print "placement Init = ",placement_init
    #print "placement End  = ",placement_end
    #print "time interval  = ",time_interval
    # generate two curves for the takeoff/landing :
    # generate a bezier curve for the middle part of the motion :
    bezier_takeoff = buildPredefinedInitTraj(placement_init, t_total)
    bezier_landing = buildPredefinedFinalTraj(placement_end, t_total)
    t_middle = (t_total - (2. * cfg.EFF_T_PREDEF))
    assert t_middle >= 0.1 and "Duration of swing phase too short for effector motion. Change the values of predef motion for effector or the duration of the contact phase. "
    bezier_middle = generatePredefMiddle(bezier_takeoff, bezier_landing, t_middle)
    curves = []
    # create polybezier with concatenation of the 3 (or 5) curves :
    # create constant curve at the beginning and end for the delay :
    if cfg.EFF_T_DELAY > 0:
        bezier_init_zero = bezier(bezier_takeoff(0), 0., cfg.EFF_T_DELAY)
        curves.append(bezier_init_zero)
    curves.append(bezier_takeoff)
    curves.append(bezier_middle)
    curves.append(bezier_landing)
    if cfg.EFF_T_DELAY > 0:
        curves.append(bezier(bezier_landing(bezier_landing.max()), 0., cfg.EFF_T_DELAY))
    pBezier = PolyBezier(curves)
    return pBezier


def generateSmoothBezierTrajWithPredef(time_interval, placement_init, placement_end):
    predef_curves = generatePredefBeziers(time_interval, placement_init, placement_end)
    bezier_takeoff = predef_curves.curves[predef_curves.idFirstNonZero()]
    bezier_landing = predef_curves.curves[predef_curves.idLastNonZero()]
    id_middle = int(math.floor(len(predef_curves.curves) / 2.))
    # update mid curve to minimize velocity along the curve:
    # set problem data for mid curve :
    pData = bezier_com.ProblemData()
    pData.c0_ = bezier_takeoff(bezier_takeoff.max())
    pData.dc0_ = bezier_takeoff.derivate(bezier_takeoff.max(), 1)
    pData.ddc0_ = bezier_takeoff.derivate(bezier_takeoff.max(), 2)
    pData.j0_ = bezier_takeoff.derivate(bezier_takeoff.max(), 3)
    pData.c1_ = bezier_landing(0)
    pData.dc1_ = bezier_landing.derivate(0, 1)
    pData.ddc1_ = bezier_landing.derivate(0, 2)
    pData.j1_ = bezier_landing.derivate(0, 3)
    pData.constraints_.flag_ = bezier_com.ConstraintFlag.INIT_POS | bezier_com.ConstraintFlag.INIT_VEL | bezier_com.ConstraintFlag.INIT_ACC | bezier_com.ConstraintFlag.END_ACC | bezier_com.ConstraintFlag.END_VEL | bezier_com.ConstraintFlag.END_POS | bezier_com.ConstraintFlag.INIT_JERK | bezier_com.ConstraintFlag.END_JERK
    t_middle = predef_curves.curves[id_middle].max()
    res = bezier_com.computeEndEffector(pData, t_middle)
    bezier_middle = res.c_of_t

    curves = predef_curves.curves[::]
    curves[id_middle] = bezier_middle
    pBezier = PolyBezier(curves)
    ref_traj = trajectories.BezierTrajectory(pBezier, placement_init, placement_end, time_interval)
    return ref_traj


"""
def generateSmoothBezierTrajWithoutPredef(time_interval,placement_init,placement_end):
    t_tot = time_interval[1]-time_interval[0]
    pData = bezier_com.ProblemData() 
    pData.c0_ = placement_init.translation.copy()
    pData.dc0_ = np.matrix(np.zeros(3)).T
    pData.ddc0_ = np.matrix(np.zeros(3)).T
    pData.j0_ = np.matrix(np.zeros(3)).T
    pData.c1_ = placement_end.translation.copy()
    pData.dc1_ = np.matrix(np.zeros(3)).T
    pData.ddc1_ = np.matrix(np.zeros(3)).T
    pData.j1_ = np.matrix(np.zeros(3)).T    
    pData.constraints_.flag_ = bezier_com.ConstraintFlag.INIT_POS | bezier_com.ConstraintFlag.INIT_VEL | bezier_com.ConstraintFlag.INIT_ACC | bezier_com.ConstraintFlag.END_ACC | bezier_com.ConstraintFlag.END_VEL | bezier_com.ConstraintFlag.END_POS | bezier_com.ConstraintFlag.INIT_JERK | bezier_com.ConstraintFlag.END_JERK    
    res = bezier_com.computeEndEffector(pData,t_tot)
    bezier_middle = res.c_of_t    
    ref_traj = trajectories.BezierTrajectory(bezier_middle,placement_init,placement_end,time_interval)    
    return ref_traj
"""


def generateSmoothBezierTrajWithoutPredef(time_interval, placement_init, placement_end):
    t_tot = time_interval[1] - time_interval[0]
    wps = np.matrix(np.zeros([3, 9]))
    for i in range(4):  # init position. init vel,acc and jerk == 0
        wps[:, i] = placement_init.translation.copy()
    # compute mid point (average and offset along z)
    wps[:, 4] = (placement_init.translation + placement_end.translation) / 2.
    wps[2, 4] += cfg.p_max
    for i in range(5, 9):  # final position. final vel,acc and jerk == 0
        wps[:, i] = placement_end.translation.copy()
    pBezier = PolyBezier(bezier(wps,0., t_tot))
    ref_traj = trajectories.BezierTrajectory(pBezier, placement_init, placement_end, time_interval)
    return ref_traj


def generateSmoothBezierTraj(time_interval,
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
        return generateSmoothBezierTrajWithPredef(time_interval, placement_init, placement_end)
    else:
        return generateSmoothBezierTrajWithoutPredef(time_interval, placement_init, placement_end)


"""

placement_init = SE3.Identity()
placement_init.translation = np.matrix([0.1,0.3,0]).T
placement_end = SE3.Identity()
placement_end.translation = np.matrix([0.6,0.22,0]).T
placement_end.rotation = Quaternion(0.9800666,0.1986693,0, 0).matrix()
t_total = 1.2
time_interval = [1,1+t_total]

"""
