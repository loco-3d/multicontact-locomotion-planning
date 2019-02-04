import hpp_wholebody_motion.config as cfg
import time
import os
from hpp_wholebody_motion.utils.polyBezier import *
import pinocchio as se3
from pinocchio import SE3
from pinocchio.utils import *
import numpy.linalg
from locomote import WrenchCone,SOC6,ContactSequenceHumanoid
import numpy as np
from tools.disp_bezier import *
import hpp_spline
import hpp_bezier_com_traj as bezier_com



class Empty:
    None
    
def stdVecToMatrix(std_vector):
    vec_l = []
    for vec in std_vector:
        vec_l.append(vec)

    res = np.hstack(tuple(vec_l))
    return res

def computeConstantsWithDDJerk(ddjerk,t):
    a = (1./6.)*ddjerk *t*t*t
    v = (1./24.) * ddjerk *t*t*t*t
    p = (1./120.) * ddjerk *t*t*t*t*t   
    return p,v,a

def computePosOffset(t_predef,t_total):
    timeMid= (t_total - (2.*t_predef))/2.
    p = cfg.p_max / (1. + 4.*timeMid/t_predef + 6.*timeMid*timeMid/(t_predef*t_predef) - (timeMid*timeMid*timeMid)/(t_predef*t_predef*t_predef))
    return p,0.,0.

def computePredefConstants(t):
    #return computeConstantsWithDDJerk(250.,cfg.EFF_T_PREDEF)
    return computePosOffset(cfg.EFF_T_PREDEF,t)

def buildPredefinedInitTraj(placement,t_total):
    p_off,v_off,a_off = computePredefConstants(t_total)
    normal = placement.rotation * np.matrix([0,0,1]).T
    c0 = placement.translation.copy()
    c1 = placement.translation.copy()
    c1 += p_off * normal
    dc0 = np.matrix(np.zeros(3)).T
    #dc1 = v_off * normal
    ddc0 = np.matrix(np.zeros(3)).T
    #ddc1 = a_off * normal
    #create wp : 
    n = 4.
    wps = np.matrix(np.zeros((3,int(n+1))))
    T = cfg.EFF_T_PREDEF
    # constrained init pos and final pos. Init vel, acc and jerk = 0
    wps[:,0] = (c0); # c0
    wps[:,1] =((dc0 * T / n )+  c0); #dc0
    wps[:,2] =((n*n*c0 - n*c0 + 2.*n*dc0*T - 2.*dc0*T + ddc0*T*T)/(n*(n - 1.)));#ddc0 // * T because derivation make a T appear
    wps[:,3] =((n*n*c0 - n*c0 + 3.*n*dc0*T - 3.*dc0*T + 3.*ddc0*T*T)/(n*(n - 1.))); #j0 = 0 
    wps[:,4] =(c1); #c1 
    return bezier(wps,T)

def buildPredefinedFinalTraj(placement,t_total):
    p_off,v_off,a_off = computePredefConstants(t_total)
    normal = placement.rotation * np.matrix([0,0,1]).T
    c0 = placement.translation.copy()
    c1 = placement.translation.copy()
    c0 += p_off * normal
    dc1 = np.matrix(np.zeros(3)).T
    #dc0 = v_off * normal
    ddc1 = np.matrix(np.zeros(3)).T
    #ddc0 = a_off * normal
    #create wp : 
    n = 4.
    wps = np.matrix(np.zeros((3,int(n+1))))
    T = cfg.EFF_T_PREDEF
    # constrained init pos and final pos. Init vel, acc and jerk = 0
    wps[:,0] = (c0); #c0
    wps[:,1] = ((n*n*c1 - n*c1 - 3*n*dc1*T + 3*dc1*T + 3*ddc1*T*T)/(n*(n - 1))) ; # j1
    wps[:,2] = ((n*n*c1 - n*c1 - 2*n*dc1*T + 2*dc1*T + ddc1*T*T)/(n*(n - 1))) ; #ddc1 * T ??
    wps[:,3] = ((-dc1 * T / n) + c1); #dc1
    wps[:,4] = (c1); #c1
    return bezier(wps,T)

def generateBezierTraj(placement_init,placement_final,t_total):
    # generate two curves for the takeoff/landing : 
    # generate a bezier curve for the middle part of the motion : 
    bezier_takeoff = buildPredefinedInitTraj(placement_init,t_total)
    bezier_landing = buildPredefinedFinalTraj(placement_final,t_total)
    # set problem datz for mid curve : 
    pData = bezier_com.ProblemData() 
    pData.c0_ = bezier_takeoff(bezier_takeoff.max())
    pData.dc0_ = bezier_takeoff.derivate(bezier_takeoff.max(),1)
    pData.ddc0_ = bezier_takeoff.derivate(bezier_takeoff.max(),2)
    pData.j0_ = bezier_takeoff.derivate(bezier_takeoff.max(),3)
    pData.c1_ = bezier_landing(0)
    pData.dc1_ = bezier_landing.derivate(0,1)
    pData.ddc1_ = bezier_landing.derivate(0,2)
    pData.j1_ = bezier_landing.derivate(0,3)    
    pData.constraints_.flag_ = bezier_com.ConstraintFlag.INIT_POS | bezier_com.ConstraintFlag.INIT_VEL | bezier_com.ConstraintFlag.INIT_ACC | bezier_com.ConstraintFlag.END_ACC | bezier_com.ConstraintFlag.END_VEL | bezier_com.ConstraintFlag.END_POS | bezier_com.ConstraintFlag.INIT_JERK | bezier_com.ConstraintFlag.END_JERK
    t_middle =  (t_total - (2.*cfg.EFF_T_PREDEF))
    res = bezier_com.computeEndEffector(pData,t_middle)
    bezier_middle = res.c_of_t
    # create polybezier with concatenation of the 3 curves :
    curves = []
    curves.append(bezier_takeoff)
    curves.append(bezier_middle)
    curves.append(bezier_landing)
    pBezier = PolyBezier(curves)
    
    return pBezier


"""

placement_init = SE3.Identity()
placement_init.translation = np.matrix([0.1,0.3,0]).T
placement_end = SE3.Identity()
placement_end.translation = np.matrix([0.6,0.22,0]).T
placement_end.rotation = Quaternion(0.9800666,0.1986693,0, 0).matrix()
t_total = 1.2

"""