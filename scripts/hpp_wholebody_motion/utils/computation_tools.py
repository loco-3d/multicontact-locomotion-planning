
import numpy as np
import pinocchio as se3
from pinocchio import SE3, Motion,Force
from hpp_wholebody_motion.utils.util import *
import hpp_wholebody_motion.config as cfg
import hpp_wholebody_motion.utils.trajectories as Trajectories
### tools to compute zmp trajectory : ### 


def pacthSameAltitude(M1,M2,eps=1e-3):
    return abs(M1.translation[2] == M2.translation[2]) <= eps

# compute an approximation of the floor altitude (used for the zmp reference)
#Only consider the feet : 
# - if only one feet in contact, return it's z coordinate
# - if both feet in contact, make a linear interp between both feet altitude wrt to which feet will move next
def computeFloorAltitude(cs,t):
    id_phase = findPhase(cs,t)
    phase = cs.contact_phases[id_phase]
    RF_patch = phase.RF_patch
    LF_patch = phase.LF_patch
    Mrf = JointPlacementForEffector(phase,cfg.Robot.rfoot)
    Mlf = JointPlacementForEffector(phase,cfg.Robot.lfoot)

    if RF_patch.active and LF_patch.active:
        if pacthSameAltitude(Mrf,Mlf):
            floor_altitude = 0.5*(Mrf.translation+Mlf.translation)[2]
        else:
            # look for wich feet just created contact :
            LF_moved = False
            if id_phase > 0: #look for the inactive contact in the previous phase
                pprev = cs.contact_phases[id_phase-1]
                if not pprev.RF_patch.active: 
                    LF_moved = False
                elif not pprev.LF_patch.active:
                    LF_moved = True
                else:
                    assert "Must never happened"                                    
            else: #look for the inactive contact in the next phase and assume cyclic gait for the last couple of phases
                pnext = cs.contact_phases[id_phase+1]
                if not pnext.RF_patch.active:
                    LF_moved = True
                elif not pnext.LF_patch.active :
                    LF_moved = False
                else:
                    assert "Must never happened"                
            # linear interp of the altitude (from the feet which are going to move to the one that stay)
            if LF_moved:
                p0 = Mrf.translation[2]
                p1 = Mlf.translation[2]
            else:
                p0 = Mlf.translation[2]
                p1 = Mrf.translation[2]

            t0 = phase.time_trajectory[0]
            t1 = phase.time_trajectory[-1]
            assert t0 < t1
            s = (t - t0) / (t1-t0)
            floor_altitude = p0 + s * (p1-p0)
            pass
    elif RF_patch.active:
        floor_altitude = Mrf.translation[2]
    elif LF_patch.active:
        floor_altitude = Mlf.translation[2]
    else:
        assert "Must never happened"
    return floor_altitude

def shiftZMPtoFloorAltitude(cs,t,phi0):
    Mshift = SE3.Identity()
    shift = Mshift.translation  
    floor_altitude = computeFloorAltitude(cs,t) 
    shift[2] = floor_altitude
    Mshift.translation = shift
    
    # apply transform to wrench : 
    phi_floor = Mshift.actInv(phi0)
    #compute zmp with transformed phi :
    w = phi_floor.angular
    f = phi_floor.linear
    #print "Zx",-w[1]/f[2]
    #print "Zy",w[0]/f[2]
    #print floor_altitude
    ZMP = np.matrix([float(-w[1]/f[2]),float(w[0]/f[2]),float(floor_altitude)]).T    
    return ZMP    
    
# not generic ! only consider feet 
# (not a problem as we only call it for openHRP)
def computeRefZMP(cs,time_t,wrench_t):
    
    N = len(time_t)
    ZMP_t = np.matrix(np.empty((3,N)))
    Mshift = SE3.Identity()
    shift = Mshift.translation
    
    # smooth wrench traj : 
    Wrench_trajectory = Trajectories.DifferentiableEuclidianTrajectory()
    Wrench_trajectory.computeFromPoints(np.asmatrix(time_t),wrench_t,0*wrench_t)
    

    for k,t in enumerate(time_t):
        wrench = Wrench_trajectory(t)[0]
        phi0 = Force(wrench)
        ZMP_t[:,k] = ZMP = shiftZMPtoFloorAltitude(cs,t,phi0)

    return ZMP_t

