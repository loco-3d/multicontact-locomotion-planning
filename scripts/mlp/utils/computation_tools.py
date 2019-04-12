
import numpy as np
import pinocchio as se3
from pinocchio import SE3, Motion,Force
from mlp.utils.util import *
import mlp.config as cfg
import mlp.utils.trajectories as Trajectories
from rospkg import RosPack
from pinocchio.robot_wrapper import RobotWrapper




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
    #print "phi0",phi0
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
def computeZMPFromWrench(cs,time_t,wrench_t):
    
    N = len(time_t)
    ZMP_t = np.matrix(np.empty((3,N)))
    
    # smooth wrench traj : 
    Wrench_trajectory = Trajectories.DifferentiableEuclidianTrajectory()
    Wrench_trajectory.computeFromPoints(np.asmatrix(time_t),wrench_t,0*wrench_t)
    

    for k in range(N):
        wrench = Wrench_trajectory(time_t[k])[0]
        phi0 = Force(wrench)
        ZMP_t[:,k] = shiftZMPtoFloorAltitude(cs,time_t[k],phi0)

    return ZMP_t

def computeWrench(res):
    rp = RosPack()
    urdf = rp.get_path(cfg.Robot.packageName)+'/urdf/'+cfg.Robot.urdfName+cfg.Robot.urdfSuffix+'.urdf'   
    #srdf = "package://" + package + '/srdf/' +  cfg.Robot.urdfName+cfg.Robot.srdfSuffix + '.srdf'
    robot = RobotWrapper(urdf, se3.StdVec_StdString(), se3.JointModelFreeFlyer(), False)
    model = robot.model
    data = robot.data    
    for k in range(res.N):
        se3.rnea(model,data,res.q_t[:,k],res.dq_t[:,k],res.ddq_t[:,k])
        pcom, vcom, acom = robot.com(res.q_t[:,k],res.dq_t[:,k],res.ddq_t[:,k]) # FIXME : why do I need to call com to have the correct values in data ??      
        phi0 = data.oMi[1].act(se3.Force(data.tau[:6]))
        res.wrench_t[:,k] = phi0.vector
    return res
    

def computeZMP(cs,res):
    res = computeWrench(res)
    res.zmp_t = computeZMPFromWrench(cs,res.t_t,res.wrench_t)
    return res

# compute wrench F0 from centroidal data
def computeWrenchRef(res):
    Mcom = SE3.Identity()
    for k,t in enumerate(res.t_t):
        Mcom.translation = res.c_reference[:,k]
        Fcom = Force.Zero()
        Fcom.linear = cfg.MASS*(res.ddc_reference[:,k] - cfg.GRAVITY)
        Fcom.angular = res.dL_reference[:,k]
        F0 = Mcom.act(Fcom)
        res.wrench_reference[:,k] = F0.vector
    return res

def computeZMPRef(cs,res):
    res = computeWrenchRef(res)
    res.zmp_reference = computeZMPFromWrench(cs,res.t_t,res.wrench_reference)
    return res    