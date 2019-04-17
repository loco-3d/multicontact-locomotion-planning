from pinocchio import SE3
import numpy as np
from numpy import cross
from numpy.linalg import norm
from pinocchio import SE3, Quaternion
import mlp.config as cfg

def quatFromConfig(q):
    return Quaternion(q[6],q[3],q[4],q[5])
    

def distPointLine(p_l,x1_l,x2_l):
    p= np.matrix(p_l)
    x1= np.matrix(x1_l)
    x2= np.matrix(x2_l)
    return norm(cross(p-x1,p-x2)) / norm(x2-x1)
    
    

def findPhase(cs,t):
    phase0 = cs.contact_phases[0]
    phasel = cs.contact_phases[-1]
    if t <= phase0.time_trajectory[0]:
        return 0
    elif t >= phasel.time_trajectory[-1]:
        return len(cs.contact_phases)-1


    id = [k for k, phase in enumerate(cs.contact_phases) if t >= phase.time_trajectory[0] and t <= phase.time_trajectory[-1]]
    assert len(id)>=1 or len(id) <= 2
    if len(id) == 2:
        return id[1]
    else:
        return id[0]
    

def SE3toVec(M):
    v = np.matrix(np.zeros((12, 1)))
    for j in range(3):
        v[j] = M.translation[j]
        v[j + 3] = M.rotation[j, 0]
        v[j + 6] = M.rotation[j, 1]
        v[j + 9] = M.rotation[j, 2]
    return v

# TODO : def SE3FromVec(s):

def MotiontoVec(M):
    v = np.matrix(np.zeros((6, 1)))
    for j in range(3):
        v[j] = M.linear[j]
        v[j + 3] = M.angular[j]
    return v


def stdVecToMatrix(std_vector):
    if len(std_vector) == 0:
        raise Exception("std_vector is Empty")
    vec_l = []
    for vec in std_vector:
        vec_l.append(vec)

    res = np.hstack(tuple(vec_l))
    return res

def numpy2DToList(m):
    l = []
    for i in range(m.shape[1]):
        l += [m[:,i].T.tolist()[0]]
    return l    

# assume that q.size >= 7 with root pos and quaternion(x,y,z,w)
def SE3FromConfig(q):
    placement = SE3.Identity()
    placement.translation = q[0:3]
    r = Quaternion(q[6,0],q[3,0],q[4,0],q[5,0])
    placement.rotation = r.matrix()
    return placement

# cfg.Robot.MRsole_offset.actInv(p0.RF_patch.placement)
# get the joint position for the given phase with the given effector name
# Note that if the effector is not in contact the phase placement may be uninitialized (==Identity)
def JointPatchForEffector(phase,eeName):
    if eeName == cfg.Robot.rfoot :
        patch = phase.RF_patch.copy()
        patch.placement = cfg.Robot.MRsole_offset.actInv(patch.placement)
    elif eeName == cfg.Robot.lfoot :
        patch = phase.LF_patch.copy()
        patch.placement = cfg.Robot.MLsole_offset.actInv(patch.placement)
    elif eeName == cfg.Robot.rhand :
        patch = phase.RH_patch.copy()
        patch.placement = cfg.Robot.MRhand_offset.actInv(patch.placement)
    elif eeName == cfg.Robot.lhand :
        patch = phase.LH_patch.copy()
        patch.placement = cfg.Robot.MLhand_offset.actInv(patch.placement)   
    else :
        raise Exception("Unknown effector name")
    return patch

def JointPlacementForEffector(phase,eeName):
    return JointPatchForEffector(phase,eeName).placement

def getContactPlacement(phase,eeName):
    if eeName == cfg.Robot.rfoot :
        return phase.RF_patch.placement
    elif eeName == cfg.Robot.lfoot :
        return phase.LF_patch.placement
    elif eeName == cfg.Robot.rhand :
        return phase.RH_patch.placement
    elif eeName == cfg.Robot.lhand :
        return phase.LH_patch.placement
    else :
        raise Exception("Unknown effector name")
    return patch


def isContactActive(phase,eeName):
    if eeName == cfg.Robot.rfoot :
        return phase.RF_patch.active
    elif eeName == cfg.Robot.lfoot :
        return phase.LF_patch.active
    elif eeName == cfg.Robot.rhand :
        return phase.RH_patch.active
    elif eeName == cfg.Robot.lhand :
        return phase.LH_patch.active
    else :
        raise Exception("Unknown effector name") 

def isContactEverActive(cs,eeName):
    for phase in cs.contact_phases:
        if eeName == cfg.Robot.rfoot :
            if phase.RF_patch.active:
                return True
        elif eeName == cfg.Robot.lfoot :
            if phase.LF_patch.active:
                return True
        elif eeName == cfg.Robot.rhand :
            if phase.RH_patch.active:
                return True
        elif eeName == cfg.Robot.lhand :
            if phase.LH_patch.active:
                return True
        else :
            raise Exception("Unknown effector name") 
    return False
    