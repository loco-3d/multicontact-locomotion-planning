from pinocchio import SE3
import numpy as np
from numpy import cross
from numpy.linalg import norm
from pinocchio import SE3, Quaternion

def quatFromConfig(q):
    return Quaternion(q[6],q[3],q[4],q[5])
    

def se3FromConfig(q):
    placement = SE3.Identity()
    placement.translation = np.matrix(q[0:3]).T
    placement.rotation = quatFromConfig(q).matrix()
    return placement

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