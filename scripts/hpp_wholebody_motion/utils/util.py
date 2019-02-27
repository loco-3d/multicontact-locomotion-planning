from pinocchio import SE3
import numpy as np
from pinocchio import SE3, Quaternion

def quatFromConfig(q):
    return Quaternion(q[6],q[3],q[4],q[5])
    

def se3FromConfig(q):
    placement = SE3.Identity()
    placement.translation = np.matrix(q[0:3]).T
    placement.rotation = quatFromConfig(q).matrix()
    return placement