from pinocchio import SE3
import numpy as np

def toRotationMatrix(q):
    """
    Returns a (3*3) array (rotation matrix)
    representing the same rotation as the (normalized) quaternion.
    """
    rm=np.zeros((3,3))
    rm[0,0]=1-2*(q[2]**2+q[3]**2)
    rm[0,1]=2*q[1]*q[2]-2*q[0]*q[3]
    rm[0,2]=2*q[1]*q[3]+2*q[0]*q[2]
    rm[1,0]=2*q[1]*q[2]+2*q[0]*q[3]
    rm[1,1]=1-2*(q[1]**2+q[3]**2)
    rm[1,2]=2*q[2]*q[3]-2*q[0]*q[1]
    rm[2,0]=2*q[1]*q[3]-2*q[0]*q[2]
    rm[2,1]=2*q[2]*q[3]+2*q[0]*q[1]
    rm[2,2]=1-2*(q[1]**2+q[2]**2)
    return rm

def se3FromConfig(q):
    placement = SE3.Identity()
    placement.translation = np.matrix(q[0:3]).T
    placement.rotation = toRotationMatrix(q[3:7])
    return placement