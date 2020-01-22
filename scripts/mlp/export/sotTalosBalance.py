import os
import mlp.config as cfg
from pinocchio import Quaternion, SE3
from pinocchio.utils import matrixToRpy
from mlp.utils.util import SE3FromVec
## NOT GENERIC : only work for talos and only with the feet in contact
## with dt = 1ms
"""
The generators look inside a provided folder, where they expect to find 4 or 5 files:
    CoM.dat
    LeftFoot.dat
    RightFoot.dat
    WaistOrientation.dat
    optionally, ZMP.dat (if not given, it is computed from the CoM)
where the CoM and the ZMP are in the following format:
    [position3D velocity3D acceleration3D]
the feet are
    [position6D velocity6D acceleration6D]
and the waist
    [orientation3D angular_velocity3D angular_acceleration3D]
Actually, the velocity and acceleration information are only really needed for the CoM.
For all other quantities, these values are not really employed, but they are needed due
to how nd-trajectory-generator is implemented. 
What I do, is I set the arbitrarily to zero.

If you want us to be able to test the contoller with the feet admittance control too in
the near future, you may want to provide also:
    phase : zero for double support phase, positive for left support, 
    negative for right support
    rho : only useful in double support phase, a parameter defining the vertical force
    ratio between left and right foot, such that 
    rho*forceLeft + (1-rho)*forceRight = forceTotal. 
    It should range in the interval rho_min <= rho <= 1-rho_min, where rho_min > 0
"""


def exportCOM(path, c_t, dc_t, ddc_t):
    assert c_t.shape[1] == dc_t.shape[1], "c and dc should have the same size"
    assert c_t.shape[1] == ddc_t.shape[1], "c and ddc should have the same size"
    assert c_t.shape[0] == 3, "c should have 3 lines"
    assert dc_t.shape[0] == 3, "dc should have 3 lines"
    assert ddc_t.shape[0] == 3, "ddc should have 3 lines"

    filename = path + "/CoM.dat"
    with open(filename, 'w') as f:
        for k in range(c_t.shape[1]):
            line = ""
            for i in range(3):
                line += str(c_t[i, k]) + " "
            for i in range(3):
                line += str(dc_t[i, k]) + " "
            for i in range(3):
                line += str(ddc_t[i, k]) + " "
            f.write(line.rstrip(" ") + "\n")
    print("Motion exported to : ", filename)
    return


def exportZMP(path, zmp_t):
    assert zmp_t.shape[0] == 3, "zmp should be of dimension 3"
    filename = path + "/ZMP.dat"
    with open(filename, 'w') as f:
        for zmp in zmp_t.T:
            line = ""
            for i in range(2):
                line += str(zmp[0, i]) + " "
            # TODO velocity and acceleration
            for i in range(7):
                line += "0 "
            f.write(line.rstrip(" ") + "\n")
    print("Motion exported to : ", filename)
    return


def exportFoot(path, name, ref):
    assert ref.shape[0] == 12, " feet trajectory must be of size 12 (3 translation + rotation matrice)"
    filename = path + "/" + name + ".dat"
    with open(filename, 'w') as f:
        for k in range(ref.shape[1]):
            placement = SE3FromVec(ref[:, k])
            line = ""
            # 3D position :
            for i in range(3):
                line += str(placement.translation[i, 0]) + " "
            # orientation :
            rot = matrixToRpy(placement.rotation)
            #rot = matrixToRpy(SE3.Identity().rotation) # DEBUG
            for i in range(3):
                line += str(rot[i, 0]) + " "
            # TODO : velocity and acceleration :
            for i in range(12):
                line += "0 "
            f.write(line.rstrip(" ") + "\n")
    print("Motion exported to : ", filename)
    return


def exportWaist(path, waist_t):
    filename = path + "/WaistOrientation.dat"
    with open(filename, 'w') as f:
        for waist in waist_t.T:
            quat = Quaternion(waist[0, 6], waist[0, 3], waist[0, 4], waist[0, 5])
            #rot = matrixToRpy(quat.matrix()) # DEBUG
            rot = matrixToRpy(SE3.Identity().rotation)  # DEBUG
            line = ""
            for i in range(3):
                line += str(rot[i, 0]) + " "
            for i in range(6):
                line += "0 "
            f.write(line.rstrip(" ") + "\n")
    print("Motion exported to : ", filename)
    return


# phase : 0 for double support, +1 for left foot only, -1 for right foot only
def exportPhase(path, act_left, act_right):
    assert act_left.shape[1] == act_right.shape[1], "Both contact activity vector must have the same size."
    filename = path + "/Phase.dat"
    with open(filename, 'w') as f:
        for k in range(act_left.shape[1]):
            if act_left[0, k] > 0 and act_right[0, k] > 0:
                phase = 0  # cannot do r - l as it's float and may not be exactly 0 ...
            else:
                phase = act_left[0, k] - act_right[0, k]
            f.write(str(phase) + " 0 0\n")
    print("Motion exported to : ", filename)
    return


# rho : 0 for single support,
# for double support : rho*forceLeft + (1-rho)*forceRight = forceTotal.
def exportRHO(path, f_left, f_right):
    assert f_left.shape[1] == f_right.shape[1], "Both contact activity vector must have the same size."
    assert f_left.shape[0] == 1, "force normal vector must have a size of 1"
    assert f_right.shape[0] == 1, "force normal vector must have a size of 1"
    filename = path + "/Rho.dat"
    with open(filename, 'w') as f:
        for k in range(f_left.shape[1]):
            if f_left[0, k] < cfg.fMin or f_right[0, k] < cfg.fMin:
                rho = 0
            else:
                rho = (f_left[0, k]) / (f_left[0, k] + f_right[0, k])
            f.write(str(rho) + " 0 0\n")
    print("Motion exported to : ", filename)
    return


def export(res):
    if res.dt != 0.001:
        return ValueError("sotTalosBalance can only export moion generated with dt = 1ms")
    path = cfg.EXPORT_PATH + "/sotTalosBalance"
    if not os.path.exists(path):
        os.makedirs(path)
    path += "/" + cfg.DEMO_NAME
    if not os.path.exists(path):
        os.makedirs(path)

    exportCOM(path, res.c_reference, res.dc_reference, res.ddc_reference)
    exportZMP(path, res.zmp_reference)
    exportFoot(path, "LeftFoot", res.effector_references[cfg.Robot.lfoot])
    exportFoot(path, "RightFoot", res.effector_references[cfg.Robot.rfoot])
    exportWaist(path, res.q_t[0:7, :])
    exportPhase(path, res.contact_activity[cfg.Robot.lfoot], res.contact_activity[cfg.Robot.rfoot])
    exportRHO(path, res.contact_normal_force[cfg.Robot.lfoot], res.contact_normal_force[cfg.Robot.rfoot])
    return
