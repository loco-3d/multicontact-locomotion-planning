import numpy as np
import pinocchio as pin
from pinocchio import SE3, Motion, Force
from curves import piecewise, polynomial
from rospkg import RosPack
from pinocchio.robot_wrapper import RobotWrapper
from curves import piecewise

def pacthSameAltitude(M1, M2, eps=1e-3):
    return abs(M1.translation[2] == M2.translation[2]) <= eps


# compute an approximation of the floor altitude (used for the zmp reference)
#Only consider the feet :
# - if only one feet in contact, return it's z coordinate
# - if both feet in contact, make a linear interp between both feet altitude wrt to which feet will move next
def computeFloorAltitude(cs, t, Robot):
    id_phase = cs.phaseIdAtTime(t)
    phase = cs.contactPhases[id_phase]

    if phase.isEffectorInContact(Robot.rfoot) and phase.isEffectorInContact(Robot.lfoot):
        Mrf = phase.contactPatch(Robot.rfoot).placement
        Mlf = phase.contactPatch(Robot.lfoot).placement
        if pacthSameAltitude(Mrf, Mlf):
            floor_altitude = 0.5 * (Mrf.translation + Mlf.translation)[2]
        else:
            # look for wich feet just created contact :
            LF_moved = False
            if id_phase > 0:  #look for the inactive contact in the previous phase
                pprev = cs.contactPhases[id_phase - 1]
                if not pprev.isEffectorInContact(Robot.rfoot):
                    LF_moved = False
                elif not pprev.isEffectorInContact(Robot.lfoot):
                    LF_moved = True
                else:
                    assert "Must never happened"
            elif id_phase < cs.size()-1:  #look for the inactive contact in the next phase and assume cyclic gait for the last couple of phases
                pnext = cs.contactPhases[id_phase + 1]
                if not pnext.isEffectorInContact(Robot.rfoot):
                    LF_moved = True
                elif not pnext.isEffectorInContact(Robot.lfoot):
                    LF_moved = False
                else:
                    assert "Must never happened"
            else:
                LF_moved = False
            # linear interp of the altitude (from the feet which are going to move to the one that stay)
            if LF_moved:
                p0 = Mrf.translation[2]
                p1 = Mlf.translation[2]
            else:
                p0 = Mlf.translation[2]
                p1 = Mrf.translation[2]

            t0 = phase.timeInitial
            t1 = phase.timeFinal
            assert t0 < t1
            s = (t - t0) / (t1 - t0)
            floor_altitude = p0 + s * (p1 - p0)
            pass
    elif phase.isEffectorInContact(Robot.rfoot):
        Mrf = phase.contactPatch(Robot.rfoot).placement
        floor_altitude = Mrf.translation[2]

    elif phase.isEffectorInContact(Robot.lfoot):
        Mlf = phase.contactPatch(Robot.lfoot).placement
        floor_altitude = Mlf.translation[2]
    else:
        assert "Must never happened"
    return floor_altitude


def shiftZMPtoFloorAltitude(cs, t, phi0, Robot):
    Mshift = SE3.Identity()
    shift = Mshift.translation
    floor_altitude = computeFloorAltitude(cs, t, Robot)
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

    ZMP = np.array([float(-w[1] / f[2]), float(w[0] / f[2]), float(floor_altitude)])
    return ZMP


# not generic ! only consider feet
# (not a problem as we only call it for openHRP)
def computeZMPFromWrench(cs, Robot, dt):

    for phase in cs.contactPhases:
        t = phase.timeInitial
        phase.zmp_t = None
        while t <= phase.timeFinal:
            phi0 = Force(phase.wrench_t(t))
            zmp = shiftZMPtoFloorAltitude(cs, t, phi0, Robot)
            if phase.zmp_t is None:
                phase.zmp_t = piecewise(polynomial(zmp.reshape(-1, 1), t, t))
            else:
                phase.zmp_t.append(zmp, t)
            t += dt
            if phase.timeFinal - dt/2. <= t <= phase.timeFinal + dt/2.:
                t = phase.timeFinal


def computeWrench(cs, Robot, dt):
    rp = RosPack()
    urdf = rp.get_path(Robot.packageName) + '/urdf/' + Robot.urdfName + Robot.urdfSuffix + '.urdf'
    #srdf = "package://" + package + '/srdf/' +  cfg.Robot.urdfName+cfg.Robot.srdfSuffix + '.srdf'
    robot = RobotWrapper.BuildFromURDF(urdf, pin.StdVec_StdString(), pin.JointModelFreeFlyer(), False)
    model = robot.model
    data = robot.data
    q_t = cs.concatenateQtrajectories()
    dq_t = cs.concatenateQtrajectories()
    ddq_t = cs.concatenateQtrajectories()
    t = q_t.min()
    for phase in cs.contactPhases:
        phase.wrench_t = None
        t = phase.timeInitial
        while t <= phase.timeFinal:
            pin.rnea(model, data, phase.q_t(t), phase.dq_t(t), phase.ddq_t(t))
            pcom, vcom, acom = robot.com( phase.q_t(t), phase.dq_t(t), phase.ddq_t(t))
            # FIXME : why do I need to call com to have the correct values in data ??
            phi0 = data.oMi[1].act(pin.Force(data.tau[:6]))
            if phase.wrench_t is None:
                phase.wrench_t = piecewise(polynomial(phi0.vector.reshape(-1,1), t, t))
            else:
                phase.wrench_t.append(phi0.vector, t)
            t += dt
            if phase.timeFinal - dt/2. <= t <= phase.timeFinal + dt/2.:
                t = phase.timeFinal


def computeZMP(cs, cfg):
    computeWrench(cs, cfg.Robot, cfg.IK_dt)
    computeZMPFromWrench(cs, cfg.Robot, cfg.IK_dt)


# compute wrench F0 from centroidal data
def computeWrenchRef(cs, mass, G, dt):
    Mcom = SE3.Identity()
    for phase in cs.contactPhases:
        t = phase.timeInitial
        phase.wrench_t = None
        while t <= phase.timeFinal:
            Mcom.translation = phase.c_t(t)
            Fcom = Force.Zero()
            Fcom.linear = mass * (phase.ddc_t(t) - G)
            Fcom.angular = phase.dL_t(t)
            F0 = Mcom.act(Fcom)
            if phase.wrench_t is None:
                phase.wrench_t = piecewise(polynomial(F0.vector.reshape(-1,1), t, t))
            else:
                phase.wrench_t.append(F0.vector, t)
            t += dt
            if phase.timeFinal - dt/2. <= t <= phase.timeFinal + dt/2.:
                t = phase.timeFinal


def computeZMPRef(cs, cfg):
    computeWrenchRef(cs, cfg.MASS, cfg.GRAVITY, cfg.SOLVER_DT)
    computeZMPFromWrench(cs, cfg.Robot, cfg.SOLVER_DT)
