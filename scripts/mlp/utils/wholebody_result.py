import numpy as np
from mlp.utils.util import discretizeCurve, discretizeDerivateCurve, discretizeSE3CurveTranslation,\
    discretizeSE3CurveToVec, discretizeSE3CurveQuaternion, constantSE3curve
from curves import piecewise_SE3
class Result:

    ### This class store discretized data points, the time step can be accessed with res.dt.
    # Each data in this struct is stored in a numpy matrix, with one discretized point per column.
    # nq is the configuration size of the robot, nv the tangent vector size,  nu the control vector size :

    def __init__(self, nq, nv, dt, eeNames, N=None, cs=None, t_begin=0, nu=None):
        self.dt = dt
        if cs:
            self.N = int(round((cs.contactPhases[-1].timeFinal - cs.contactPhases[0].timeInitial) / self.dt)) + 1
        elif N:
            self.N = N
        else:
            raise RuntimeError(
                "Result constructor must be called with either a contactSequence object or a number of points")
        N = self.N
        self.nq = nq
        self.nv = nv
        if not nu:
            nu = nv - 6
        self.nu = nu
        # timing vector (1 x N ) (time at each discretized point, from t=0 to t=dt*(N-1))
        self.t_t = np.array([t_begin + i * self.dt for i in range(N)])
        # matrix of joint configurations (nq x N )
        self.q_t = np.zeros([self.nq, N])
        # matrix of joint velocities (nv x N )
        self.dq_t = np.zeros([self.nv, N])
        # matrix of joint accelerations ( nv x N )
        self.ddq_t = np.zeros([self.nv, N])
        # matrix of joint accelerations ( v x N )
        self.tau_t = np.zeros([self.nu, N])
        # Center of Mass (CoM) positions at each time step (3 x N )
        self.c_t = np.zeros([3, N])
        # Center of Mass (CoM) velocities at each time step (3 x N )
        self.dc_t = np.zeros([3, N])
        # Center of Mass (CoM) accelerations at each time step (3 x N )
        self.ddc_t = np.zeros([3, N])
        # Angular momentum at each time step (3 x N )
        self.L_t = np.zeros([3, N])
        # Angular momentum rate at each time step (3 x N )
        self.dL_t = np.zeros([3, N])
        # Center of Mass (CoM) positions at each time step (3xN ) as computed by the CoM trajectory optimization
        self.c_reference = np.zeros([3, N])
        # Center of Mass (CoM) velocities at each time step (3 x N ) as computed by the CoM trajectory optimization
        self.dc_reference = np.zeros([3, N])
        # Center of Mass (CoM) accelerations at each time step (3 x N ) as computed by the CoM trajectory optimization
        self.ddc_reference = np.zeros([3, N])
        # Angular momentum at each time step (3 x N ) as computed by the CoM trajectory optimization
        self.L_reference = np.zeros([3, N])
        # Angular momentum rate at each time step (3 x N ) as computed by the CoM trajectory optimization
        self.dL_reference = np.zeros([3, N])
        #  Centroidal wrench at each time step (6 x N )
        self.wrench_t = np.zeros([6, N])
        # Centroidal wrench at each time step (6 x N ) as computed by the CoM trajectory optimization
        self.wrench_reference = np.zeros([6, N])
        # Zero Moment Point location at each time step(3 x N )
        self.zmp_t = np.zeros([3, N])
        # Zero Moment Point location at each time step(3 x N ) as computed by the CoM trajectory optimization
        self.zmp_reference = np.zeros([3, N])
        # Reference used (from contact planning) of the waist orientation, stored as quaternion (x,y,z,w)
        self.waist_orientation_reference = np.zeros([4, N])
        # Reference used (from contact planning) of the waist angular velocity,
        # stored as angular velocity (in world frame)
        self.d_waist_orientation_reference = np.zeros([3, N])
        # Reference used (from contact planning) of the waist angular acceleration,
        # stored as angular acceleration (in world frame)
        self.dd_waist_orientation_reference = np.zeros([3, N])
        self.eeNames = eeNames  # The list of all effector names used during the motion
        # The following variables are maps with keys = effector name and value = numpy matrices :
        # 3d forces at each contact force generator/contact point of each end-effector.
        # I.e., for a humanoid with rectangular contacts having four discrete contact points each this amounts to
        # 12 x N per end-effector. (the order and position of the generator used can be found here :
        # https://github.com/loco-3d/multicontact-locomotion-planning/blob/master/scripts/mlp/wholebody/tsid_invdyn.py#L33)
        self.contact_forces = {}
        # the contact normal force at each end-effector (1 x N per end-effector).
        self.contact_normal_force = {}
        # SE(3) trajectory of the effector (the real trajectory corresponding to the motion in q_t)
        # (12 x N per end-effector).
        # This store a column representation of an SE(3) object, see mlp.utils.util.SE3FromVec to get back the SE3 object
        self.effector_trajectories = {}
        # Velocity of the effector (the real trajectory corresponding to the motion in q_t) )
        # (6 x N per end-effector).
        # This store a column representation of a Motion object,
        # see mlp.utils.util.MotionFromVec to get back the Motion object
        self.d_effector_trajectories = {}
        # Acceleration of the effector (the real trajectory corresponding to the motion in q_t)
        # (6 x N per end-effector).
        # This store a column representation of a Motion object,
        # see mlp.utils.util.MotionFromVec to get back the Motion object
        self.dd_effector_trajectories = {}
        # Reference SE(3) trajectory of the end-effector tracked by the whole-body method
        # (12 x N per end-effector).
        # This store a column representation of an SE(3) object,
        # see mlp.utils.util.SE3FromVec to get back the SE3 object
        self.effector_references = {}
        # Reference velocity of the end-effector tracked by the whole-body method
        # (6 x N per end-effector).
        # This store a column representation of a Motion object,
        # see mlp.utils.util.MotionFromVec to get back the Motion object
        self.d_effector_references = {}
        # Reference acceleration of the end-effector tracked by the whole-body method
        # (6 x N per end-effector).
        # This store a column representation of a Motion object,
        # see mlp.utils.util.MotionFromVec to get back the Motion object
        self.dd_effector_references = {}
        # binary indicator whether a contact is active (1 x N per end-effector)
        self.contact_activity = {}
        for ee in self.eeNames:
            self.contact_forces.update({ee: np.zeros([12, N])})
            self.contact_normal_force.update({ee: np.zeros([1, N])})
            self.effector_trajectories.update({ee: np.zeros([12, N])})
            self.d_effector_trajectories.update({ee: np.zeros([6, N])})
            self.dd_effector_trajectories.update({ee: np.zeros([6, N])})
            self.effector_references.update({ee: np.zeros([12, N])})
            self.d_effector_references.update({ee: np.zeros([6, N])})
            self.dd_effector_references.update({ee: np.zeros([6, N])})
            self.contact_activity.update({ee: np.zeros([1, N])})
        if cs:
            # Correspondence between the contact phases and the indexes of discretized points in phase_intervals.
            # This field have the same length as the number of contact phases in the motion, and each element is a range of Id.
            self.phases_intervals = self.buildPhasesIntervals(cs) 
        else:
            print("Result constructor called without contactSequence object, phase_interval member not initialized")

    # By definition of a contact sequence, at the state at the transition time between two contact phases
    # belong to both contact phases
    # This mean that phases_intervals[i][-1] == phases_intervals[i+1][0]
    def buildPhasesIntervals(self, cs):
        intervals = []
        k = 0
        for phase in cs.contactPhases:
            duration = phase.duration
            n_phase = int(round(duration / self.dt))
            interval = range(k, k + n_phase + 1)
            k += n_phase
            intervals += [interval]
        return intervals

    def resizePhasesIntervals(self, N):
        new_intervals = []
        for interval in self.phases_intervals:
            if interval[-1] <= N:
                new_intervals += [interval]
            else:
                n = N - interval[0]
                reduced_interval = interval[:n]
                new_intervals += [reduced_interval]
                break
        return new_intervals

    def fillAllValues(self, k, other, k_other=None):
        if not k_other:
            k_other = k
        self.t_t[k] = other.t_t[k_other]
        self.q_t[:, k] = other.q_t[:, k_other]
        self.dq_t[:, k] = other.dq_t[:, k_other]
        self.ddq_t[:, k] = other.ddq_t[:, k_other]
        self.tau_t[:, k] = other.tau_t[:, k_other]
        self.c_t[:, k] = other.c_t[:, k_other]
        self.dc_t[:, k] = other.dc_t[:, k_other]
        self.ddc_t[:, k] = other.ddc_t[:, k_other]
        self.L_t[:, k] = other.L_t[:, k_other]
        self.dL_t[:, k] = other.dL_t[:, k_other]
        self.c_reference[:, k] = other.c_reference[:, k_other]
        self.dc_reference[:, k] = other.dc_reference[:, k_other]
        self.ddc_reference[:, k] = other.ddc_reference[:, k_other]
        self.L_reference[:, k] = other.L_t[:, k_other]
        self.dL_reference[:, k] = other.dL_t[:, k_other]
        self.wrench_t[:, k] = other.wrench_t[:, k_other]
        self.zmp_reference[:, k] = other.zmp_t[:, k_other]
        self.wrench_reference[:, k] = other.wrench_t[:, k_other]
        self.zmp_t[:, k] = other.zmp_t[:, k_other]
        self.waist_orientation_reference[:, k] = other.waist_orientation_reference[:, k_other]
        self.d_waist_orientation_reference[:, k] = other.d_waist_orientation_reference[:, k_other]
        self.dd_waist_orientation_reference[:, k] = other.dd_waist_orientation_reference[:, k_other]
        for ee in self.eeNames:
            self.contact_forces[ee][:, k] = other.contact_forces[ee][:, k_other]
            self.contact_normal_force[ee][:, k] = other.contact_normal_force[ee][:, k_other]
            self.effector_trajectories[ee][:, k] = other.effector_trajectories[ee][:, k_other]
            self.d_effector_trajectories[ee][:, k] = other.d_effector_trajectories[ee][:, k_other]
            self.dd_effector_trajectories[ee][:, k] = other.dd_effector_trajectories[ee][:, k_other]
            self.effector_references[ee][:, k] = other.effector_references[ee][:, k_other]
            self.d_effector_references[ee][:, k] = other.d_effector_references[ee][:, k_other]
            self.dd_effector_references[ee][:, k] = other.dd_effector_references[ee][:, k_other]
            self.contact_activity[ee][:, k] = other.contact_activity[ee][:, k_other]

    def resize(self, N):
        self.N = N
        self.t_t = self.t_t[:N]
        self.q_t = self.q_t[:, :N]
        self.dq_t = self.dq_t[:, :N]
        self.ddq_t = self.ddq_t[:, :N]
        self.tau_t = self.tau_t[:, :N]
        self.c_t = self.c_t[:, :N]
        self.dc_t = self.dc_t[:, :N]
        self.ddc_t = self.ddc_t[:, :N]
        self.L_t = self.L_t[:, :N]
        self.dL_t = self.dL_t[:, :N]
        self.c_reference = self.c_reference[:, :N]
        self.dc_reference = self.dc_reference[:, :N]
        self.ddc_reference = self.ddc_reference[:, :N]
        self.L_reference = self.L_t[:, :N]
        self.dL_reference = self.dL_t[:, :N]
        self.wrench_t = self.wrench_t[:, :N]
        self.zmp_t = self.zmp_t[:, :N]
        self.wrench_reference = self.wrench_reference[:, :N]
        self.zmp_reference = self.zmp_reference[:, :N]
        self.waist_orientation_reference = self.waist_orientation_reference[:, :N]
        self.d_waist_orientation_reference = self.d_waist_orientation_reference[:, :N]
        self.dd_waist_orientation_reference = self.dd_waist_orientation_reference[:, :N]
        for ee in self.eeNames:
            self.contact_forces[ee] = self.contact_forces[ee][:, :N]
            self.contact_normal_force[ee] = self.contact_normal_force[ee][:, :N]
            self.effector_trajectories[ee] = self.effector_trajectories[ee][:, :N]
            self.d_effector_trajectories[ee] = self.d_effector_trajectories[ee][:, :N]
            self.dd_effector_trajectories[ee] = self.dd_effector_trajectories[ee][:, :N]
            self.effector_references[ee] = self.effector_references[ee][:, :N]
            self.d_effector_references[ee] = self.d_effector_references[ee][:, :N]
            self.dd_effector_references[ee] = self.dd_effector_references[ee][:, :N]
            self.contact_activity[ee] = self.contact_activity[ee][:, :N]
        self.phases_intervals = self.resizePhasesIntervals(N)
        return self

    def exportNPZ(self, path, name):
        import os
        if not os.path.exists(path):
            os.makedirs(path)
        filename = path + "/" + name
        np.savez_compressed(filename,
                            N=self.N,
                            nq=self.nq,
                            nv=self.nv,
                            nu=self.nu,
                            dt=self.dt,
                            t_t=self.t_t,
                            q_t=self.q_t,
                            dq_t=self.dq_t,
                            ddq_t=self.ddq_t,
                            tau_t=self.tau_t,
                            c_t=self.c_t,
                            dc_t=self.dc_t,
                            ddc_t=self.ddc_t,
                            L_t=self.L_t,
                            dL_t=self.dL_t,
                            c_reference=self.c_reference,
                            dc_reference=self.dc_reference,
                            ddc_reference=self.ddc_reference,
                            L_reference=self.L_reference,
                            dL_reference=self.dL_reference,
                            wrench_t=self.wrench_t,
                            zmp_t=self.zmp_t,
                            wrench_reference=self.wrench_reference,
                            zmp_reference=self.zmp_reference,
                            waist_orientation_reference=self.waist_orientation_reference,
                            d_waist_orientation_reference=self.d_waist_orientation_reference,
                            dd_waist_orientation_reference=self.dd_waist_orientation_reference,
                            eeNames=self.eeNames,
                            contact_forces=self.contact_forces,
                            contact_normal_force=self.contact_normal_force,
                            effector_trajectories=self.effector_trajectories,
                            d_effector_trajectories=self.d_effector_trajectories,
                            dd_effector_trajectories=self.dd_effector_trajectories,
                            effector_references=self.effector_references,
                            d_effector_references=self.d_effector_references,
                            dd_effector_references=self.dd_effector_references,
                            contact_activity=self.contact_activity,
                            phases_intervals=self.phases_intervals)

        print("Results exported to ", filename)

    def qAtT(self, t):
        k = int(round(t / self.dt))
        assert self.t_t[k] == t and "Error in computation of time in Result struct."
        return self.q_t[:, k]



def loadFromNPZ(filename):
    f = np.load(filename)
    N = f['N'].tolist()
    nq = f['nq'].tolist()
    nv = f['nv'].tolist()
    nu = f['nu'].tolist()
    dt = f['dt'].tolist()
    eeNames = f['eeNames'].tolist()
    res = Result(nq, nv, dt, eeNames=eeNames, N=N, nu=nu)
    res.t_t = f['t_t']
    res.q_t = f['q_t']
    res.dq_t = f['dq_t']
    res.ddq_t = f['ddq_t']
    res.tau_t = f['tau_t']
    res.c_t = f['c_t']
    res.dc_t = f['dc_t']
    res.ddc_t = f['ddc_t']
    res.L_t = f['L_t']
    res.dL_t = f['dL_t']
    res.c_reference = f['c_reference']
    res.dc_reference = f['dc_reference']
    res.ddc_reference = f['ddc_reference']
    res.L_reference = f['L_t']
    res.dL_reference = f['dL_t']
    res.wrench_t = f['wrench_t']
    res.zmp_reference = f['zmp_t']
    res.wrench_reference = f['wrench_t']
    res.zmp_t = f['zmp_t']
    res.waist_orientation_reference = f['waist_orientation_reference']
    res.d_waist_orientation_reference = f['d_waist_orientation_reference']
    res.dd_waist_orientation_reference = f['dd_waist_orientation_reference']
    res.contact_forces = f['contact_forces'].tolist()
    res.contact_normal_force = f['contact_normal_force'].tolist()
    res.effector_trajectories = f['effector_trajectories'].tolist()
    res.d_effector_trajectories = f['d_effector_trajectories'].tolist()
    res.dd_effector_trajectories = f['dd_effector_trajectories'].tolist()
    res.effector_references = f['effector_references'].tolist()
    res.d_effector_references = f['d_effector_references'].tolist()
    res.dd_effector_references = f['dd_effector_references'].tolist()
    res.contact_activity = f['contact_activity'].tolist()
    res.phases_intervals = f['phases_intervals'].tolist()
    f.close()
    return res


def FromContactSequenceWB(cs_ref, cs, dt):
    p0 = cs.contactPhases[0]
    nq = p0.q_t.dim()
    nv = p0.dq_t.dim()
    nu = p0.tau_t.dim()
    eeNames = cs.getAllEffectorsInContact()
    t_begin = p0.timeInitial
    res = Result(nq, nv, dt, eeNames, cs=cs, t_begin=t_begin, nu=nu)
    # fill all the array with discretization of the curves
    res.q_t = discretizeCurve(cs.concatenateQtrajectories(), dt)[0]
    res.dq_t = discretizeCurve(cs.concatenateDQtrajectories(), dt)[0]
    res.ddq_t = discretizeCurve(cs.concatenateDDQtrajectories(), dt)[0]
    res.tau_t = discretizeCurve(cs.concatenateTauTrajectories(), dt)[0]
    res.c_t = discretizeCurve(cs.concatenateCtrajectories(), dt)[0]
    res.dc_t = discretizeCurve(cs.concatenateDCtrajectories(), dt)[0]
    res.ddc_t = discretizeCurve(cs.concatenateDDCtrajectories(), dt)[0]
    res.L_t = discretizeCurve(cs.concatenateLtrajectories(), dt)[0]
    res.dL_t = discretizeCurve(cs.concatenateDLtrajectories(), dt)[0]
    res.c_reference = discretizeCurve(cs_ref.concatenateCtrajectories(), dt)[0]
    res.dc_reference = discretizeCurve(cs_ref.concatenateDCtrajectories(), dt)[0]
    res.ddc_reference = discretizeCurve(cs_ref.concatenateDDCtrajectories(), dt)[0]
    res.L_reference = discretizeCurve(cs_ref.concatenateLtrajectories(), dt)[0]
    res.dL_reference = discretizeCurve(cs_ref.concatenateDLtrajectories(), dt)[0]
    res.zmp_t = discretizeCurve(cs.concatenateZMPtrajectories(), dt)[0]
    res.wrench_t = discretizeCurve(cs.concatenateWrenchTrajectories(), dt)[0]
    res.zmp_reference = discretizeCurve(cs_ref.concatenateZMPtrajectories(), dt)[0]
    res.wrench_reference = discretizeCurve(cs_ref.concatenateWrenchTrajectories(), dt)[0]
    root_t = cs_ref.concatenateRootTrajectories()
    res.waist_orientation_reference = discretizeSE3CurveQuaternion(root_t, dt)[0]
    res.d_waist_orientation_reference = discretizeDerivateCurve(root_t, dt, 1)[0][3:, :]
    res.dd_waist_orientation_reference = discretizeDerivateCurve(root_t, dt, 2)[0][3:, :]
    for ee in eeNames:
        res.contact_forces[ee] = discretizeCurve(cs.concatenateContactForceTrajectories(ee), dt)[0]
        res.contact_normal_force[ee] = discretizeCurve(cs.concatenateNormalForceTrajectories(ee), dt)[0]
        eff = cs.concatenateEffectorTrajectories(ee)
        # append init/end constant trajectorie if required :
        if eff.min() < t_begin:
            eff_begin = constantSE3curve(eff.evaluateAsSE3(eff.min()), t_begin, eff.min())
            eff_mid = eff
            eff = piecewise_SE3(eff_begin)
            eff.append(eff_mid)
        if eff.max() < cs.contactPhases[-1].timeFinal:
            eff_end= constantSE3curve(eff.evaluateAsSE3(eff.max()), eff.max(), cs.contactPhases[-1].timeFinal)
            eff.append(eff_end)
        res.effector_trajectories[ee] = discretizeSE3CurveToVec(eff, dt)[0]
        res.d_effector_trajectories[ee] = discretizeDerivateCurve(eff, dt, 1)[0]
        res.dd_effector_trajectories[ee] = discretizeDerivateCurve(eff, dt, 2)[0]
        eff = cs_ref.concatenateEffectorTrajectories(ee)
        # append init/end constant trajectorie if required :
        if eff.min() < t_begin:
            eff_begin = constantSE3curve(eff.evaluateAsSE3(eff.min()), t_begin, eff.min())
            eff_mid = eff
            eff = piecewise_SE3(eff_begin)
            eff.append(eff_mid)
        if eff.max() < cs.contactPhases[-1].timeFinal:
            eff_end = constantSE3curve(eff.evaluateAsSE3(eff.max()), eff.max(), cs.contactPhases[-1].timeFinal)
            eff.append(eff_end)
        res.effector_references[ee] = discretizeSE3CurveToVec(eff, dt)[0]
        res.d_effector_references[ee] = discretizeDerivateCurve(eff, dt, 1)[0]
        res.dd_effector_references[ee] = discretizeDerivateCurve(eff, dt, 2)[0]


    return res