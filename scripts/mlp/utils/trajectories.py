import numpy as np
import numpy.matlib
from numpy.polynomial.polynomial import polyval
#from  numpy import polyder
from numpy.linalg import pinv
import pinocchio as pin
from pinocchio import SE3, log3, exp3, Motion, Quaternion
from curves import SO3Linear
from pinocchio.utils import zero as mat_zeros


# COM Trajecotry smoothing using cubic spline
def polyder(coeffs):
    return np.polyder(coeffs[::-1])[::-1]


class SmoothedFootTrajectory(object):
    def __init__(self, time_intervals, foot_placements, z_amplitude=0.05, name="Foot ref trajectory"):
        self.name = name
        self.time_intervals = time_intervals
        self.foot_placements = foot_placements
        self.z_amplitude = z_amplitude
        self._R = np.identity(3)
        self.__compute()
        self.t0_l = []
        for k in range(len(time_intervals)):
            self.t0_l.append(time_intervals[k])

    def setOrientation(self, R):
        self._R = R

    def __compute(self):
        self.polycoeff_l = []
        self.dpolycoeff_l = []
        self.ddpolycoeff_l = []
        num_intervals = len(self.time_intervals)

        for k in range(num_intervals):
            xyz_polycoeff = []
            dxyz_polycoeff = []
            ddxyz_polycoeff = []

            foot_init_position = self.foot_placements[0].translation
            foot_end_position = self.foot_placements[1].translation

            # X trajectory
            x0 = foot_init_position[0, 0]
            x1 = foot_end_position[0, 0]

            nx = 6
            Px = np.zeros([nx, nx])
            Px[0, 0] = 1.
            Px[1, :] += 1.
            Px[2, 1] = 1.
            Px[3, 1:] = range(1, nx)
            Px[4, 2] = 1.
            Px[5, 2:] = np.array(range(2, nx)) * np.array(range(1, nx - 1))

            bx = np.array([x0, x1, 0., 0., 0., 0.])
            x_coeff = pinv(Px).dot(bx)
            xyz_polycoeff.append(x_coeff)

            dx_coeff = polyder(x_coeff)
            dxyz_polycoeff.append(dx_coeff)

            ddx_coeff = polyder(dx_coeff)
            ddxyz_polycoeff.append(ddx_coeff)

            # Y trajectory
            y0 = foot_init_position[1, 0]
            y1 = foot_end_position[1, 0]

            if y0 == y1:
                xyz_polycoeff.append(y0)
                dxyz_polycoeff.append([])
                ddxyz_polycoeff.append([])
            else:
                ny = 6
                Py = np.zeros([ny, ny])
                Py[0, 0] = 1.
                Py[1, :] += 1.
                Py[2, 1] = 1.
                Py[3, 1:] = range(1, ny)
                Py[4, 2] = 1.
                Py[5, 2:] = np.array(range(2, ny)) * np.array(range(1, ny - 1))

                by = np.array([y0, y1, 0., 0., 0., 0.])
                y_coeff = pinv(Py).dot(by)
                xyz_polycoeff.append(y_coeff)

                dy_coeff = polyder(y_coeff)
                dxyz_polycoeff.append(dy_coeff)

                ddy_coeff = polyder(dy_coeff)
                ddxyz_polycoeff.append(ddy_coeff)

            # Z trajectory depends directly on X not on time
            z0 = foot_init_position[2, 0]
            z1 = foot_end_position[2, 0]

            nz = 7  # number of coefficients for polynome on z
            Pz = np.zeros([nz, nz])
            # Position
            Pz[0, 0] = 1.
            Pz[1, :] += 1.
            # Velocity
            Pz[2, 1] = 1.
            Pz[3, 1:nz] = range(1, nz)
            # Mid trajectory constraint
            t_max = 0.4
            t_max = 0.5
            Pz[4, :] = np.power(t_max, range(nz))
            Pz[5, 1:] = range(1, nz) * np.power(t_max, range(nz - 1))
            Pz[6, 2:] = np.array(range(2, nz)) * np.array(range(1, nz - 1)) * np.power(t_max, range(nz - 2))

            bz = np.array([z0, z1, 2., -0., 0.5 * (z0 + z1) + self.z_amplitude, 0., -0.1])
            bz = np.array([z0, z1, 1., -0.8, 0.5 * (z0 + z1) + self.z_amplitude, 0., -0.1])
            z_coeff = pinv(Pz).dot(bz)
            xyz_polycoeff.append(z_coeff)

            dz_coeff = polyder(z_coeff)
            dxyz_polycoeff.append(dz_coeff)

            ddz_coeff = polyder(dz_coeff)
            ddxyz_polycoeff.append(ddz_coeff)

            self.polycoeff_l.append(xyz_polycoeff)
            self.dpolycoeff_l.append(dxyz_polycoeff)
            self.ddpolycoeff_l.append(ddxyz_polycoeff)

    def __call__(self, t):
        # assert t <= self.time_intervals[-1][1], "t must be lower than the final time tf={}".format(self.time_intervals[-1][1])

        index = len(self.t0_l) - 1
        if t > self.time_intervals[1]:
            t = self.time_intervals[1]
        elif t < self.time_intervals[0]:
            t = self.time_intervals[0]
            index = 0
        else:
            for k in range(len(self.t0_l)):
                if self.t0_l[k] > t:
                    index = k - 1
                    break

        xyz_polycoeff = self.polycoeff_l[index]
        dxyz_polycoeff = self.dpolycoeff_l[index]
        ddxyz_polycoeff = self.ddpolycoeff_l[index]

        t0 = self.time_intervals[0]
        t1 = self.time_intervals[1]

        if t0 == t1:
            tau = 0.
            dtau_dt = 0.
        else:
            tau = (t - t0) / (t1 - t0)
            dtau_dt = 1.

        # Evaluate X
        x = polyval(tau, xyz_polycoeff[0])
        if len(dxyz_polycoeff[0]):
            x_dot = polyval(tau, dxyz_polycoeff[0]) * dtau_dt
        else:
            x_dot = 0.

        if len(ddxyz_polycoeff[0]):
            x_dotdot = polyval(tau, ddxyz_polycoeff[0]) * dtau_dt**2
        else:
            x_dotdot = 0.

        # Evaluate Y
        y = polyval(tau, xyz_polycoeff[1])
        if len(dxyz_polycoeff[1]):
            y_dot = polyval(tau, dxyz_polycoeff[1]) * dtau_dt
        else:
            y_dot = 0.

        if len(ddxyz_polycoeff[1]):
            y_dotdot = polyval(tau, ddxyz_polycoeff[1]) * dtau_dt**2
        else:
            y_dotdot = 0.

        # Evaluate Z
        x0 = polyval(0., xyz_polycoeff[0])
        x1 = polyval(1., xyz_polycoeff[0])
        if x0 == x1:
            tau_x = 0.
            dtau_x_dt = 0.
        else:
            tau_x = (x - x0) / (x1 - x0)
            dtau_x_dt = x_dot

        z = polyval(tau_x, xyz_polycoeff[2])
        if len(dxyz_polycoeff[2]):
            z_dot = polyval(tau_x, dxyz_polycoeff[2]) * x_dot
        else:
            z_dot = 0.

        if len(ddxyz_polycoeff[2]):
            z_dotdot = polyval(tau_x, ddxyz_polycoeff[2]) * x_dot**2 + polyval(tau_x, dxyz_polycoeff[2]) * x_dotdot
        else:
            z_dotdot = 0.

        M = SE3.Identity()
        v = Motion.Zero()
        a = Motion.Zero()

        M.translation = np.matrix([x, y, z]).T
        M.rotation = self._R
        v.linear = np.matrix([x_dot, y_dot, z_dot]).T
        a.linear = np.matrix([x_dotdot, y_dotdot, z_dotdot]).T

        return M, v, a


class RefTrajectory(object):
    def __init__(self, name):
        self._name = name
        self._dim = 0

    @property
    def dim(self):
        return self._dim

    def has_trajectory_ended(self, t):
        return True

    def __call__(self, t):
        return np.matrix([]).reshape(0, 0)


class DifferentiableEuclidianTrajectory(RefTrajectory):
    def __init__(self, name="Differentiable trajectory"):
        self.name = name
        self._time_i = []
        self._t0_l = []
        self._coeffs_l = []
        self._dcoeffs_l = []

    @property
    def dim(self):
        return self._dim

    def computeFromPoints(self, timeline, p_points, v_points):
        m_p, n_p = p_points.shape
        m_v, n_v = v_points.shape

        timeline = timeline.A.squeeze()
        N = len(timeline.T)

        assert n_p == N and n_v == N
        assert m_p == m_v

        self._dim = m_p

        # Compute time intervals
        self._time_i = []
        self._t0_l = timeline[:-1]

        for k in range(N - 1):
            self._time_i.append((timeline[k], timeline[k + 1]))

        # Compute the polynomial coeff on each intervals
        self._coeffs_l = []
        self._dcoeffs_l = []

        for k in range(N - 1):

            coeffs = []
            dcoeffs = []
            for i in range(self._dim):
                X0 = p_points[i, k]
                X1 = p_points[i, k + 1]
                coeffs.append([X0, X1 - X0])

                dX0 = v_points[i, k]
                dX1 = v_points[i, k + 1]
                dcoeffs.append([dX0, dX1 - dX0])

            self._coeffs_l.append(coeffs)
            self._dcoeffs_l.append(dcoeffs)

    def __call__(self, t):
        #assert t <= self._time_i[-1][1], "t must be lower than the final time tf={}".format(self.time_i[-1][1])
        index = len(self._t0_l) - 1
        if t > self._time_i[-1][1]:
            t = self._time_i[-1][1]
        elif t < self._time_i[0][0]:
            t = self._time_i[0][0]
            index = 0
        else:
            for k in range(len(self._t0_l)):
                if self._t0_l[k] > t:
                    index = k - 1
                    break

        coeffs_l = self._coeffs_l[index]
        dcoeffs_l = self._dcoeffs_l[index]

        t0 = self._time_i[index][0]
        t1 = self._time_i[index][1]

        tau = (t - t0) / (t1 - t0)

        dim = self._dim
        X = np.matrix(np.zeros([dim, 1]))
        Xd = np.matrix(np.zeros([dim, 1]))

        for k in range(dim):
            if not coeffs_l[k] == []:
                X[k] = polyval(tau, coeffs_l[k])

            if not dcoeffs_l[k] == []:
                Xd[k] = polyval(tau, dcoeffs_l[k])

        return X, Xd


class TwiceDifferentiableEuclidianTrajectory(RefTrajectory):
    def __init__(self, name="Twice differentiable trajectory"):
        self.name = name
        self._time_i = []
        self._t0_l = []
        self._coeffs_l = []
        self._dcoeffs_l = []
        self._ddcoeffs_l = []

    @property
    def dim(self):
        return self._dim

    def computeFromPoints(self, timeline, p_points, v_points, a_points):
        #print p_points
        m_p, n_p = p_points.shape
        m_v, n_v = v_points.shape
        m_a, n_a = a_points.shape

        timeline = timeline.A.squeeze()
        N = len(timeline.T)

        assert n_p == N and n_v == N and n_a == N
        assert m_p == m_v and m_p == m_a

        self._dim = m_p

        # Compute time intervals
        self._time_i = []
        self._t0_l = timeline[:-1]

        for k in range(N - 1):
            self._time_i.append((timeline[k], timeline[k + 1]))

        # Compute the polynomial coeff on each intervals
        self._coeffs_l = []
        self._dcoeffs_l = []
        self._ddcoeffs_l = []

        for k in range(N - 1):

            coeffs = []
            dcoeffs = []
            ddcoeffs = []
            for i in range(self._dim):
                X0 = p_points[i, k]
                X1 = p_points[i, k + 1]
                coeffs.append([X0, X1 - X0])

                dX0 = v_points[i, k]
                dX1 = v_points[i, k + 1]
                dcoeffs.append([dX0, dX1 - dX0])

                ddX0 = a_points[i, k]
                ddX1 = a_points[i, k + 1]
                ddcoeffs.append([ddX0, ddX1 - ddX0])

            self._coeffs_l.append(coeffs)
            self._dcoeffs_l.append(dcoeffs)
            self._ddcoeffs_l.append(ddcoeffs)

    def __call__(self, t):
        #assert t <= self._time_i[-1][1], "t must be lower than the final time tf={}".format(self.time_i[-1][1])
        index = len(self._t0_l) - 1
        if t > self._time_i[-1][1]:
            t = self._time_i[-1][1]
        elif t < self._time_i[0][0]:
            t = self._time_i[0][0]
            index = 0
        else:
            for k in range(len(self._t0_l)):
                if self._t0_l[k] > t:
                    index = k - 1
                    break

        coeffs_l = self._coeffs_l[index]
        dcoeffs_l = self._dcoeffs_l[index]
        ddcoeffs_l = self._ddcoeffs_l[index]

        t0 = self._time_i[index][0]
        t1 = self._time_i[index][1]

        tau = (t - t0) / (t1 - t0)

        dim = self._dim
        X = np.matrix(np.zeros([dim, 1]))
        Xd = np.matrix(np.zeros([dim, 1]))
        Xdd = np.matrix(np.zeros([dim, 1]))

        for k in range(dim):
            if not coeffs_l[k] == []:
                X[k] = polyval(tau, coeffs_l[k])

            if not dcoeffs_l[k] == []:
                Xd[k] = polyval(tau, dcoeffs_l[k])

            if not ddcoeffs_l[k] == []:
                Xdd[k] = polyval(tau, ddcoeffs_l[k])

        return X, Xd, Xdd

class HPPEffectorTrajectory(RefTrajectory):
    def __init__(self, eeName, fullBody, problem, pid, name="HPP-effector-trajectory"):
        RefTrajectory.__init__(self, name)
        self._dim = 3
        self._eeName = eenName
        self._fb = fullBody
        self._problem = problem
        self._pid = pid
        self._length = problem.pathLength(pid)

    def __call__(self, t):
        if t < 0.:
            print("Trajectory called with negative time.")
            t = 0.
        elif t > self._length:
            print("Trajectory called after final time.")
            t = self._length
        return effectorPositionFromHPPPath(self._fb, self._problem, self._eeName, self._pid, t)
