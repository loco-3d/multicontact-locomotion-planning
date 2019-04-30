import numpy as np
import numpy.matlib
from numpy.polynomial.polynomial import polyval
#from  numpy import polyder
from  numpy.linalg import pinv
import pinocchio as pin
from pinocchio import SE3, log3, exp3, Motion, Quaternion
from pinocchio.utils import zero as mat_zeros

# COM Trajecotry smoothing using cubic spline
def polyder(coeffs):
  return np.polyder(coeffs[::-1])[::-1]

class SmoothedCOMTrajectory(object):
    def __init__(self, name, phase, com_init, dt, t_init=0.0):
        self._name = name
        self._dim = 3
        self._dt = dt;
        self._com_init = com_init
        self._t_init = t_init
        self._phase = phase
        self.__compute()

    @property
    def dim(self):
        return self._dim

    def has_trajectory_ended(self, t):
        return (t - self._t_init >= self._T);

    def __compute(self):
        self.polycoeff_l = []
        self.dpolycoeff_l = []
        self.ddpolycoeff_l = []

        num_interval = len(self._phase.time_trajectory)
        self._num_interval = num_interval

        for i in range(num_interval):
            xyz_polycoeff = []
            dxyz_polycoeff = []
            ddxyz_polycoeff = []

            if i > 0:
                self._com_init = self._phase.state_trajectory[i-1]

            com_end = self._phase.state_trajectory[i]
            # for com in x-direction
            nx = 6
            Px = np.zeros([nx, nx])
            Px[0, 0] = 1.
            Px[1, :] += 1.
            Px[2, 1] = 1.
            Px[3, 1:] = range(1, nx)
            Px[4, 2] = 1.
            Px[5, 2:] = np.array(range(2, nx)) * np.array(range(1, nx - 1))

            bx = []
            bx = np.array([self._com_init[0, 0], com_end[0, 0], 0.0, 0.0, 0., 0.0])

            x_coeff = pinv(Px).dot(bx)
            xyz_polycoeff.append(x_coeff)

            dx_coeff = polyder(x_coeff)
            dxyz_polycoeff.append(dx_coeff)

            ddx_coeff = polyder(dx_coeff)
            ddxyz_polycoeff.append(ddx_coeff)

            # for com in y-direction
            ny = 6
            Py = np.zeros([ny, ny])
            Py[0, 0] = 1.
            Py[1, :] += 1.
            Py[2, 1] = 1.
            Py[3, 1:] = range(1, ny)
            Py[4, 2] = 1.
            Py[5, 2:] = np.array(range(2, ny)) * np.array(range(1, ny - 1))

            by = []
            by = np.array([self._com_init[1, 0], com_end[1, 0], 0., 0.0, 0., 0.])
            y_coeff = pinv(Py).dot(by)
            xyz_polycoeff.append(y_coeff)

            dy_coeff = polyder(y_coeff)
            dxyz_polycoeff.append(dy_coeff)

            ddy_coeff = polyder(dy_coeff)
            ddxyz_polycoeff.append(ddy_coeff)

            # for com in z-direction
            nz = 6
            Pz = np.zeros([nz, nz])
            Pz[0, 0] = 1.
            Pz[1, :] += 1.
            Pz[2, 1] = 1.
            Pz[3, 1:] = range(1, nz)
            Pz[4, 2] = 1.
            Pz[5, 2:] = np.array(range(2, nz)) * np.array(range(1, nz - 1))

            bz = []
            bz = np.array([self._com_init[2, 0], com_end[2, 0], 0., 0., 0., 0.])

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
        index = 0
        if t > self._phase.time_trajectory[self._num_interval -1]:
            t = self._phase.time_trajectory[self._num_interval -1]

        else:
            for k in range(0, self._num_interval):
                if self._phase.time_trajectory[k] > t:
                    index = k
                    break

        xyz_polycoeff = self.polycoeff_l[index]
        dxyz_polycoeff = self.dpolycoeff_l[index]
        ddxyz_polycoeff = self.ddpolycoeff_l[index]

        if index == 0:
            t0 = 0.0
        else:
            t0 = self._phase.time_trajectory[index-1]
        t1 = self._phase.time_trajectory[index]

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
            x_dotdot = polyval(tau, ddxyz_polycoeff[0]) * dtau_dt ** 2
        else:
            x_dotdot = 0.

        # Evaluate Y
        y = polyval(tau, xyz_polycoeff[1])
        if len(dxyz_polycoeff[1]):
            y_dot = polyval(tau, dxyz_polycoeff[1]) * dtau_dt
        else:
            y_dot = 0.

        if len(ddxyz_polycoeff[1]):
            y_dotdot = polyval(tau, ddxyz_polycoeff[1]) * dtau_dt ** 2
        else:
            y_dotdot = 0.

        # Evaluate Z
        z = polyval(tau, xyz_polycoeff[2])
        if len(dxyz_polycoeff[2]):
            z_dot = polyval(tau, dxyz_polycoeff[2]) * dtau_dt
        else:
            z_dot = 0.

        if len(ddxyz_polycoeff[2]):
            z_dotdot = polyval(tau, ddxyz_polycoeff[2]) * dtau_dt ** 2
        else:
            z_dotdot = 0.

        com_pos = np.matrix([x, y, z])
        com_vel = np.matrix([x_dot, y_dot, z_dot])
        com_acc = np.matrix([x_dotdot, y_dotdot, z_dotdot])

        return com_pos, com_vel, com_acc
        #return (self._x_ref[:, i], self._v_ref[:, i], self._a_ref[:, i]);

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
            x_dotdot = polyval(tau, ddxyz_polycoeff[0]) * dtau_dt ** 2
        else:
            x_dotdot = 0.

        # Evaluate Y
        y = polyval(tau, xyz_polycoeff[1])
        if len(dxyz_polycoeff[1]):
            y_dot = polyval(tau, dxyz_polycoeff[1]) * dtau_dt
        else:
            y_dot = 0.

        if len(ddxyz_polycoeff[1]):
            y_dotdot = polyval(tau, ddxyz_polycoeff[1]) * dtau_dt ** 2
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
            z_dotdot = polyval(tau_x, ddxyz_polycoeff[2]) * x_dot ** 2 + polyval(tau_x, dxyz_polycoeff[2]) * x_dotdot
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

class RefTrajectory (object):

  def __init__ (self, name):
    self._name = name
    self._dim = 0

  @property
  def dim(self):
    return self._dim

  def has_trajectory_ended(self, t):
    return True;

  def __call__ (self, t):
    return np.matrix ([]).reshape (0, 0)
  


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


    for k in range(N-1):
      self._time_i.append((timeline[k], timeline[k+1]))

    # Compute the polynomial coeff on each intervals 
    self._coeffs_l = []
    self._dcoeffs_l = []

    for k in range(N-1):

      coeffs = []
      dcoeffs = []
      for i in range(self._dim):
        X0 = p_points[i,k]
        X1 = p_points[i,k+1]
        coeffs.append([X0, X1 - X0])

        dX0 = v_points[i,k]
        dX1 = v_points[i,k+1]
        dcoeffs.append([dX0, dX1 - dX0])

      self._coeffs_l.append(coeffs)
      self._dcoeffs_l.append(dcoeffs)

  def __call__ (self, t):
    #assert t <= self._time_i[-1][1], "t must be lower than the final time tf={}".format(self.time_i[-1][1])
    index = len(self._t0_l)-1
    if t > self._time_i[-1][1]:
      t = self._time_i[-1][1]
    elif t < self._time_i[0][0]:
      t = self._time_i[0][0]
      index = 0
    else:
      for k in range(len(self._t0_l)):
        if self._t0_l[k] > t:
          index = k-1
          break

    coeffs_l = self._coeffs_l[index]
    dcoeffs_l = self._dcoeffs_l[index]

    t0 = self._time_i[index][0]
    t1 = self._time_i[index][1]

    tau = (t-t0)/(t1-t0)

    dim = self._dim
    X = np.matrix(np.zeros([dim,1]))
    Xd = np.matrix(np.zeros([dim,1]))

    for k in range(dim):
      if not coeffs_l[k] == []: 
        X[k] = polyval(tau,coeffs_l[k])

      if not dcoeffs_l[k] == []: 
        Xd[k] = polyval(tau,dcoeffs_l[k])

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


    for k in range(N-1):
      self._time_i.append((timeline[k], timeline[k+1]))

    # Compute the polynomial coeff on each intervals
    self._coeffs_l = []
    self._dcoeffs_l = []
    self._ddcoeffs_l = []

    for k in range(N-1):

      coeffs = []
      dcoeffs = []
      ddcoeffs = []
      for i in range(self._dim):
        X0 = p_points[i,k]
        X1 = p_points[i,k+1]
        coeffs.append([X0, X1 - X0])

        dX0 = v_points[i,k]
        dX1 = v_points[i,k+1]
        dcoeffs.append([dX0, dX1 - dX0])

        ddX0 = a_points[i,k]
        ddX1 = a_points[i,k+1]
        ddcoeffs.append([ddX0, ddX1 - ddX0])

      self._coeffs_l.append(coeffs)
      self._dcoeffs_l.append(dcoeffs)
      self._ddcoeffs_l.append(ddcoeffs)

  def __call__ (self, t):
    #assert t <= self._time_i[-1][1], "t must be lower than the final time tf={}".format(self.time_i[-1][1])
    index = len(self._t0_l)-1
    if t > self._time_i[-1][1]:
      t = self._time_i[-1][1]
    elif t < self._time_i[0][0]:
      t = self._time_i[0][0]
      index = 0
    else:
      for k in range(len(self._t0_l)):
        if self._t0_l[k] > t:
          index = k-1
          break

    coeffs_l = self._coeffs_l[index]
    dcoeffs_l = self._dcoeffs_l[index]
    ddcoeffs_l = self._ddcoeffs_l[index]

    t0 = self._time_i[index][0]
    t1 = self._time_i[index][1]

    tau = (t-t0)/(t1-t0)

    dim = self._dim
    X = np.matrix(np.zeros([dim,1]))
    Xd = np.matrix(np.zeros([dim,1]))
    Xdd = np.matrix(np.zeros([dim,1]))

    for k in range(dim):
      if not coeffs_l[k] == []:
        X[k] = polyval(tau,coeffs_l[k])

      if not dcoeffs_l[k] == []:
        Xd[k] = polyval(tau,dcoeffs_l[k])

      if not ddcoeffs_l[k] == []:
        Xdd[k] = polyval(tau,ddcoeffs_l[k])

    return X, Xd, Xdd

class BezierTrajectory(RefTrajectory):


  def __init__(self,curves,placement_init,placement_end,time_interval):
    RefTrajectory.__init__(self,"BezierTrajectory")
    self.placement_init=placement_init
    self.placement_end=placement_end
    self.curves = curves
    self.time_interval = time_interval
    self.t_total = curves.max()
    assert abs(self.t_total - (time_interval[1] - time_interval[0])) <= 1e-4, "time interval is not coherent with the length of the Bezier curves"
    assert len(curves.times)%2 == 0, "PolyBezier object contain an even number of curves, not implemented yet."
    id_mid = len(curves.times)/2
    # retrieve the timings of the middle segment (duration and begin/end wrt to the other curves)
    self.t_mid_begin = curves.times[id_mid-1]
    self.t_mid_end = curves.times[id_mid]
    self.t_mid = curves.curves[curves.numCurves()/2].max()
    
    curves.computeDerivates()
    
    self.M = SE3.Identity()
    self.v = Motion.Zero()
    self.a = Motion.Zero()

  
  
  def __call__(self,t):
    return self.compute_for_normalized_time(t - self.time_interval[0])
  
  def compute_for_normalized_time(self,t):
    if t < 0:
      print "Trajectory called with negative time."
      return self.compute_for_normalized_time(0)
    elif t > self.t_total:
      print "Trajectory called after final time."
      return self.compute_for_normalized_time(self.t_total)
    self.M = SE3.Identity()    
    self.v = Motion.Zero()
    self.a = Motion.Zero()         
    self.M.translation = self.curves(t)
    self.v.linear = self.curves.d(t)
    self.a.linear = self.curves.dd(t)
    #rotation : 
    if self.curves.isInFirstNonZero(t):
      self.M.rotation = self.placement_init.rotation.copy()
    elif self.curves.isInLastNonZero(t):
      self.M.rotation = self.placement_end.rotation.copy()
    else:
      # make a slerp between self.effector_placement[id][0] and [1] :
      quat0 = Quaternion(self.placement_init.rotation)
      quat1 = Quaternion(self.placement_end.rotation)
      t_rot = t - self.t_mid_begin
      """
      print "t : ",t
      print "t_mid_begin : ",self.t_mid_begin
      print "t_rot : ",t_rot
      print "t mid : ",self.t_mid
      """
      assert t_rot >= 0 , "Error in the time intervals of the polybezier"
      assert t_rot <= (self.t_mid + 1e-6) , "Error in the time intervals of the polybezier"
      u =  t_rot/self.t_mid
      # normalized time without the pre defined takeoff/landing phases
      self.M.rotation = (quat0.slerp(u,quat1)).matrix()
      # angular velocity :
      dt=0.001
      u_dt = dt/self.t_mid
      r_plus_dt = (quat0.slerp(u+u_dt,quat1)).matrix()
      self.v.angular= pin.log3(self.M.rotation.T * r_plus_dt)/dt
      r_plus2_dt = (quat0.slerp(u+(2.*u_dt),quat1)).matrix()
      next_angular_velocity = pin.log3( r_plus_dt.T * r_plus2_dt)/dt
      self.a.angular = ( next_angular_velocity - self.v.angular)/dt
      #r_plus_dt = (quat0.slerp(u+u_dt,quat1)).matrix()            
      #next_angular_vel = (pin.log3(self.M.rotation.T * r_plus_dt)/dt)
      #self.a.angular = (next_angular_vel - self.v.angular)/dt    
    return self.M, self.v, self.a

class TrajectorySE3LinearInterp(RefTrajectory):
  
  def __init__(self,placement_init,placement_final,time_interval):
    RefTrajectory.__init__(self,"TrajectorySE3LinearInterp")
    self.placement_init = placement_init    
    self.placement_final = placement_final
    self.t0 = time_interval[0]
    self.t1 = time_interval[1]
    self.length = self.t1 - self.t0
    self.quat0 = Quaternion(self.placement_init.rotation)
    self.quat1 = Quaternion(self.placement_final.rotation)    
    self.M = SE3.Identity()
    self.v = Motion.Zero()
    self.a = Motion.Zero()    
    # constant velocity and null acceleration : 
    self.v.linear = (placement_final.translation - placement_final.translation)/self.length
    self.v.angular= pin.log3(placement_final.rotation.T * placement_final.rotation)/self.length
    
    

  def __call__(self,t):
    return self.compute_for_normalized_time(t - self.t0)
  
  def compute_for_normalized_time(self,t):
    if t < 0:
      print "Trajectory called with negative time."
      return self.compute_for_normalized_time(0)
    elif t > self.length:
      print "Trajectory called after final time."
      return self.compute_for_normalized_time(self.t_total)    
    u = t/self.length
    self.M = SE3.Identity()
    self.M.translation = u*self.placement_final.translation + (1.-u)*self.placement_init.translation
    self.M.rotation = (self.quat0.slerp(u,self.quat1)).matrix()  
    return self.M, self.v, self.a
    

class HPPEffectorTrajectory (RefTrajectory):

  def __init__ (self,eeName,fullBody,problem,pid,name = "HPP-effector-trajectory"):
    RefTrajectory.__init__(self,name)
    self._dim = 3
    self._eeName = eenName
    self._fb = fullBody
    self._problem = problem
    self._pid = pid
    self._length = problem.pathLength(pid)

  def __call__ (self, t):
    if t < 0. : 
      print "Trajectory called with negative time."
      t = 0.
    elif t > self._length:
      print "Trajectory called after final time."
      t = self._length    
    return effectorPositionFromHPPPath(self._fb,self._problem,self._eeName,self._pid,t)

# coefs represent a polynome in the forme
# c0 + c1*x +c2*x^2 + ... +cn*x^n
# returns the coefficients of the first derivative
def derivatePolynome(coefs):
  dim,deg = coefs.shape
  deg -= 1
  dcoefs = np.matrix(np.zeros([dim,deg]))
  for i in range(deg):
    dcoefs[:,i] = float(i+1.)*coefs[:,i+1]
  return dcoefs

## thrid order spline with equation : 
# c0 + c1*t + c2*t^2 +c3*t^3
class cubicSplineTrajectory(RefTrajectory):
  def __init__ (self,t_init,t_end,p_init,v_init,p_end,v_end, name="cubic-spline-trajectory"):
    RefTrajectory.__init__(self,name)
    self.t_init = t_init
    self.t_end = t_end
    T = t_end - t_init
    self.T = T
    dim = len(p_init)
    assert len(v_init) == dim
    assert len(p_end) == dim
    assert len(v_end) == dim
    self._dim = dim
    self.coefs=np.matrix(np.zeros([dim,4])) # one column for each coefficient
    T2 = T*T
    T3 = T2*T
    # M_inv : matrice use to compute the polynomial coefficient : [coefs] = M [boundary conditions]
    M_inv = np.matrix(np.zeros([4,4]))
    M_inv[0,:] = np.matrix([1.,0.,0.,0.])
    M_inv[1,:] = np.matrix([1.,T,T2,T3])
    M_inv[2,:] = np.matrix([0.,1.,0.,0.])
    M_inv[3,:] = np.matrix([0.,1.,2.*T,3.*T2])
    M = pinv(M_inv)
    for k in range(self._dim):
      # compute boundary condition vector (column) : 
      bc = np.matrix(np.zeros(4)).T
      bc[0,0] = p_init[k,0]
      bc[1,0] = p_end[k,0]
      bc[2,0] = v_init[k,0]
      bc[3,0] = v_end[k,0]
      ## compute polynomes coefficients from boundary conditions : 
      self.coefs[k,:] = (M.dot(bc)).T
    # compute values of coefficients for first derivative : 
    self.dcoefs = derivatePolynome(self.coefs)
    

  def __call__ (self, t_total):
    assert t_total >= self.t_init and "t less than min bound"
    assert t_total <= self.t_end and "t greater than max bound"
    # compute normalized time : 
    t = (t_total-self.t_init)
    X = np.matrix(np.zeros(self._dim)).T
    dX = np.matrix(np.zeros(self._dim)).T
    for k in range(self._dim):
      X[k,0] = polyval(t,self.coefs[k,:].T)
      dX[k,0] = polyval(t,self.dcoefs[k,:].T)

    return X,dX 
  



## five order spline with equation : 
# c0 + c1*t + c2*t^2 +c3*t^3 +c4*t^4 +c5*t^5 
class quinticSplineTrajectory(RefTrajectory):
  def __init__ (self,t_init,t_end,p_init,v_init,a_init,p_end,v_end,a_end, name="qintic-spline-trajectory"):
    RefTrajectory.__init__(self,name)
    self.t_init = t_init
    self.t_end = t_end
    T = t_end - t_init
    self.T = T
    dim = len(p_init)
    assert len(v_init) == dim
    assert len(a_init) == dim
    assert len(p_end) == dim
    assert len(v_end) == dim
    assert len(a_end) == dim
    self._dim = dim
    self.coefs=np.matrix(np.zeros([dim,6])) # one column for each coefficient
    self.dcoefs=np.matrix(np.zeros([dim,5])) # one column for each coefficient
    self.ddcoefs=np.matrix(np.zeros([dim,4])) # one column for each coefficient
    T2 = T*T
    T3 = T2*T
    T4 = T3*T
    T5 = T4*T
    # M_inv : matrice use to compute the polynomial coefficient : [coefs] = M * [boundary conditions]
    M_inv = np.matrix(np.zeros([6,6]))
    M_inv[0,:] = np.matrix([1.,0.,0.,0.,0.,0.])
    M_inv[1,:] = np.matrix([1.,T,T2,T3,T4,T5])
    M_inv[2,:] = np.matrix([0.,1.,0.,0.,0.,0.])
    M_inv[3,:] = np.matrix([0.,1.,2.*T,3.*T2,4.*T3,5.*T4])
    M_inv[4,:] = np.matrix([0.,0.,2.,0.,0.,0.])
    M_inv[5,:] = np.matrix([0.,0.,2.,6.*T,12.*T2,20.*T3])
    M = pinv(M_inv)
    for k in range(self._dim):
      # compute boundary condition vector (column) : 
      bc = np.matrix(np.zeros(6)).T
      bc[0,0] = p_init[k,0]
      bc[1,0] = p_end[k,0]
      bc[2,0] = v_init[k,0]
      bc[3,0] = v_end[k,0]
      bc[4,0] = a_init[k,0]
      bc[5,0] = a_end[k,0]
      ## compute polynomes coefficients from boundary conditions : 
      self.coefs[k,:] = (M.dot(bc)).T
    self.dcoefs = derivatePolynome(self.coefs)
    self.ddcoefs = derivatePolynome(self.dcoefs)
    
    
  def __call__ (self, t_total):
    assert t_total >= self.t_init and "t less than min bound"
    assert t_total <= self.t_end and "t greater than max bound"
    # compute normalized time : 
    t = (t_total-self.t_init)
    X = np.matrix(np.zeros(self._dim)).T
    dX = np.matrix(np.zeros(self._dim)).T
    ddX = np.matrix(np.zeros(self._dim)).T
    
    for k in range(self._dim):
      X[k,0] = polyval(t,self.coefs[k,:].T)
      dX[k,0] = polyval(t,self.dcoefs[k,:].T)
      ddX[k,0] = polyval(t,self.ddcoefs[k,:].T)

    return X,dX,ddX 
  
  