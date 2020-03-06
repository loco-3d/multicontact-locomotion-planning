import pinocchio as pin

class PinocchioIntegration:
    """
    Perfect integration of the joint acceleration with pinocchio
    """
    def __init__(self, urdf, package_path, dt):
        self.robot = pin.RobotWrapper.BuildFromURDF(urdf, package_path, pin.JointModelFreeFlyer())
        self.dt = dt
        self.q = None
        self.v = None

    def init(self, q0, v0):
        self.q = q0
        self.v = v0

    def simulate(self, dv):
        """
        Update state by integrating the given joint acceleration
        :param dv: joint acceleration
        :return: updated wholebody configuration and joint velocity
        """
        v_mean = self.v + 0.5 * self.dt * dv
        self.v += self.dt * dv
        self.q = pin.integrate(self.robot.model(), self.q, self.dt * v_mean)
        return self.q, self.v
