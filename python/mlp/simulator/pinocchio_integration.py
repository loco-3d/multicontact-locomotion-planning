import pinocchio as pin

class PinocchioIntegration:
    """
    Perfect integration of the joint acceleration with pinocchio
    """
    def __init__(self, dt, model):
        self.model = model
        self.dt = dt
        self.q = None
        self.v = None

    @staticmethod
    def build_from_urdf(dt, urdf, package_path):
        robot = pin.RobotWrapper.BuildFromURDF(urdf, package_path, pin.JointModelFreeFlyer())
        return PinocchioIntegration(dt, robot.model)

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
        self.q = pin.integrate(self.model, self.q, self.dt * v_mean)
        return self.q, self.v
