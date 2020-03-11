from rospkg import RosPack
from multicontact_api import ContactSequence
from mlp.utils.requirements import Requirements as Inputs
import pinocchio as pin

class Outputs(Inputs):
    consistentContacts = True
    timings = True
    configurationValues = True
    jointsTrajectories = True

def generateWholeBodyMotion(cfg, cs, fullBody=None, viewer=None):
    rp = RosPack()
    urdf = rp.get_path(cfg.Robot.packageName) + '/urdf/' + cfg.Robot.urdfName + cfg.Robot.urdfSuffix + '.urdf'
    if cfg.WB_VERBOSE:
        print("load robot : ", urdf)
    robot = pin.RobotWrapper.BuildFromURDF(urdf, pin.StdVec_StdString(), pin.JointModelFreeFlyer(), False)
    if cfg.WB_VERBOSE:
        print("robot loaded.")
    cs_wb = ContactSequence()
    print("Load wholebody contact sequence from  file : ", cfg.WB_FILENAME)
    cs_wb.loadFromBinary(cfg.WB_FILENAME)
    return cs_wb, robot
