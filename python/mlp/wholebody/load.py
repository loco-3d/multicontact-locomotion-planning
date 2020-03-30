from rospkg import RosPack
from multicontact_api import ContactSequence
from mlp.utils.requirements import Requirements as WholebodyInputsLoad
import pinocchio as pin
import logging
logging.basicConfig(format='[%(name)-12s] %(levelname)-8s: %(message)s')
logger = logging.getLogger("load wholebody")
logger.setLevel(logging.DEBUG) #DEBUG, INFO or WARNING

class WholebodyOutputsLoad(WholebodyInputsLoad):
    consistentContacts = True
    timings = True
    configurationValues = True
    jointsTrajectories = True

def generate_wholebody_load(cfg, cs, fullBody=None, viewer=None):
    rp = RosPack()
    urdf = rp.get_path(cfg.Robot.packageName) + '/urdf/' + cfg.Robot.urdfName + cfg.Robot.urdfSuffix + '.urdf'
    logger.info("load robot : %s", urdf)
    robot = pin.RobotWrapper.BuildFromURDF(urdf, pin.StdVec_StdString(), pin.JointModelFreeFlyer(), False)
    logger.info("robot loaded.")
    cs_wb = ContactSequence()
    logger.warning("Load wholebody contact sequence from  file : %s", cfg.WB_FILENAME)
    cs_wb.loadFromBinary(cfg.WB_FILENAME)
    return cs_wb
