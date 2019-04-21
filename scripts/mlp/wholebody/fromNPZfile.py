import mlp.config as cfg
from rospkg import RosPack
from mlp.utils import wholebody_result

def generateWholeBodyMotion(cs,fullBody=None,viewer=None):
    rp = RosPack()
    urdf = rp.get_path(cfg.Robot.packageName)+'/urdf/'+cfg.Robot.urdfName+cfg.Robot.urdfSuffix+'.urdf'
    if cfg.WB_VERBOSE:
        print "load robot : " ,urdf 
    pinRobot  = pin.RobotWrapper.BuildFromURDF(urdf, pin.StdVec_StdString(), pin.JointModelFreeFlyer(), False)        
    if cfg.WB_VERBOSE:
        print "robot loaded."
    filename = cfg.EXPORT_PATH+"/npz",cfg.DEMO_NAME+".npz"
    print "Load npz file : ",filename
    res = wholebody_result.loadFromNPZ(filename)
    return res,robot
    