import mlp.config as cfg
import multicontact_api
from multicontact_api import ContactSequence
multicontact_api.switchToNumpyArray()


def generateCentroidalTrajectory(cs, cs_initGuess=None, fullBody=None, viewer=None):
    cs_res = ContactSequence(0)
    filename = cfg.CONTACT_SEQUENCE_PATH + "/" + cfg.DEMO_NAME + "_COM.cs"
    cs_res.loadFromBinary(filename)
    print("Import contact sequence binary file : ", filename)
    return cs_res
