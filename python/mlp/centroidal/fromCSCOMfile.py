import multicontact_api
from multicontact_api import ContactSequence
from mlp.utils.requirements import Requirements as Inputs

multicontact_api.switchToNumpyArray()

class Outputs(Inputs):
    consistentContacts = True
    timings = True
    centroidalTrajectories = True
    centroidalValues = True


def generateCentroidalTrajectory(cfg, cs, cs_initGuess=None, fullBody=None, viewer=None):
    cs_res = ContactSequence(0)
    cs_res.loadFromBinary(cfg.COM_FILENAME)
    print("Import contact sequence binary file : ", cfg.COM_FILENAME)
    return cs_res
