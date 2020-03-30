import multicontact_api
from multicontact_api import ContactSequence
from mlp.utils.requirements import Requirements as CentroidalInputsLoad
import logging
logging.basicConfig(format='[%(name)-12s] %(levelname)-8s: %(message)s')
logger = logging.getLogger("load cs")
logger.setLevel(logging.WARNING) #DEBUG, INFO or WARNING
multicontact_api.switchToNumpyArray()

class CentroidalOutputsLoad(CentroidalInputsLoad):
    consistentContacts = True
    timings = True
    centroidalTrajectories = True
    centroidalValues = True


def generate_centroidal_load(cfg, cs, cs_initGuess=None, fullBody=None, viewer=None, first_iter = True):
    cs_res = ContactSequence(0)
    cs_res.loadFromBinary(cfg.COM_FILENAME)
    logger.warning("Import contact sequence binary file : %s", cfg.COM_FILENAME)
    return cs_res
