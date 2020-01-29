from multicontact_api import ContactSequence
import mlp.config as cfg
import mlp.viewer.display_tools as display_tools
from mlp.utils.requirements import Requirements

class Outputs(Requirements):
    consistentContacts = True



def generateContactSequence():
    fb, v = display_tools.initScene(cfg.Robot, cfg.ENV_NAME)
    cs = ContactSequence(0)
    filename = cfg.CONTACT_SEQUENCE_PATH + "/" + cfg.DEMO_NAME + ".cs"
    print("Import contact sequence binary file : ", filename)
    cs.loadFromBinary(filename)
    display_tools.displayWBconfig(v, cs.contactPhases[0].q_init)
    return cs, fb, v
