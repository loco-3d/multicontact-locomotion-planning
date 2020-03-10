from multicontact_api import ContactSequence
import mlp.viewer.display_tools as display_tools
from mlp.utils.requirements import Requirements


class Outputs(Requirements):
    consistentContacts = True



def generate_contact_sequence_load(cfg):
    fb, v = display_tools.initScene(cfg.Robot, cfg.ENV_NAME)
    cs = ContactSequence(0)
    print("Import contact sequence binary file : ", cfg.CS_FILENAME)
    cs.loadFromBinary(cfg.CS_FILENAME)
    display_tools.displayWBconfig(v, cs.contactPhases[0].q_init)
    return cs, fb, v
