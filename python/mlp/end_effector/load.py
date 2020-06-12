from mlp.utils.requirements import Requirements as EffectorInputsLoad
from multicontact_api import ContactSequence

class EffectorOutputsLoad(EffectorInputsLoad):
    effectorTrajectories = True

def generate_effector_trajectories_for_sequence_load(cfg, cs, fullBody=None):
    cs_ref = ContactSequence()
    filename = cfg.REF_FILENAME
    print("Load contact sequence with end effector trajectories from : ", filename)
    cs_ref.loadFromBinary(filename)
    return cs_ref
