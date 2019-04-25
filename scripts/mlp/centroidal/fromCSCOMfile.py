import mlp.config as cfg
from multicontact_api import ContactSequenceHumanoid

def generateCentroidalTrajectory(cs,cs_initGuess = None, fullBody = None, viewer = None):
    cs_res = ContactSequenceHumanoid(0)
    filename = cfg.CONTACT_SEQUENCE_PATH + "/"+cfg.DEMO_NAME+"_COM.cs"
    cs_res.loadFromBinary(filename) 
    print "Import contact sequence binary file : ",filename    
    return cs_res
