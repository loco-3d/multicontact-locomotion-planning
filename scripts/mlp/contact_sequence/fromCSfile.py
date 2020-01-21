from multicontact_api import ContactSequenceHumanoid
import mlp.config as cfg
import mlp.viewer.display_tools as display_tools

def generateContactSequence():
    fb,v = display_tools.initScene(cfg.Robot,cfg.ENV_NAME)
    cs = ContactSequenceHumanoid(0)
    filename = cfg.CONTACT_SEQUENCE_PATH + "/"+cfg.DEMO_NAME+".cs"   
    print("Import contact sequence binary file : ",filename)    
    cs.loadFromBinary(filename) 
    display_tools.displayWBconfig(v,cs.contact_phases[0].reference_configurations[0])
    return cs,fb,v