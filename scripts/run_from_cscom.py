from multicontact_api import ContactSequenceHumanoid

cs_com = ContactSequenceHumanoid(0)
filename = "/local/dev_hpp/src/hpp-wholebody-motion/res/contact_sequences/talos_table_COM.xml" # FIXME : load it from somewhere
cs_com.loadFromXML(filename, "ContactSequence")  

import mlp.wholebody.tsid_invdyn as wb
res,robot = wb.generateWholeBodyMotion(cs_com)


# Init viewer and load models here ... 
# TODO
# import mlp.viewer.display_tools as display_tools
# display_tools.displayWBmotion(v,res.q_t,cfg.IK_dt,cfg.DT_DISPLAY)