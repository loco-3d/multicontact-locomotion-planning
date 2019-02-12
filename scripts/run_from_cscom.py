import hpp_wholebody_motion.viewer.display_tools as display_tools
from locomote import ContactSequenceHumanoid

cs_com = ContactSequenceHumanoid(0)
filename = "/local/dev_hpp/src/hpp-wholebody-motion/res/contact_sequences/talos_table_COM.xml" # A CHANGER
cs_com.loadFromXML(filename, "ContactSequence")  

import hpp_wholebody_motion.wholebody.tsid_invdyn as wb
q_t = wb.generateWholeBodyMotion(cs_com)


# initialiser un viewer et y charger les models ici ... 
# TODO
# display_tools.displayWBmotion(v,q_t,cfg.IK_dt,cfg.DT_DISPLAY)