import hpp_wholebody_motion.config as cfg
import importlib
#the following script must produce a sequence of configurations in contact (configs) 
# with exactly one contact change between each configurations
# It must also initialise a FullBody object name fullBody and optionnaly a Viewer object named V
cp = importlib.import_module('scenarios.demos.'+cfg.DEMO_NAME)
import hpp_wholebody_motion.generate_contact_sequence as generate_cs
import hpp_wholebody_motion.display_tools as display_tools

v = cp.v

if cfg.DISPLAY_CS:
    import tools.display_tools as rbprmDisplay
    raw_input("Press Enter to display the contact sequence ...")
    rbprmDisplay.displayContactSequence(v,cp.configs)    

beginState = 0
endState = len(cp.configs) - 1
cs = generate_cs.generateContactSequence(cp.fullBody,cp.configs,beginState,endState)

if cfg.SAVE_CS :
    filename = cfg.CONTACT_SEQUENCE_PATH + "/"+cfg.DEMO_NAME+".xml"
    print "Write contact sequence XML file : ",filename
    cs.saveAsXML(filename, "ContactSequence")    
if cfg.DISPLAY_CS_STONES :
    display_tools.displaySteppingStones(cs,v)
    
if cfg.USE_GEOM_INIT_GUESS:
    print "Generate geometric init guess."
    cs_initGuess = generate_cs.generateGeometricInitGuess(cs)
if cfg.USE_CROC_INIT_GUESS:
    print "Generate init guess with CROC."
    cs_initGuess = generate_cs.generateCROCinitGuess(cs,cp.fullBody,beginState,endState)
if cfg.DISPLAY_INIT_GUESS_TRAJ and (cfg.USE_GEOM_INIT_GUESS or cfg.USE_CROC_INIT_GUESS):
    colors = [v.color.red, v.color.yellow]
    display_tools.displayCOMTrajectory(cs,v,colors,"_init")

import hpp_wholebody_motion.centroidal_timeopt as timeopt
cs_com,tp = timeopt.generateCentroidalTrajectory(cs,cs_initGuess)
print "Duration of the motion : "+str(cs_com.contact_phases[-1].time_trajectory[-1])+" s."


if cfg.SAVE_CS_COM :
    filename = cfg.CONTACT_SEQUENCE_PATH + "/"+cfg.DEMO_NAME+"_COM.xml"
    print "Write contact sequence with centroidal trajectory XML file : ",filename
    cs_com.saveAsXML(filename, "ContactSequence") 
if cfg.DISPLAY_COM_TRAJ:
    colors = [v.color.blue, v.color.green]
    display_tools.displayCOMTrajectory(cs_com,v,colors)

import hpp_wholebody_motion.whole_body as wb
if USE_CROC_COM:
    assert USE_CROC_INIT_GUESS, "You must generate CROC initial guess if you want to use it as reference for the COM"  
    q_t = wb.generateWholeBodyMotion(cs_initGuess,v)
else : 
    q_t = wb.generateWholeBodyMotion(cs_com,v)

if cfg.DISPLAY_WB_MOTION:
    raw_input("Press Enter to display the whole body motion ...")
    display_tools.displayWBmotion(v,q_t,cfg.IK_dt,cfg.DT_DISPLAY)
