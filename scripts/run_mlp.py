import mlp.config as cfg
import importlib
#the following script must produce a sequence of configurations in contact (configs) 
# with exactly one contact change between each configurations
# It must also initialise a FullBody object name fullBody and optionnaly a Viewer object named V
cp = importlib.import_module('scenarios.'+cfg.SCRIPT_PATH+'.'+cfg.DEMO_NAME)
import mlp.contact_sequence.rbprm as generate_cs
import mlp.viewer.display_tools as display_tools
from multicontact_api import ContactSequenceHumanoid

v = cp.v

if cfg.DISPLAY_CS:
    import tools.display_tools as rbprmDisplay
    raw_input("Press Enter to display the contact sequence ...")
    rbprmDisplay.displayContactSequence(v,cp.configs,0.2) 
    
if cfg.LOAD_CS:
    cs = ContactSequenceHumanoid(0)
    filename = cfg.CONTACT_SEQUENCE_PATH + "/"+cfg.DEMO_NAME+".xml"   
    print "Import contact sequence XML file : ",filename    
    cs.loadFromXML(filename, "ContactSequence") 
else:
    beginState = 0
    endState = len(cp.configs) - 1
    cs = generate_cs.generateContactSequence(cp.fullBody,cp.configs,beginState,endState)

if cfg.SAVE_CS and not cfg.LOAD_CS:
    filename = cfg.CONTACT_SEQUENCE_PATH + "/"+cfg.DEMO_NAME+".xml"
    print "Write contact sequence XML file : ",filename
    cs.saveAsXML(filename, "ContactSequence")    
if cfg.DISPLAY_CS_STONES :
    display_tools.displaySteppingStones(cs,v)
    
if cfg.USE_GEOM_INIT_GUESS:
    print "Generate geometric init guess."
    import mlp.centroidal.geometric as initGuess_geom 
    cs_initGuess = initGuess_geom.generateCentroidalTrajectory(cs)
if cfg.USE_CROC_INIT_GUESS:
    print "Generate init guess with CROC."
    import mlp.centroidal.croc as initGuess_croc    
    cs_initGuess = initGuess_croc.generateCentroidalTrajectory(cs,cp.fullBody,beginState,endState)
if cfg.DISPLAY_INIT_GUESS_TRAJ and (cfg.USE_GEOM_INIT_GUESS or cfg.USE_CROC_INIT_GUESS):
    colors = [v.color.red, v.color.yellow]
    display_tools.displayCOMTrajectory(cs_initGuess,v,colors,"_init")

if cfg.LOAD_CS_COM :
    cs_com = ContactSequenceHumanoid(0)
    filename = cfg.CONTACT_SEQUENCE_PATH + "/"+cfg.DEMO_NAME+"_COM.xml"
    cs_com.loadFromXML(filename, "ContactSequence")     
else:
    import mlp.centroidal.topt as centroidal
    cs_com,tp = centroidal.generateCentroidalTrajectory(cs,cs_initGuess,v)
    print "Duration of the motion : "+str(cs_com.contact_phases[-1].time_trajectory[-1])+" s."


if cfg.SAVE_CS_COM and not cfg.LOAD_CS_COM:
    filename = cfg.CONTACT_SEQUENCE_PATH + "/"+cfg.DEMO_NAME+"_COM.xml"
    print "Write contact sequence with centroidal trajectory XML file : ",filename
    cs_com.saveAsXML(filename, "ContactSequence") 
if cfg.DISPLAY_COM_TRAJ:
    colors = [v.color.blue, v.color.green]
    display_tools.displayCOMTrajectory(cs_com,v,colors)

import mlp.wholebody.tsid_invdyn as wb
if cfg.USE_CROC_COM:
    assert cfg.USE_CROC_INIT_GUESS, "You must generate CROC initial guess if you want to use it as reference for the COM"  
    res,robot = wb.generateWholeBodyMotion(cs_initGuess,v,cp.fullBody)
else : 
    #q_t,v_t,a_t = wb.generateWholeBodyMotion(cs_com,v,cp.fullBody)
    res,robot =  wb.generateWholeBodyMotion(cs_com,v,cp.fullBody)

if cfg.DISPLAY_WB_MOTION:
    raw_input("Press Enter to display the whole body motion ...")
    display_tools.displayWBmotion(v,res.q_t,cfg.IK_dt,cfg.DT_DISPLAY)

if cfg.CHECK_FINAL_MOTION :
    from mlp.utils import check_path
    print "## Begin validation of the final motion (collision and joint-limits)"
    validator = check_path.PathChecker(v,cp.fullBody,cs_com,res.nq,True)
    motion_valid,t_invalid = validator.check_motion(res.q_t)
    print "## Check final motion, valid = ",motion_valid
    if not motion_valid:
        print "## First invalid time : ",t_invalid
else :
    motion_valid = True

if cfg.PLOT:
    from mlp.utils import plot
    plot.plotALLFromWB(cs_com,res)

if cfg.EXPORT_OPENHRP and motion_valid:
    from mlp.export import openHRP
    openHRP.export(cs_com,res)
if cfg.EXPORT_GAZEBO and motion_valid:
    from mlp.export import gazebo
    gazebo.export(res.q_t)
if cfg.EXPORT_NPZ and motion_valid :
    res.exportNPZ(cfg.EXPORT_PATH+"/npz",cfg.DEMO_NAME+".npz")


def dispCS(step = 0.2): 
    rbprmDisplay.displayContactSequence(v,cp.configs,step)
    
def dispWB():
    display_tools.displayWBmotion(v,res.q_t,cfg.IK_dt,cfg.DT_DISPLAY)
 
"""
#record gepetto-viewer 
v.startCapture("capture/capture","png")
dispWB()
v.stopCapture()


"""