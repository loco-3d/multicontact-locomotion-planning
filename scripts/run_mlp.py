import mlp.config as cfg
import mlp.viewer.display_tools as display_tools
from multicontact_api import ContactSequenceHumanoid


if cfg.LOAD_CS:
    import mlp.contact_sequence.fromCSfile as gen_cs
else:
    import mlp.contact_sequence.rbprm as gen_cs

cs,fullBody,v = gen_cs.generateContactSequence()



if cfg.DISPLAY_CS:
    raw_input("Press Enter to display the contact sequence ...")
    display_tools.displayContactSequence(v,cs,step)
    
if cfg.SAVE_CS and not cfg.LOAD_CS:
    filename = cfg.CONTACT_SEQUENCE_PATH + "/"+cfg.DEMO_NAME
    print "Write contact sequence binary file : ",filename
    cs.saveAsBinary(filename)    
if cfg.DISPLAY_CS_STONES :
    display_tools.displaySteppingStones(cs,v)
    
if cfg.USE_GEOM_INIT_GUESS:
    print "Generate geometric init guess."
    import mlp.centroidal.geometric as initGuess_geom 
    cs_initGuess = initGuess_geom.generateCentroidalTrajectory(cs)
if cfg.USE_CROC_INIT_GUESS:
    print "Generate init guess with CROC."
    import mlp.centroidal.croc as initGuess_croc    
    cs_initGuess = initGuess_croc.generateCentroidalTrajectory(cs,fullBody,beginState,endState)
if cfg.DISPLAY_INIT_GUESS_TRAJ and (cfg.USE_GEOM_INIT_GUESS or cfg.USE_CROC_INIT_GUESS):
    colors = [v.color.red, v.color.yellow]
    display_tools.displayCOMTrajectory(cs_initGuess,v,colors,"_init")

if cfg.LOAD_CS_COM :
    cs_com = ContactSequenceHumanoid(0)
    filename = cfg.CONTACT_SEQUENCE_PATH + "/"+cfg.DEMO_NAME+"_COM"
    cs_com.loadFromBinary(filename)     
else:
    import mlp.centroidal.topt as centroidal
    cs_com,tp = centroidal.generateCentroidalTrajectory(cs,cs_initGuess,v)
    print "Duration of the motion : "+str(cs_com.contact_phases[-1].time_trajectory[-1])+" s."


if cfg.SAVE_CS_COM and not cfg.LOAD_CS_COM:
    filename = cfg.CONTACT_SEQUENCE_PATH + "/"+cfg.DEMO_NAME+"_COM.xml"
    print "Write contact sequence with centroidal trajectory XML file : ",filename
    cs_com.saveAsBinary(filename) 
if cfg.DISPLAY_COM_TRAJ:
    colors = [v.color.blue, v.color.green]
    display_tools.displayCOMTrajectory(cs_com,v,colors)

import mlp.wholebody.tsid_invdyn as wb
if cfg.USE_CROC_COM:
    assert cfg.USE_CROC_INIT_GUESS, "You must generate CROC initial guess if you want to use it as reference for the COM"  
    res,robot = wb.generateWholeBodyMotion(cs_initGuess,v,fullBody)
else : 
    #q_t,v_t,a_t = wb.generateWholeBodyMotion(cs_com,v,fullBody)
    res,robot =  wb.generateWholeBodyMotion(cs_com,v,fullBody)

if cfg.DISPLAY_WB_MOTION:
    raw_input("Press Enter to display the whole body motion ...")
    display_tools.displayWBmotion(v,res.q_t,cfg.IK_dt,cfg.DT_DISPLAY)

if cfg.CHECK_FINAL_MOTION :
    from mlp.utils import check_path
    print "## Begin validation of the final motion (collision and joint-limits)"
    validator = check_path.PathChecker(v,fullBody,cs_com,res.nq,True)
    motion_valid,t_invalid = validator.check_motion(res.q_t)
    print "## Check final motion, valid = ",motion_valid
    if not motion_valid:
        print "## First invalid time : ",t_invalid
else :
    motion_valid = True

if cfg.PLOT:
    from mlp.utils import plot
    plot.plotALLFromWB(cs_com,res,cfg.DISPLAY_PLOT,cfg.SAVE_PLOT,cfg.OUTPUT_DIR+"/plot/"+cfg.DEMO_NAME)
    plot.plotKneeTorque(res.t_t,res.phases_intervals,res.tau_t,6 + (res.nq - res.nv),cfg.Robot.kneeIds)
    
if cfg.EXPORT_OPENHRP and motion_valid:
    from mlp.export import openHRP
    openHRP.export(cs_com,res)
if cfg.EXPORT_GAZEBO and motion_valid:
    from mlp.export import gazebo
    gazebo.export(res.q_t)
if cfg.EXPORT_NPZ and motion_valid :
    res.exportNPZ(cfg.EXPORT_PATH+"/npz",cfg.DEMO_NAME+".npz")


def dispCS(step = 0.2): 
    display_tools.displayContactSequence(v,cs,step)
    
def dispWB():
    display_tools.displayWBmotion(v,res.q_t,cfg.IK_dt,cfg.DT_DISPLAY)
 
"""
#record gepetto-viewer 
v.startCapture("capture/capture","png")
dispWB()
v.stopCapture()


"""