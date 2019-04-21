import mlp.config as cfg
import mlp.viewer.display_tools as display_tools

print "### MLP : contact sequence ###"
from mlp.contact_sequence import generateContactSequence
cs,fullBody,viewer = generateContactSequence()

if cfg.DISPLAY_CS:
    raw_input("Press Enter to display the contact sequence ...")
    display_tools.displayContactSequence(viewer,cs,step)    
if cfg.SAVE_CS:
    filename = cfg.CONTACT_SEQUENCE_PATH + "/"+cfg.DEMO_NAME+".cs"
    print "Write contact sequence binary file : ",filename
    cs.saveAsBinary(filename)    
if cfg.DISPLAY_CS_STONES :
    display_tools.displaySteppingStones(cs,viewer)
    

print "------------------------------"
print "### MLP : centroidal, initial Guess ###"
from mlp.centroidal import generateCentroidalInitGuess
cs_initGuess = generateCentroidalInitGuess(cs,fullBody=fullBody,viewer=viewer)

if cfg.DISPLAY_INIT_GUESS_TRAJ and cs_initGuess:
    colors = [viewer.color.red, viewer.color.yellow]
    display_tools.displayCOMTrajectory(cs_initGuess,viewer,colors,"_init")    

print "------------------------------"
print "### MLP : centroidal  ###"
from mlp.centroidal import generateCentroidalTrajectory
cs_com = generateCentroidalTrajectory(cs,cs_initGuess,fullBody,viewer)

if cfg.SAVE_CS_COM:
    filename = cfg.CONTACT_SEQUENCE_PATH + "/"+cfg.DEMO_NAME+"_COM.cs"
    print "Write contact sequence binary file with centroidal trajectory : ",filename
    cs_com.saveAsBinary(filename) 
if cfg.DISPLAY_COM_TRAJ:
    colors = [viewer.color.blue, viewer.color.green]
    display_tools.displayCOMTrajectory(cs_com,viewer,colors)
    
print "------------------------------"
print "### MLP : whole-body  ###"
from mlp.wholebody import generateWholeBodyMotion
res,robot = generateWholeBodyMotion(cs_com,fullBody,viewer)

if cfg.CHECK_FINAL_MOTION :
    from mlp.utils import check_path
    print "## Begin validation of the final motion (collision and joint-limits)"
    validator = check_path.PathChecker(viewer,fullBody,cs_com,res.nq,True)
    motion_valid,t_invalid = validator.check_motion(res.q_t)
    print "## Check final motion, valid = ",motion_valid
    if not motion_valid:
        print "## First invalid time : ",t_invalid
else :
    motion_valid = True
if cfg.DISPLAY_WB_MOTION:
    raw_input("Press Enter to display the whole body motion ...")
    display_tools.displayWBmotion(viewer,res.q_t,cfg.IK_dt,cfg.DT_DISPLAY)

if cfg.PLOT:
    from mlp.utils import plot
    plot.plotKneeTorque(res.t_t,res.phases_intervals,res.tau_t,6 + (res.nq - res.nv),cfg.Robot.kneeIds)    
    plot.plotALLFromWB(cs_com,res,cfg.DISPLAY_PLOT,cfg.SAVE_PLOT,cfg.OUTPUT_DIR+"/plot/"+cfg.DEMO_NAME)
    
if cfg.EXPORT_OPENHRP and motion_valid:
    from mlp.export import openHRP
    openHRP.export(cs_com,res)
if cfg.EXPORT_GAZEBO and motion_valid:
    from mlp.export import gazebo
    gazebo.export(res.q_t)
if cfg.EXPORT_NPZ and motion_valid :
    res.exportNPZ(cfg.EXPORT_PATH+"/npz",cfg.DEMO_NAME+".npz")
if cfg.EXPORT_BLENDER:
    from mlp.export import blender
    blender.export(res.q_t,viewer)

def dispCS(step = 0.2): 
    display_tools.displayContactSequence(viewer,cs,step)
    
def dispWB():
    display_tools.displayWBmotion(viewer,res.q_t,cfg.IK_dt,cfg.DT_DISPLAY)
 
"""
#record gepetto-viewer 
viewer.startCapture("capture/capture","png")
dispWB()
viewer.stopCapture()


"""