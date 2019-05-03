import mlp.config as cfg
import mlp.viewer.display_tools as display_tools

print "### MLP : contact sequence ###"
from mlp.contact_sequence import generateContactSequence
cs,fullBody,viewer = generateContactSequence()

if cfg.WRITE_STATUS:
    f = open(cfg.STATUS_FILENAME,"a")
    f.write("gen_cs_success: True\n")
    f.close()
       
if cfg.DISPLAY_CS:
    raw_input("Press Enter to display the contact sequence ...")
    display_tools.displayContactSequence(viewer,cs,step)    
if cfg.SAVE_CS:
    filename = cfg.CONTACT_SEQUENCE_PATH + "/"+cfg.DEMO_NAME+".cs"
    print "Write contact sequence binary file : ",filename
    cs.saveAsBinary(filename)    
if cfg.DISPLAY_CS_STONES :
    display_tools.displaySteppingStones(cs,viewer.client.gui,viewer.sceneName,cfg.Robot)
    

print "------------------------------"
print "### MLP : centroidal, initial Guess ###"
from mlp.centroidal import generateCentroidalInitGuess
cs_initGuess = generateCentroidalInitGuess(cs,fullBody=fullBody,viewer=viewer)

if cfg.DISPLAY_INIT_GUESS_TRAJ and cs_initGuess:
    colors = [viewer.color.red, viewer.color.yellow]
    display_tools.displayCOMTrajectory(cs_initGuess,viewer.client.gui,viewer.sceneName,colors,"_init")    

print "------------------------------"
print "### MLP : centroidal  ###"
from mlp.centroidal import generateCentroidalTrajectory
cs_com = generateCentroidalTrajectory(cs,cs_initGuess,fullBody,viewer)

if cfg.WRITE_STATUS:
    f = open(cfg.STATUS_FILENAME,"a")
    f.write("centroidal_success: True\n")
    f.close()

if cfg.SAVE_CS_COM:
    filename = cfg.CONTACT_SEQUENCE_PATH + "/"+cfg.DEMO_NAME+"_COM.cs"
    print "Write contact sequence binary file with centroidal trajectory : ",filename
    cs_com.saveAsBinary(filename) 
if cfg.DISPLAY_COM_TRAJ:
    colors = [viewer.color.blue, viewer.color.green]
    display_tools.displayCOMTrajectory(cs_com,viewer.client.gui,viewer.sceneName,colors)
    
print "------------------------------"
print "### MLP : whole-body  ###"
from mlp.wholebody import generateWholeBodyMotion
res,robot = generateWholeBodyMotion(cs_com,fullBody,viewer)

if cfg.WRITE_STATUS:
    f = open(cfg.STATUS_FILENAME,"a")
    f.write("wholebody_success: True\n")
    if res.N == (int(round(cs_com.contact_phases[-1].time_trajectory[-1]/cfg.IK_dt)) + 1):
        f.write("wholebody_reach_goal: True\n")
    else : 
        f.write("wholebody_reach_goal: False\n")   
    f.close()


if cfg.CHECK_FINAL_MOTION :
    from mlp.utils import check_path
    print "## Begin validation of the final motion (collision and joint-limits)"
    validator = check_path.PathChecker(fullBody,cs_com,res.nq,True)
    motion_valid,t_invalid = validator.check_motion(res.q_t)
    print "## Check final motion, valid = ",motion_valid
    if not motion_valid:
        print "## First invalid time : ",t_invalid
    if cfg.WRITE_STATUS:
        f = open(cfg.STATUS_FILENAME,"a")
        f.write("motion_valid: "+str(motion_valid)+"\n")
        f.close() 
else :
    motion_valid = True
if cfg.DISPLAY_WB_MOTION:
    raw_input("Press Enter to display the whole body motion ...")
    display_tools.displayWBmotion(viewer,res.q_t,cfg.IK_dt,cfg.DT_DISPLAY)

if cfg.PLOT:
    from mlp.utils import plot
    plot.plotKneeTorque(res.t_t,res.phases_intervals,res.tau_t,(res.nq - res.nu),cfg.Robot.kneeIds)    
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
    blender.export(res.q_t,viewer,cfg.IK_dt)

def dispCS(step = 0.2): 
    display_tools.displayContactSequence(viewer,cs,step)
    
def dispWB(t=None):
    if t is None:
        display_tools.displayWBmotion(viewer,res.q_t,cfg.IK_dt,cfg.DT_DISPLAY)
    else:
        display_tools.displayWBatT(viewer,res,t)

    
"""
#record gepetto-viewer 
viewer.startCapture("capture/capture","png")
dispWB()
viewer.stopCapture()


"""