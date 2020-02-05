import mlp.config as cfg
import mlp.viewer.display_tools as display_tools
import os
import eigenpy
import pinocchio
import curves
import multicontact_api
eigenpy.switchToNumpyArray()

try:
    #python2
    input = raw_input
except NameError:
    pass


print("### MLP : contact sequence ###")
import mlp.contact_sequence as contactGeneration
cs, fullBody, viewer = contactGeneration.generateContactSequence()
contactGeneration.Outputs.assertRequirements(cs)
gui = viewer.client.gui


if cfg.WRITE_STATUS:
    f = open(cfg.STATUS_FILENAME, "a")
    f.write("gen_cs_success: True\n")
    f.close()

if cfg.DISPLAY_CS_STONES:
    display_tools.displaySteppingStones(cs, gui, viewer.sceneName, cfg.Robot)
if cfg.DISPLAY_CS:
    if display_tools.DisplayContactSequenceRequirements.checkAndFillRequirements(cs, cfg, fullBody):
        input("Press Enter to display the contact sequence ...")
        display_tools.displayContactSequence(viewer, cs)
if cfg.SAVE_CS:
    if not os.path.exists(cfg.CONTACT_SEQUENCE_PATH):
        os.makedirs(cfg.CONTACT_SEQUENCE_PATH)
    filename = cfg.CONTACT_SEQUENCE_PATH + "/" + cfg.DEMO_NAME + ".cs"
    print("Write contact sequence binary file : ", filename)
    cs.saveAsBinary(filename)


print("------------------------------")
print("### MLP : centroidal, initial Guess ###")
import mlp.centroidal.initGuess as centroidalInitGuess
if not centroidalInitGuess.Inputs.checkAndFillRequirements(cs,cfg, fullBody):
    raise RuntimeError("The current contact sequence cannot be given as input to the centroidalInitGuess method selected.")
cs_initGuess = centroidalInitGuess.generateCentroidalTrajectory(cs, fullBody=fullBody, viewer=viewer)
centroidalInitGuess.Outputs.assertRequirements(cs_initGuess)

if cfg.DISPLAY_INIT_GUESS_TRAJ and cs_initGuess:
    colors = [viewer.color.red, viewer.color.yellow]
    display_tools.displayCOMTrajectory(cs_initGuess, gui, viewer.sceneName, cfg.DT_DISPLAY, colors, "_init")

print("------------------------------")
print("### MLP : centroidal  ###")
import mlp.centroidal as centroidal
if not centroidal.Inputs.checkAndFillRequirements(cs,cfg,fullBody):
    raise RuntimeError("The current contact sequence cannot be given as input to the centroidal method selected.")
cs_com = centroidal.generateCentroidalTrajectory(cs, cs_initGuess, fullBody, viewer)
centroidal.Outputs.assertRequirements(cs_com)

if cfg.WRITE_STATUS:
    f = open(cfg.STATUS_FILENAME, "a")
    f.write("centroidal_success: True\n")
    f.close()

if cfg.SAVE_CS_COM:
    if not os.path.exists(cfg.CONTACT_SEQUENCE_PATH):
        os.makedirs(cfg.CONTACT_SEQUENCE_PATH)
    filename = cfg.CONTACT_SEQUENCE_PATH + "/" + cfg.DEMO_NAME + "_COM.cs"
    print("Write contact sequence binary file with centroidal trajectory : ", filename)
    cs_com.saveAsBinary(filename)
if cfg.DISPLAY_COM_TRAJ:
    colors = [viewer.color.blue, viewer.color.green]
    display_tools.displayCOMTrajectory(cs_com, gui, viewer.sceneName, cfg.DT_DISPLAY, colors)
if cfg.PLOT_CENTROIDAL:
    from mlp.utils.plot import plotCOMTrajFromCS
    plotCOMTrajFromCS(cs_com)

print("------------------------------")
print("### MLP : whole-body  ###")
import mlp.wholebody as wholeBody
if not wholeBody.Inputs.checkAndFillRequirements(cs_com,cfg,fullBody):
    raise RuntimeError("The current contact sequence cannot be given as input to the wholeBody method selected.")
cs_wb, res, robot = wholeBody.generateWholeBodyMotion(cs_com, fullBody, viewer)
wholeBody.Outputs.assertRequirements(cs_wb)

if cfg.WRITE_STATUS:
    if not os.path.exists(cfg.OUTPUT_DIR):
        os.makedirs(cfg.OUTPUT_DIR)
    f = open(cfg.STATUS_FILENAME, "a")
    f.write("wholebody_success: True\n")
    if res.N == (int(round(cs_com.contact_phases[-1].time_trajectory[-1] / cfg.IK_dt)) + 1):
        f.write("wholebody_reach_goal: True\n")
    else:
        f.write("wholebody_reach_goal: False\n")
    f.close()

if cfg.CHECK_FINAL_MOTION:
    from mlp.utils import check_path
    print("## Begin validation of the final motion (collision and joint-limits)")
    validator = check_path.PathChecker(fullBody, cs_com, res.nq, True)
    motion_valid, t_invalid = validator.check_motion(res.q_t)
    print("## Check final motion, valid = ", motion_valid)
    if not motion_valid:
        print("## First invalid time : ", t_invalid)
    if cfg.WRITE_STATUS:
        f = open(cfg.STATUS_FILENAME, "a")
        f.write("motion_valid: " + str(motion_valid) + "\n")
        f.close()
elif res:
    motion_valid = True
else:
    motion_valid = False
if cfg.DISPLAY_WB_MOTION:
    input("Press Enter to display the whole body motion ...")
    display_tools.displayWBmotion(viewer, res.q_t, cfg.IK_dt, cfg.DT_DISPLAY)

if cfg.PLOT:
    from mlp.utils import plot
    plot.plotKneeTorque(res.t_t, res.phases_intervals, res.tau_t, (res.nq - res.nu), cfg.Robot.kneeIds)
    plot.plotALLFromWB(cs_com, res, cfg.DISPLAY_PLOT, cfg.SAVE_PLOT, cfg.OUTPUT_DIR + "/plot/" + cfg.DEMO_NAME)

if cfg.EXPORT_OPENHRP and motion_valid:
    from mlp.export import openHRP
    openHRP.export(cs_com, res)
if cfg.EXPORT_GAZEBO and motion_valid:
    from mlp.export import gazebo
    gazebo.export(res.q_t)
if cfg.EXPORT_NPZ and motion_valid:
    res.exportNPZ(cfg.EXPORT_PATH + "/npz", cfg.DEMO_NAME + ".npz")
if cfg.EXPORT_BLENDER:
    from mlp.export import blender
    blender.export(res.q_t, viewer, cfg.IK_dt)
    blender.exportSteppingStones(viewer)
if cfg.EXPORT_SOT:
    from mlp.export import sotTalosBalance
    sotTalosBalance.export(res)



def dispCS(step=0.2):
    display_tools.displayContactSequence(viewer, cs, step)


def dispWB(t=None):
    if t is None:
        display_tools.displayWBmotion(viewer, res.q_t, cfg.IK_dt, cfg.DT_DISPLAY)
    else:
        display_tools.displayWBatT(viewer, res, t)


"""
#record gepetto-viewer 
viewer.startCapture("capture/capture","png")
dispWB()
viewer.stopCapture()


"""
