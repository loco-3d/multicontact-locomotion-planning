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
print("### MLP : End effector initial Guess  ###")
import mlp.end_effector.initGuess as effectorsInitGuess
if not effectorsInitGuess.Inputs.checkAndFillRequirements(cs_com, cfg, fullBody):
    raise RuntimeError("The current contact sequence cannot be given as input to the end effector method selected.")
cs_ref = effectorsInitGuess.generateEffectorTrajectoriesForSequence(cs_com, fullBody)
effectorsInitGuess.Outputs.assertRequirements(cs_ref)

if cfg.SAVE_CS_REF:
    if not os.path.exists(cfg.CONTACT_SEQUENCE_PATH):
        os.makedirs(cfg.CONTACT_SEQUENCE_PATH)
    filename = cfg.CONTACT_SEQUENCE_PATH + "/" + cfg.DEMO_NAME + "_REF.cs"
    print("Write contact sequence binary file with centroidal and end effector trajectories: ", filename)
    cs_ref.saveAsBinary(filename)

if cfg.DISPLAY_ALL_FEET_TRAJ:
    from mlp.viewer.display_tools import displayEffectorTrajectories
    displayEffectorTrajectories(cs_ref, viewer, fullBody, "_ref", 0.6)


print("------------------------------")
print("### MLP : whole-body  ###")
import mlp.wholebody as wholeBody
if not wholeBody.Inputs.checkAndFillRequirements(cs_ref,cfg,fullBody):
    raise RuntimeError("The current contact sequence cannot be given as input to the wholeBody method selected.")
cs_wb, robot = wholeBody.generateWholeBodyMotion(cs_ref, fullBody, viewer)
wholeBody.Outputs.assertRequirements(cs_wb)

if cfg.SAVE_CS_REF:
    if not os.path.exists(cfg.CONTACT_SEQUENCE_PATH):
        os.makedirs(cfg.CONTACT_SEQUENCE_PATH)
    filename = cfg.CONTACT_SEQUENCE_PATH + "/" + cfg.DEMO_NAME + "_REF.cs"
    print("Write contact sequence binary file with centroidal and end effector trajectories: ", filename)
    cs_ref.saveAsBinary(filename)

if cfg.SAVE_CS_WB:
    if not os.path.exists(cfg.CONTACT_SEQUENCE_PATH):
        os.makedirs(cfg.CONTACT_SEQUENCE_PATH)
    filename = cfg.CONTACT_SEQUENCE_PATH + "/" + cfg.DEMO_NAME + "_WB.cs"
    print("Write contact sequence binary file with wholebody trajectories: ", filename)
    cs_wb.saveAsBinary(filename)


if cfg.WRITE_STATUS:
    if not os.path.exists(cfg.OUTPUT_DIR):
        os.makedirs(cfg.OUTPUT_DIR)
    f = open(cfg.STATUS_FILENAME, "a")
    f.write("wholebody_success: True\n")
    if cs_wb.size() == cs_ref.size() :
        f.write("wholebody_reach_goal: True\n")
    else:
        f.write("wholebody_reach_goal: False\n")
    f.close()

if cfg.DISPLAY_FEET_TRAJ:
    from mlp.viewer.display_tools import displayEffectorTrajectories
    if cfg.IK_store_effector:
        displayEffectorTrajectories(cs_wb, viewer, fullBody)
    else :
        displayEffectorTrajectories(cs_ref, viewer, fullBody)


if cfg.CHECK_FINAL_MOTION:
    from mlp.utils import check_path
    print("## Begin validation of the final motion (collision and joint-limits)")
    validator = check_path.PathChecker(fullBody, cfg.CHECK_DT, True)
    motion_valid, t_invalid = validator.check_motion(cs_wb.concatenateQtrajectories())
    print("## Check final motion, valid = ", motion_valid)
    if not motion_valid:
        print("## First invalid time : ", t_invalid)
    if cfg.WRITE_STATUS:
        f = open(cfg.STATUS_FILENAME, "a")
        f.write("motion_valid: " + str(motion_valid) + "\n")
        f.close()
elif cs_wb is not None:
    motion_valid = True
else:
    motion_valid = False
if cfg.DISPLAY_WB_MOTION:
    input("Press Enter to display the whole body motion ...")
    display_tools.displayWBmotion(viewer, cs_wb.concatenateQtrajectories(), cfg.DT_DISPLAY)

if cfg.PLOT:
    from mlp.utils import plot
    plot.plotALLFromWB(cs_ref, cs_wb, cfg.DISPLAY_PLOT, cfg.SAVE_PLOT, cfg.OUTPUT_DIR + "/plot/" + cfg.DEMO_NAME)

if cfg.EXPORT_OPENHRP and motion_valid:
    from mlp.export import openHRP
    openHRP.export(cs_com, cs_wb) # FIXME
if cfg.EXPORT_GAZEBO and motion_valid:
    from mlp.export import gazebo
    gazebo.export(cs_wb.concatenateQtrajectories())
#if cfg.EXPORT_NPZ and motion_valid:
#    res.exportNPZ(cfg.EXPORT_PATH + "/npz", cfg.DEMO_NAME + ".npz") # TODO
if cfg.EXPORT_BLENDER:
    from mlp.export import blender
    blender.export(cs_wb.concatenateQtrajectories(), viewer, cfg.IK_dt)
    blender.exportSteppingStones(viewer)
if cfg.EXPORT_SOT:
    from mlp.export import sotTalosBalance
    sotTalosBalance.export(cs_wb) # TODO



def dispCS(step=0.2):
    display_tools.displayContactSequence(viewer, cs, step)


def dispWB(t=None):
    if t is None:
        display_tools.displayWBmotion(viewer,cs_wb.concatenateQtrajectories(), cfg.DT_DISPLAY)
    else:
        display_tools.displayWBatT(viewer, cs_wb, t)


"""
#record gepetto-viewer 
viewer.startCapture("capture/capture","png")
dispWB()
viewer.stopCapture()


"""
