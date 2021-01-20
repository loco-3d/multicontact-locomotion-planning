import numpy as np
import mlp.utils.wholebody_result as wb_res
import mlp.utils.plot as plot
import multicontact_api
from multicontact_api import ContactSequenceHumanoid
import time
import os
from subprocess import check_output
from mlp.utils.status import Status
import pinocchio as pin
from pinocchio import SE3
from mlp.viewer.display_tools import displayCOMTrajectory, displaySteppingStones, displayFeetTrajFromResult
from mlp.viewer.display_tools import initScenePinocchio



class Robot:  # data for talos (to avoid a depencie on talos-rbprm for this script)
    rfoot = 'leg_right_6_joint'
    lfoot = 'leg_left_6_joint'
    rhand = 'arm_right_7_joint'
    lhand = 'arm_left_7_joint'
    rLegOffset = [0., -0.00018, -0.102]
    lLegOffset = [0., -0.00018, -0.102]
    rArmOffset = [-0.01, 0., -0.154]
    lArmOffset = [-0.01, 0., -0.154]
    MRsole_offset = SE3.Identity()
    MRsole_offset.translation = np.matrix(rLegOffset).T
    MLsole_offset = SE3.Identity()
    MLsole_offset.translation = np.matrix(lLegOffset).T
    MRhand_offset = SE3.Identity()
    MRhand_offset.translation = np.matrix(rArmOffset).T
    MLhand_offset = SE3.Identity()
    MLhand_offset.translation = np.matrix(lArmOffset).T
    dict_offset = {rfoot: MRsole_offset, lfoot: MLsole_offset, rhand: MRhand_offset, lhand: MLhand_offset}
    MRsole_display = SE3.Identity()
    MLsole_display = SE3.Identity()
    MRhand_display = SE3.Identity()
    #MRhand_display.translation = np.matrix([0,  0., -0.11])
    MLhand_display = SE3.Identity()
    #MLhand_display.translation = np.matrix([0,  0., -0.11])
    dict_display_offset = {rfoot: MRsole_display, lfoot: MLsole_display, rhand: MRhand_display, lhand: MLhand_display}
    dict_limb_color_traj = {rfoot: [0, 1, 0, 1], lfoot: [1, 0, 0, 1], rhand: [0, 0, 1, 1], lhand: [0.9, 0.5, 0, 1]}
    dict_size = {rfoot: [0.2, 0.13], lfoot: [0.2, 0.13], rhand: [0.1, 0.1], lhand: [0.1, 0.1]}


def loadMotionFromFiles(gui, path, npzFilename, csFilename):
    # load cs from file :
    cs = ContactSequenceHumanoid(0)
    cs.loadFromBinary(path + csFilename)
    displaySteppingStones(cs, gui, sceneName, Robot)
    colors = [[0., 0., 1., 1.], [0., 1., 0., 1.]]
    displayCOMTrajectory(cs, gui, sceneName, colors)
    #extract data from npz archive :
    res = wb_res.loadFromNPZ(path + npzFilename)
    displayFeetTrajFromResult(gui, sceneName, res, Robot)
    plot.plotALLFromWB(cs, res)
    return res, cs


# example code :

urdfName = "talos_reduced"
packageName = "talos_data"
#envName = "multicontact/ground"
robot, gui = initScenePinocchio(urdfName, packageName)

path = "/local/dev_hpp/src/multicontact-locomotion-planning/res/"
npzFile = "export/npz/talos_circle.npz"
csFile = "contact_sequences/talos_circle_COM.cs"
res, cs = loadMotionFromFiles(gui, path, npzFile, csFile)
robot.display(res.q_t[:, 0])
status = Status(path + "/../scripts/infos.log")


def dispWB():
    dt_display = 0.04  # 25 fps
    step = dt_display / res.dt
    assert step % 1 == 0, "display dt shouldbe a multiple of ik dt"
    # check if robot have extradof :
    step = int(step)
    id = 0
    while id < res.q_t.shape[1]:
        t_start = time.time()
        robot.display(res.q_t[:, id])
        id += step
        elapsed = time.time() - t_start
        if elapsed > dt_display:
            print("Warning : display not real time ! choose a greater time step for the display.")
        else:
            time.sleep(dt_display - elapsed)
    # display last config if the total duration is not a multiple of the dt
    robot.display(res.q_t[:, -1])


dispWB()
