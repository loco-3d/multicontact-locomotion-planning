import pinocchio as pin
from pinocchio import SE3, Quaternion
import time
from rospkg import RosPack
import gepetto.corbaserver
from mlp.utils.util import numpy2DToList, hppConfigFromMatrice, discretizeCurve, build_fullbody
from mlp.utils.requirements import Requirements
pin.switchToNumpyArray()


class DisplayContactSequenceRequirements(Requirements):
    configurationValues = True

STONE_HEIGHT = 0.005
STONE_GROUP = "stepping_stones"
STONE_RF = STONE_GROUP + "/" + "RF"
STONE_LF = STONE_GROUP + "/" + "LF"
STONE_RH = STONE_GROUP + "/" + "RH"
STONE_LH = STONE_GROUP + "/" + "LH"
TRAJ_GROUP = "com_traj"


def displaySphere(viewer, pos, size=0.01, color=[0, 0, 0, 1]):
    rootName = "s"
    # add indices until the name is free
    node_list = viewer.client.gui.getNodeList()
    if node_list:
        i = 0
        name = rootName
        while node_list.count(name) > 0:
            name = rootName + "_" + str(i)
            i += 1
        viewer.client.gui.addSphere(name, size, color)
        viewer.client.gui.addToGroup(name, viewer.sceneName)
        #viewer.client.gui.setVisibility(name,'ALWAYS_ON_TOP')
        q = pos + [0, 0, 0, 1]
        viewer.client.gui.applyConfiguration(name, q)
        viewer.client.gui.refresh()


def SE3ToViewerConfig(placement):
    q = [0] * 7
    q[0:3] = placement.translation.tolist()
    r = Quaternion(placement.rotation)
    q[6] = r.w
    q[3:6] = r.coeffs().tolist()[0:3]
    return q


def addContactLandmark(M, color, v):
    global i_sphere
    gui = v.client.gui
    name = 's' + str(i_sphere)
    i_sphere += 1
    gui.addSphere(name, 0.01, color)
    #gui.setVisibility(name,"ALWAYS_ON_TOP")
    gui.addToGroup(name, viewer.sceneName)
    gui.applyConfiguration(name, SE3ToViewerConfig(M))
    gui.addLandmark(name, 0.03)
    #print "contact altitude : "+str(p[2])


def displayContactsLandmarkFromPhase(phase, viewer, Robot):
    if phase.LF_patch.active:
        addContactLandmark(phase.LF_patch.plact * Robot.MLsole_display, Robot.dict_limb_color_traj[Robot.lfoot],
                           viewer)
    if phase.RF_patch.active:
        addContactLandmark(phase.RF_patch.placement * Robot.MRsole_display, Robot.dict_limb_color_traj[Robot.rfoot],
                           viewer)
    if phase.LH_patch.active:
        addContactLandmark(phase.LH_patch.placement * Robot.MLhand_display, Robot.dict_limb_color_traj[Robot.lhand],
                           viewer)
    if phase.RH_patch.active:
        addContactLandmark(phase.RH_patch.placement * Robot.MRhand_display, Robot.dict_limb_color_traj[Robot.rhand],
                           viewer)
    viewer.client.gui.refresh()


def addSteppingStone(gui, placement, name, group, size, color):
    gui.addBox(name, size[0], size[1], STONE_HEIGHT, color)
    gui.addToGroup(name, group)
    gui.applyConfiguration(name, SE3ToViewerConfig(placement))


def displaySteppingStones(cs, gui, sceneName, Robot):
    gui.createGroup(STONE_GROUP)
    gui.createGroup(STONE_RF)
    gui.createGroup(STONE_LF)
    gui.createGroup(STONE_RH)
    gui.createGroup(STONE_LH)
    name_RF = STONE_RF + "/stone_"
    name_LF = STONE_LF + "/stone_"
    name_RH = STONE_RH + "/stone_"
    name_LH = STONE_LH + "/stone_"
    id_RF = 0
    id_LF = 0
    id_RH = 0
    id_LH = 0

    for phase in cs.contactPhases:
        if phase.isEffectorInContact(Robot.lfoot):
            addSteppingStone(gui, phase.contactPatch(Robot.lfoot).placement * Robot.dict_display_offset[Robot.lfoot],
                             name_LF + str(id_LF), STONE_LF, Robot.dict_size[Robot.lfoot],
                             Robot.dict_limb_color_traj[Robot.lfoot])
            id_LF += 1
        if phase.isEffectorInContact(Robot.rfoot):
            addSteppingStone(gui, phase.contactPatch(Robot.rfoot).placement * Robot.dict_display_offset[Robot.rfoot],
                             name_RF + str(id_RF), STONE_RF, Robot.dict_size[Robot.rfoot],
                             Robot.dict_limb_color_traj[Robot.rfoot])
            id_RF += 1
        if phase.isEffectorInContact(Robot.lhand):
            addSteppingStone(gui, phase.contactPatch(Robot.lhand).placement * Robot.dict_display_offset[Robot.lhand],
                             name_LH + str(id_LH), STONE_LH, Robot.dict_size[Robot.lhand],
                             Robot.dict_limb_color_traj[Robot.lhand])
            id_LH += 1
        if phase.isEffectorInContact(Robot.rhand):
            addSteppingStone(gui, phase.contactPatch(Robot.rhand).placement * Robot.dict_display_offset[Robot.rhand],
                             name_RH + str(id_RH), STONE_RH, Robot.dict_size[Robot.rhand],
                             Robot.dict_limb_color_traj[Robot.rhand])
            id_RH += 1

    gui.addToGroup(STONE_RF, STONE_GROUP)
    gui.addToGroup(STONE_LF, STONE_GROUP)
    gui.addToGroup(STONE_RH, STONE_GROUP)
    gui.addToGroup(STONE_LH, STONE_GROUP)
    gui.addToGroup(STONE_GROUP, sceneName)
    gui.refresh()



def displayCOMTrajForPhase(phase, gui, name, name_group, color, dt):
    c = numpy2DToList(discretizeCurve(phase.c_t, dt)[0])
    gui.addCurve(name, c, color)
    gui.addToGroup(name, name_group)


def displayCOMTrajectory(cs, gui, sceneName, dt, colors=[0, 0, 0, 1], nameGroup=""):
    name_group = TRAJ_GROUP + nameGroup
    gui.createGroup(name_group)
    for pid,phase in enumerate(cs.contactPhases):
        name = name_group + "/" + '%.2f' % phase.timeInitial + "-" + '%.2f' % phase.timeFinal
        color = colors[pid % len(colors)]
        displayCOMTrajForPhase(phase, gui, name, name_group, color, dt)
    gui.addToGroup(name_group, sceneName)
    gui.refresh()


def displaySE3Traj(traj, gui, sceneName, name, color, time_interval, offset=SE3.Identity()):
    if name == None:
        name = "SE3_traj"
    rootName = name
    # add indices until the name is free
    node_list = gui.getNodeList()
    if node_list:
        i = 0
        while node_list.count(name) > 0:
            name = rootName + "_" + str(i)
            i += 1
        path = []
        dt = 0.01
        t = time_interval[0]
        while t <= time_interval[1]:
            m = traj.evaluateAsSE3(t)
            m = m.act(offset)
            path += [m.translation.tolist()]
            t += dt
        gui.addCurve(name, path, color)
        gui.addToGroup(name, sceneName)
        gui.refresh()

def displayEffectorTrajectories(cs, viewer, Robot, suffixe = "", colorAlpha = 1):
    effectors = cs.getAllEffectorsInContact()
    for pid,phase in enumerate(cs.contactPhases):
        for eeName in effectors:
            if phase.effectorHaveAtrajectory(eeName):
                color = Robot.dict_limb_color_traj[eeName]
                color[-1] = colorAlpha
                displaySE3Traj(phase.effectorTrajectory(eeName), viewer.client.gui, viewer.sceneName,
                                 eeName + "_traj_"+ suffixe + str(pid), color,
                                 [phase.timeInitial, phase.timeFinal], Robot.dict_offset[eeName])

def displayWBconfig(viewer, q_matrix):
    viewer(hppConfigFromMatrice(viewer.robot, q_matrix))


def displayWBatT(viewer, cs_wb, t):
    q = cs_wb.phaseAtTime(t).q_t(t)
    viewer(hppConfigFromMatrice(viewer.robot,q))


def displayWBmotion(viewer, q_t, dt_display):
    t = q_t.min()
    while t <= q_t.max():
        t_start = time.time()
        displayWBconfig(viewer, q_t(t))
        t += dt_display
        elapsed = time.time() - t_start
        if elapsed > dt_display:
            print("Warning : display not real time ! choose a greater time step for the display.")
        else:
            time.sleep(dt_display - elapsed)
    # display last config if the total duration is not a multiple of the dt
    displayWBconfig(viewer, q_t(q_t.max()))


def displayFeetTrajFromResult(gui, sceneName, res, Robot):
    for eeName in res.eeNames:
        name = "feet_traj_" + str(eeName)
        offset = Robot.dict_offset[eeName].translation
        traj = res.effector_references[eeName][:3, :].copy()
        for i in range(traj.shape[1]):
            traj[:, i] += offset
        traj = numpy2DToList(traj)
        color = Robot.dict_limb_color_traj[eeName]
        gui.addCurve(name, traj, color)
        gui.addToGroup(name, sceneName)
        gui.refresh()


def displayContactSequence(v, cs, step=0.2):
    for p in cs.contactPhases:
        displayWBconfig(v, p.q_init)
        time.sleep(step)
    displayWBconfig(v, cs.contactPhases[-1].q_final)


def initScene(Robot, envName="multicontact/ground", genLimbsDB=True, context = None):
    from hpp.gepetto import ViewerFactory
    fullBody, ps = build_fullbody(Robot, genLimbsDB, context)
    vf = ViewerFactory(ps)
    vf.loadObstacleModel("package://hpp_environments/urdf/" + envName + ".urdf", "planning")
    v = vf.createViewer(ghost = True, displayCoM=True)
    v(fullBody.getCurrentConfig())
    return fullBody, v


def initScenePinocchio(urdf_name, package_name, env_name=None, env_package_name="hpp_environments", scene_name = "world"):
    rp = RosPack()
    package_path = rp.get_path(package_name)
    urdf = package_path + '/urdf/' + urdf_name + '.urdf'
    robot = pin.RobotWrapper.BuildFromURDF(urdf, package_path, pin.JointModelFreeFlyer(),)
    robot.initDisplay(loadModel=True)
    robot.displayCollisions(False)
    robot.displayVisuals(True)
    #robot.display(robot.model.neutralConfiguration)

    cl = gepetto.corbaserver.Client()
    gui = cl.gui
    if env_name:
        urdfEnvPath = rp.get_path(env_package_name)
        urdfEnv = urdfEnvPath + '/urdf/' + env_name + '.urdf'
        gui.addUrdfObjects(scene_name + "/environments", urdfEnv, True)
    return robot, gui


def disp_wb_pinocchio(robot, q_t, dt_display = 0.04):
    t = q_t.min()
    while t < q_t.max():
        t_start = time.time()
        robot.display(q_t(t))
        elapsed = time.time() - t_start
        left = q_t.max() - t
        if dt_display > left:
            dt_display = left
        if elapsed > dt_display:
            print("Warning : display not real time ! choose a greater time step for the display.")
        else:
            time.sleep(dt_display - elapsed)
        t += dt_display
    # display last config if the total duration is not a multiple of the dt
    robot.display(q_t(q_t.max()))



