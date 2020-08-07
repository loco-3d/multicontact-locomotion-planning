import pinocchio as pin
from pinocchio import SE3, Quaternion
import time
from rospkg import RosPack
import gepetto.corbaserver
from mlp.utils.util import numpy2DToList, hppConfigFromMatrice, discretizeCurve, build_fullbody, Loop
from mlp.utils.requirements import Requirements
from pathlib import Path
pin.switchToNumpyArray()


class DisplayContactSequenceRequirements(Requirements):
    configurationValues = True

STONE_HEIGHT = 0.005  # Size along the z axis of the box used to display the contact placements
STONE_GROUP = "stepping_stones"  # Name of the group node in gepetto-gui containing the stepping stones nodes
STONE_RF = STONE_GROUP + "/" + "RF"
STONE_LF = STONE_GROUP + "/" + "LF"
STONE_RH = STONE_GROUP + "/" + "RH"
STONE_LH = STONE_GROUP + "/" + "LH"
TRAJ_GROUP = "com_traj"  # Name of the group node in gepetto-gui containing the curves of the CoM positions


def displaySphere(viewer, pos, size=0.01, color=[0, 0, 0, 1]):
    """
    Add a sphere in the viewer.
    Use the name "s_i" with i the smallest integer such that no other node with the same name exist.
    :param viewer: An instance of hpp.gepetto.Viewer
    :param pos: 3D translation in the world frame of the center of the sphere
    :param size: Radius of the sphere
    :param color: Color of the sphere (R, G, B, A)
    """
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
    """
    Convert a pinocchio.SE3 object to a python list of lenght 7 : translation + quaternion (x, y, z, w)
    :param placement: the pinocchio.SE3 object
    :return: a list of lenght 7
    """
    q = [0] * 7
    q[0:3] = placement.translation.tolist()
    r = Quaternion(placement.rotation)
    q[6] = r.w
    q[3:6] = r.coeffs().tolist()[0:3]
    return q


def addContactLandmark(M, color, v):
    """
    Add a landmark at the given placement.
    A landmark is a small sphere with 3 axis of different colors : Red (x axis), Green (y axis), Blue (z axis)
    :param M: a pinocchio.SE3 object
    :param color: the color of the sphere (R, G, B, A)
    :param v: a hpp.gepetto.Viewer object
    """
    global i_sphere
    gui = v.client.gui
    name = 's' + str(i_sphere)
    i_sphere += 1
    gui.addSphere(name, 0.01, color)
    #gui.setVisibility(name,"ALWAYS_ON_TOP")
    gui.addToGroup(name, v.sceneName)
    gui.applyConfiguration(name, SE3ToViewerConfig(M))
    gui.addLandmark(name, 0.03)
    #print "contact altitude : "+str(p[2])


def displayContactsLandmarkFromPhase(phase, viewer, Robot):
    """
    Display a landmark for each contact of the given phase, with colors as defined in Robot.dict_limb_color_traj
    :param phase: a ContactPhase object
    :param viewer: a hpp.gepetto.Viewer object
    :param Robot: a Robot configuration class (eg. the class defined in talos_rbprm.talos.Robot)
    """
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
    """
    Display a box at the given placement.
    :param gui: a gepetto.corbaserver.GraphicalInterface instance
    :param placement: a pinocchio.SE3 object defining the placement of the center of the box
    :param name: the node name of the new object created
    :param group: the name of the node group where the new node will be added
    :param size: the size [x, y] of the box
    :param color: the color (R, G, B, A) of the box)
    """
    gui.addBox(name, size[0], size[1], STONE_HEIGHT, color)
    gui.addToGroup(name, group)
    gui.applyConfiguration(name, SE3ToViewerConfig(placement))
    gui.setVisibility(name, "ON")

def hideSteppingStone(gui):
    """
    Set visibility = OFF for all the nodes belonging to the groups STONE_*
    :param gui: a gepetto.corbaserver.GraphicalInterface instance
    """
    node_list = gui.getNodeList()
    for node in node_list:
        if any(node.startswith(stone + "/") for stone in [STONE_LF, STONE_RF, STONE_LH, STONE_RH]):
            gui.setVisibility(node, "OFF")

def displaySteppingStones(cs, gui, sceneName, Robot):
    """
    Display a box at each contact placement defined in the given ContactSequence
    The size of the boxes match the size of the end effector creating the contact (defined in Robot.dict_size)
    The color used for each effector is defined in Robot.dict_limb_color_traj
    :param cs: a ContactSequence object
    :param gui: a gepetto.corbaserver.GraphicalInterface object
    :param sceneName: the node group used to add the new graphical objects
    :param Robot: a Robot configuration class (eg. the class defined in talos_rbprm.talos.Robot
    """
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
    """
    Display a curve representing the CoM 3D position stored in the given phase
    :param phase: the ContactPhase object storing the CoM trajectory
    :param gui: a gepetto.corbaserver.GraphicalInterface instance
    :param name: the node name of the graphical object
    :param name_group: the name of the group where the new node will be added
    :param color: the color of the trajectory (R, G, B, A)
    :param dt: the time step used to discretize the CoM trajectory
    """
    c = numpy2DToList(discretizeCurve(phase.c_t, dt)[0])
    gui.addCurve(name, c, color)
    gui.addToGroup(name, name_group)


def displayCOMTrajectory(cs, gui, sceneName, dt, colors=[[0., 0., 1., 1.], [0., 1., 0., 1.]], nameGroup=""):
    """
    Display curves representing the CoM 3D position stored in the phases of the given contact sequence
    :param cs: the ContactSequence object storing the CoM trajectories
    :param gui: a gepetto.corbaserver.GraphicalInterface instance
    :param sceneName: the node group used to add the new graphical objects
    :param dt: the time step used to discretize the CoM trajectories
    :param colors: A list of colors (R, G, B, A) of the curves. The color of the curves change at each contact phases
    :param nameGroup: the name of the group where the new node will be added
    """
    name_group = TRAJ_GROUP + nameGroup
    gui.createGroup(name_group)
    for pid,phase in enumerate(cs.contactPhases):
        name = name_group + "/" + '%.2f' % phase.timeInitial + "-" + '%.2f' % phase.timeFinal
        color = colors[pid % len(colors)]
        displayCOMTrajForPhase(phase, gui, name, name_group, color, dt)
    gui.addToGroup(name_group, sceneName)
    gui.refresh()


def displaySE3Traj(traj, gui, sceneName, name, color, time_interval, offset=SE3.Identity()):
    """
    Display a a given trajectory in the viewer.
    :param traj: a curves.SE3Curve object representing the trajectory to display
    :param gui: a gepetto.corbaserver.GraphicalInterface instance
    :param sceneName: the node group used to add the new graphical objects
    :param name: the name of the node created. If name is None, the name "SE3_traj_i" with i
    the smallest integer such that no other node with the same name exist is used
    :param color: the color of the trajectory (R, G, B, A)
    :param time_interval: [t_min, t_max] the time definition used to display the trajectory
    :param offset: A pinocchio.SE3 object defining an offset applied to the trajectory before displaying it
    """
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

def displayEffectorTrajectories(cs, viewer, Robot, suffixe = "", colorAlpha = 1, applyOffset = False):
    """
    Display all the effector trajectories stored in the given ContactSequence.
    With colors for each effectors defined in Robot.dict_limb_color_traj
    :param cs: the ContactSequence storing the trajectories
    :param viewer: An instance of hpp.gepetto.Viewer
    :param Robot: a Robot configuration class (eg. the class defined in talos_rbprm.talos.Robot)
    :param suffixe: an optionnal suffixe to the name of the node objects created
    :param colorAlpha: the transparency of the trajectories (must be between 0 and 1)
    """
    effectors = cs.getAllEffectorsInContact()
    for pid,phase in enumerate(cs.contactPhases):
        for eeName in effectors:
            if phase.effectorHaveAtrajectory(eeName):
                color = Robot.dict_limb_color_traj[eeName]
                color[-1] = colorAlpha
                if applyOffset:
                    offset = -Robot.dict_offset[eeName]
                else:
                    offset = SE3.Identity()
                displaySE3Traj(phase.effectorTrajectory(eeName), viewer.client.gui, viewer.sceneName,
                                 eeName + "_traj_"+ suffixe + str(pid), color,
                                 [phase.timeInitial, phase.timeFinal], offset)

def displayWBconfig(viewer, q_matrix):
    """
    display a joint configuration stored as a numpy.array in the viewer
    :param viewer: An instance of hpp.gepetto.Viewer
    :param q_matrix: a numpy.array representing the robot configuration
    """
    viewer(hppConfigFromMatrice(viewer.robot, q_matrix))


def displayWBatT(viewer, cs_wb, t):
    """
    Display the configuration of the robot stored in the given contact sequence at the given time
    :param viewer: An instance of hpp.gepetto.Viewer
    :param cs_wb: the ContactSequence storing the joint trajectories
    :param t: the desired time index
    """
    q = cs_wb.phaseAtTime(t).q_t(t)
    viewer(hppConfigFromMatrice(viewer.robot,q))


def displayFeetTrajFromResult(gui, sceneName, res, Robot):
    """
    Display all effector trajectories stored in a mlp.utils.wholebody_result.Result struct
    :param gui: a gepetto.corbaserver.GraphicalInterface instance
    :param sceneName: the node group used to add the new graphical objects
    :param res: a mlp.utils.wholebody_result.Result
    :param Robot: a Robot configuration class (eg. the class defined in talos_rbprm.talos.Robot)
    :return:
    """
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
    """
    Display a sequence of wholebody configuration for each new step of the robot.
    It use the ContactPhase.q_init member of each phases of the given sequence
    :param v: a hpp.gepetto.Viewer instance
    :param cs: a ContactSequence object storing the configuration of each phases
    :param step: the time step between each configuration change
    """
    for p in cs.contactPhases:
        displayWBconfig(v, p.q_init)
        time.sleep(step)
    displayWBconfig(v, cs.contactPhases[-1].q_final)


def initScene(Robot, envName="multicontact/ground", genLimbsDB=True, context=None):
    """
    Create and initialize a hpp.gepetto.Viewer object with a hpp.corbaserver.rbprm.FullBody object
    :param Robot: a Robot configuration class (eg. the class defined in talos_rbprm.talos.Robot)
    :param envName: the name of the environment file to load. Load a flat floor at z = 0 by default
    Currently only work with environments from the hpp-environment package
    :param genLimbsDB: If True (default value) generate the limbs database of the fullbody object
    :param context: An optional string that give a name to a corba context instance
    :return: a Fullbody object and a Viewer object
    """
    from hpp.gepetto import ViewerFactory
    fullBody, ps = build_fullbody(Robot, genLimbsDB, context)
    vf = ViewerFactory(ps)
    vf.loadObstacleModel("package://hpp_environments/urdf/" + envName + ".urdf", "planning")
    v = vf.createViewer(ghost = True, displayCoM=True)
    v(fullBody.getCurrentConfig())
    return fullBody, v


def initScenePinocchio(urdf_name, package_name, env_name=None, env_package_name="hpp_environments", scene_name = "world"):
    """
    Create and initialize a pinocchio.RobotWrapper with a gepetto-gui display
    :param urdf_name: the name of the robot urdf to load
    :param package_name: the name of the package containing the robot urdf
    :param env_name: the name of the environment file to load
    :param env_package_name: the name of the package containing the environment file
    :param scene_name: the main node group in the viewer (default to 'world' for pinocchio)
    :return: an instance of pinocchio.RobotWrapper and a gepetto.corbaserver.GraphicalInterface
    """
    rp = RosPack()
    package_path = rp.get_path(package_name)
    urdf_path = str(Path(package_path) / 'urdf' / Path(urdf_name+ '.urdf'))
    robot = pin.RobotWrapper.BuildFromURDF(urdf_path, package_path, pin.JointModelFreeFlyer())
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



class DisplayMotion(Loop):
    """
    Class used to display a new configuration of the robot in gepetto-gui at a given frequency
    """
    def __init__(self, period, q_t, display_function):
        """
        Constructor
        :param period: the time step between each new frame
        :param q_t: the joint trajectory to display, stored in a Curves object
        :param display_function: a pointer to a function that can be called with one argument:
         a joint configuration vector, and which display it
        """
        self.display_function = display_function
        self.q_t = q_t
        self.t = q_t.min()
        self.display_function(q_t(self.t)) # Display the first configuration
        super().__init__(period)

    def loop(self, signum, frame):
        self.t += self.period
        if self.t > self.q_t.max():
            # Display the last configuration and then stop the loop
            self.display_function(self.q_t(self.q_t.max()))
            self.stop()
        self.display_function(self.q_t(self.t))




def displayWBmotion(viewer, q_t, dt_display=0.04):
    """
    Display the motion represented by the given joint trajectory
    :param viewer: An instance of hpp.gepetto.Viewer
    :param q_t: a Curves object representing a joint trajectory
    :param dt_display: the time step between each frame displayed (default to 0.04s = 25fps)
    """
    def display_function(q):
        displayWBconfig(viewer, q)
    DisplayMotion(dt_display, q_t, display_function)



def disp_wb_pinocchio(robot, q_t, dt_display = 0.04):
    """
    Display the motion represented by the given joint trajectory
    :param robot: a RobotWrapper linked to a viewer
    :param q_t: a Curves object representing a joint trajectory
    :param dt_display: the time step between each frame displayed (default to 0.04s = 25 fps)
    """
    DisplayMotion(dt_display, q_t, robot.display)
