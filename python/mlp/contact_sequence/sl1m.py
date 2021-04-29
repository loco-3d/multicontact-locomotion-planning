try:
    # ~ import sl1m.planner as sl1m
    from sl1m.generic_solver import solve_L1_combinatorial, solve_MIP
    from sl1m.problem_definition import Problem
    from sl1m.stand_alone_scenarios.surfaces.stair_surfaces import quadruped_surfaces
except ImportError:
    message = "ERROR: Cannot import SL1M python library.\n"
    message += "Did you correctly installed it?\n"
    message +="See https://gepgitlab.laas.fr/loco-3d/sl1m"
    raise ImportError(message)
from pinocchio.utils import *
import importlib
import multicontact_api
from multicontact_api import ContactSequence
from mlp.utils.cs_tools import addPhaseFromConfig, setFinalState, computeCenterOfSupportPolygonFromPhase
from mlp.utils.util import rotationFromNormal
from mlp.viewer.display_tools import initScene, displaySteppingStones
from pinocchio.utils import matrixToRpy
from pinocchio import Quaternion, SE3
from hpp.corbaserver.rbprm.tools.surfaces_from_path import getSurfacesFromGuideContinuous
import random
from mlp.utils.requirements import Requirements
import numpy as np
from numpy import array
from numpy.linalg import norm
import logging
logging.basicConfig(format='[%(name)-12s] %(levelname)-8s: %(message)s')
logger = logging.getLogger("sl1m")
logger.setLevel(logging.DEBUG) #DEBUG, INFO or WARNING
multicontact_api.switchToNumpyArray()

class ContactOutputsSl1m(Requirements):
    consistentContacts = True

Z_AXIS = np.array([0, 0, 1])
EPS_Z = 0.001  # collision margin used, distance between the feet position and the surface required to avoid detecting it as a collision


#TODO REMOVE ?
# global vars that hold constraints for MIP
__ineq_com = []
__ineq_relative = []
# global vars that hold constraints for sl1m
__ineq_right_foot = None
__ineq_left_foot = None
__ineq_right_foot_reduced = None
__ineq_left_foot_reduced = None
__ineq_rf_in_rl = None
__ineq_lf_in_rf = None


####################################


def normal(phase):
    """
    Compute the normal of the first surface stored in the phase data
    :param phase: A phase data dict (see sl1m output doc)
    :return: The surface normal, as an array of size 3
    """
    s = phase["S"][0]
    n = cross(s[:, 1] - s[:, 0], s[:, 2] - s[:, 0])
    n /= norm(n)
    if n[2] < 0.:
        for i in range(3):
            n[i] = -n[i]
    logger.debug("normal %s", n)
    return n

def normal_from_ineq(s_ineq):
    """
    Get the normal stored in the phase surface inequalitie vector
    :param s_ineq: A phase data surface vector (see SL1M output doc)
    :return: The surface normal, as an array of size 3
    """
    return s_ineq[2]

def quatConfigFromMatrix(m):
    """
    Transform a rotation matrix to a list containing the quaternion representation (x, y, z, w)
    :param m: a rotation matrix
    :return: The Quaternion stored as a list
    """
    quat = Quaternion(m)
    return quatToConfig(quat)


def quatToConfig(quat):
    """
    Shape a pinocchio.Quaternion as a list
    :param quat: A Quaternion object
    :return: a list containing the quaternion values (x, y, z, w)
    """
    return [quat.x, quat.y, quat.z, quat.w]


def foot_pose_from_guide(Robot, root_pos):
    """
    Build a list of effector position (in the world frame) corresponding to the given root position
    :param Robot:
    :param root_pos: the desired position of the root
    :return: a List (of size num_effector),
    each element is an array representing the effector 3D position in the world frame
    """
    return [array(root_pos[0:3]) + Robot.dict_ref_effector_from_root[limb_name] +
            Robot.dict_offset[Robot.dict_limb_joint[limb_name]].translation
            for limb_name in Robot.limbs_names]
    # FIXME: apply epsz along the normal


def initial_foot_pose_from_fullbody(fullbody, q_init):
    """
    Build a list of effector position (in the world frame) corresponding to the given whole body configuration
    :param fullbody: an instance of rbprm.FullBody
    :param q_init: the desired whole body configuration
    :return: a List (of size num_effector),
    each element is an array representing the effector 3D position in the world frame
    """
    fullbody.setCurrentConfig(q_init)
    ee_names = [fullbody.dict_limb_joint[limb] for limb in fullbody.limbs_names]
    return [array(fullbody.getJointPosition(ee_name)[:3]) +
            fullbody.dict_offset[ee_name].translation
            for ee_name in ee_names]
    # FIXME: apply epsz along the normal


def gen_pbREMOVE(Robot, initial_contacts, R, surfaces):
    """
    Build a SL1M problem, with all the kinematics and foot relative position constraints required
    :param Robot:
    :param initial_contacts: A list of the initial contact positions
    :param R: a list of rotation matrix for the base of the robot (must be the same size as surfaces)
    :param surfaces: A list of surfaces candidates, with one set of surface candidates for each phase
    :return: a "res" dictionnary with the format required by SL1M
    """
    #  Nested function declaration, as they need access to the Robot variable:
    def right_foot_constraints(transform):
        global __ineq_right_foot
        if __ineq_right_foot is None:
            obj = sl1m.load_obj(Robot.filekin_right)
            __ineq_right_foot = sl1m.as_inequalities(obj)
        transform2 = transform.copy()
        transform2[2, 3] += 0.105
        ine = sl1m.rotate_inequalities(__ineq_right_foot, transform2)
        return (ine.A, ine.b)

    def left_foot_constraints(transform):
        global __ineq_left_foot
        if __ineq_left_foot is None:
            obj = sl1m.load_obj(Robot.filekin_left)
            __ineq_left_foot = sl1m.as_inequalities(obj)
        transform2 = transform.copy()
        transform2[2, 3] += 0.105
        ine = sl1m.rotate_inequalities(__ineq_left_foot, transform2)
        return (ine.A, ine.b)

    # add foot offset
    def right_foot_in_lf_frame_constraints(transform):
        global __ineq_rf_in_rl
        if __ineq_rf_in_rl is None:
            obj = sl1m.load_obj(Robot.file_rf_in_lf)
            __ineq_rf_in_rl = sl1m.as_inequalities(obj)
        transform2 = transform.copy()
        ine = sl1m.rotate_inequalities(__ineq_rf_in_rl, transform2)
        return (ine.A, ine.b)

    def left_foot_in_rf_frame_constraints(transform):
        global __ineq_lf_in_rf
        if __ineq_lf_in_rf is None:
            obj = sl1m.load_obj(Robot.file_lf_in_rf)
            __ineq_lf_in_rf = sl1m.as_inequalities(obj)
        transform2 = transform.copy()
        ine = sl1m.rotate_inequalities(__ineq_lf_in_rf, transform2)
        return (ine.A, ine.b)

    logger.debug("surfaces = %s",surfaces)
    logger.info("number of surfaces : %d", len(surfaces))
    logger.info("number of rotation matrix for root : %d", len(R))
    nphases = len(surfaces)
    logger.info("init_contacts = %s", initial_contacts)

    res = {"p0": initial_contacts, "c0": None, "nphases": nphases}
    #print "number of rotations values : ",len(R)
    #print "R= ",R
    #TODO in non planar cases, K must be rotated
    phaseData = [{
        "moving":
        i % 2,
        "fixed": (i + 1) % 2,
        "K": [
            sl1m.genKinematicConstraints(left_foot_constraints, right_foot_constraints, index=i, rotation=R, min_height=0.3)
            for _ in range(len(surfaces[i]))
        ],
        "relativeK": [
            sl1m.genFootRelativeConstraints(right_foot_in_lf_frame_constraints,
                                       left_foot_in_rf_frame_constraints,
                                       index=i,
                                       rotation=R)[(i) % 2] for _ in range(len(surfaces[i]))
        ],
        "rootOrientation":
        R[i],
        "S":
        surfaces[i]
    } for i in range(nphases)]
    res["phaseData"] = phaseData
    return res

## Helper methods to load constraints :
# add foot offset
def com_in_limb_effector_frame_constraintREMOVE(Robot, transform, limbId):
    """
    Generate the inequalities constraints for the CoM position given a contact position for one limb
    :param Robot:
    :param transform: Transformation to apply to the constraints
    :param limbId: the Id of the limb used (see Robot.limbs_names list)
    :return: [A, b] the inequalities, in the form Ax <= b
    """
    global __ineq_com
    assert (limbId <= len(Robot.limbs_names))
    limb_name = Robot.limbs_names[limbId]
    if __ineq_com[limbId] is None:
        filekin = Robot.kinematic_constraints_path +"/COM_constraints_in_"+limb_name+ "_effector_frame_quasi_static_reduced.obj"
        obj = sl1m.load_obj(filekin)
        __ineq_com[limbId] = sl1m.as_inequalities(obj)
    transform2 = transform.copy()
    # ~ transform2[2,3] += 0.105
    ine = sl1m.rotate_inequalities(__ineq_com[limbId], transform2)
    return ine.A, ine.b


def realIdxFootId(limbId, footId):
    if footId > limbId:
        return footId -1
    return footId

# add foot offset
def foot_in_limb_effector_frame_constraintREMOVE(Robot, transform, limbId, footId):
    """
    Generate the constraints for the position of a given effector, given another effector position
    :param Robot:
    :param transform: The transform to apply to the constraints
    :param limbId: the Id of the fixed limb (see Robot.limbs_names list)
    :param footId: the Id of the limb for which the constraint are build
    :return: [A, b] the inequalities, in the form Ax <= b
    """
    global __ineq_relative
    assert (limbId != footId)
    limb_name = Robot.limbs_names[limbId]
    eff_name = Robot.dict_limb_joint[Robot.limbs_names[footId]]
    if __ineq_relative[limbId][realIdxFootId(limbId,footId)] is None:
        filekin = Robot.relative_feet_constraints_path + "/" + eff_name + "_constraints_in_" + limb_name + "_reduced.obj"
        obj = sl1m.load_obj(filekin)
        __ineq_relative[limbId][realIdxFootId(limbId,footId)] = sl1m.as_inequalities(obj)
    transform2 = transform.copy()
    ine = sl1m.rotate_inequalities(__ineq_relative[limbId][realIdxFootId(limbId,footId)], transform2)
    return ine.A, ine.b

def genCOMConstraintsREMOVE(Robot, rotation, normals):
    """
    Generate the constraints on the CoM position for all the effectors
    :param Robot:
    :param rotation: the rotation to apply to the constraints
    :param normals: the default contact normals of each effectors
    :return: a list of [A,b] inequalities, in the form Ax <= b
    """
    return [com_in_limb_effector_frame_constraint(Robot, sl1m.default_transform_from_pos_normal_(rotation, sl1m.zero3, normals[idx]),idx)
            for idx in range(len(Robot.limbs_names))]

def genRelativeConstraintsREMOVE(Robot, rotation, normals):
    """
    Generate all the relative position constraints for all limbs
    :param Robot:
    :param rotation: the rotation to apply to the constraints
    :param normals: the default contact normals of each effectors
    :return: a list of [A,b] inequalities, in the form Ax <= b
    """
    transforms = [sl1m.default_transform_from_pos_normal_(rotation, sl1m.zero3, normals[idx])
                  for idx in range(len(Robot.limbs_names))]
    res = []
    for limbId, transform in enumerate(transforms):
        res += [[(footId, foot_in_limb_effector_frame_constraint(Robot, transform, limbId, footId))
                 for footId in range(len(Robot.limbs_names)) if footId != limbId]]
    return res


def gen_pb_mipREMOVE(Robot, R, surfaces):
    """
    Build a SL1M problem for the Mixed Integer formulation,
    with all the kinematics and foot relative position constraints required
    :param Robot:
    :param R: a list of rotation matrix for the base of the robot (must be the same size as surfaces)
    :param surfaces: A list of surfaces candidates, with one set of surface candidates for each phase
    :return: a "res" dictionnary with the format required by SL1M
    """
    normals = [np.array([0, 0, 1]) for _ in range(len(Robot.limbs_names))] #FIXME: compute it from the surfaces
    logger.debug("surfaces = %s",surfaces)
    logger.info("number of surfaces : %d", len(surfaces))
    logger.info("number of rotation matrix for root : %d", len(R))
    nphases = len(surfaces)
    res = {"p0": None, "c0": None, "nphases": nphases}
    #TODO in non planar cases, K must be rotated
    phaseData = [{
        "K": [genCOMConstraints(Robot, R[i], normals) for _ in range(len(surfaces[i]))],
        "allRelativeK": [genRelativeConstraints(Robot, R[i], normals) for _ in range(len(surfaces[i]))],
        "rootOrientation":R[i],
        "S": surfaces[i]
    } for i in range(nphases)]
    res["phaseData"] = phaseData
    return res

def solve(planner, cfg, display_surfaces = False, initial_contacts = None, final_contacts = None):
    """
    Automatically formulate and solve a SL1M problem.
    First call the method to extract the surfaces candidates from the guide path,
    Then generate the problem from this surfaces and finally solve it with SL1M.
    If the problem cannot be solved, try again with a small variation on the discretization step size
    used to extract surfaces candidates from the guide path.
    :param planner: A rbprm.AbstractPlanner instance which store the result of the planning problem
    :param cfg:
    :param display_surfaces: if True, display the candidate surfaces with the viewer instanciated in the planner
    :param initial_contacts: a list of the initial contact position for each limb.
    If None, it will be computed from the initial root position stored in the planner
    :param final_contacts: a list of the final contact position for each limb.
    If None, it will be computed from the final root position stored in the planner
    :return: [pathId, pb, coms, footpos, allfeetpos, res]:
    pathId: the Id of the guide path used
    pb: a dictionary defining the SL1M problem
    coms: a list of CoM position at each phase, computed by SL1M
    footpos: a list of effector position at each phase, computed by SL1M
    allfeetpos: a list of effector position at each phase, computed by SL1M
    res: A dictionary containing the otput of SL1M
    """
    
    if initial_contacts is None:
        initial_contacts = foot_pose_from_guide(cfg.Robot, planner.q_init)
    if final_contacts is None:
        final_contacts = foot_pose_from_guide(cfg.Robot, planner.q_goal)

    logger.info("Initial contacts : %s", initial_contacts)
    logger.info("Final contacts : %s", final_contacts)
    #  Extract candidate surfaces and root rotation from the guide planning
    success = False
    maxIt = 50
    it = 0
    defaultStep = cfg.GUIDE_STEP_SIZE
    step = defaultStep
    variation = 0.4  # FIXME : put it in config file, +- bounds on the step size    
    if hasattr(planner, "pathId"):
        pathId = planner.pathId
    elif hasattr(planner, "pId"):
        pathId = planner.pId
    else:
        pathId = planner.ps.numberPaths() - 1
    
    solveFun = None  
    if cfg.SL1M_USE_MIP:
        def f(pb, surfaces):
            return solve_MIP(pb, surfaces, costs={"final_com" : [1., planner.q_goal[0:3]]})
        solveFun = f
    else:
        def f(pb, surfaces):
            return solve_L1_combinatorial(pb, surfaces, costs={"final_com" : [1., planner.q_goal[0:3]]})
        solveFun = f
        
    
    while not success and it < maxIt:
        if it > 0:
            step = defaultStep + random.uniform(-variation, variation)
        viewer = planner.v
        if not hasattr(viewer, "client"):
            viewer = None
        R, surfaces = getSurfacesFromGuideContinuous(planner.rbprmBuilder,
                                                     planner.ps,
                                                     planner.afftool,
                                                     pathId,
                                                     viewer if display_surfaces else None,
                                                     step,
                                                     useIntersection=cfg.SL1M_USE_INTERSECTION,
                                                     max_yaw=cfg.GUIDE_MAX_YAW,
                                                     max_surface_area=cfg.MAX_SURFACE_AREA)
        pb = Problem(cfg.Robot, suffix_com= cfg.SL1M_SUFFIX_COM_CONSTRAINTS, suffix_feet=cfg.SL1M_SUFFIX_FEET_CONSTRAINTS, limb_names=cfg.SL1M_FEET_NAME_FOR_CONSTRAINTS)
        pb.generate_problem(R, surfaces[:], cfg.SL1M_GAIT, initial_contacts, np.array(planner.q_init[0:3])) #TODO COM position not used
        try:
            resultData = solveFun(pb, surfaces)
            success = True
        except Exception as e:
            logger.warning("## Planner failed at iter : %d with step length = %f", it, step)
        it += 1
    if not success:
        raise RuntimeError("planner always fail.")
    
    
    # ~ import sl1m.tools.plot_tools as plot
    # ~ from mpl_toolkits.mplot3d import Axes3D
    # ~ ax = plot.draw_scene([], cfg.SL1M_GAIT)
    # ~ plot.plot_initial_contacts(initial_contacts, ax=ax)
    # ~ if(success):
        # ~ plot.plot_planner_result(resultData.coms, resultData.moving_foot_pos, resultData.all_feet_pos, ax, True)
    # ~ else:
        # ~ plt.show(block=False)
    return pathId, pb, resultData 


def runLPFromGuideScript(cfg):
    """
    Instanciate a rbprm.AbstractPlanner class from the data in the configuration script, and run it.
    Then, call the solve() method to automatically formulate and solve a SL1M problem.
    :param cfg:
    :return: [RF, root_init, root_end, pb, coms, footpos, allfeetpos, res]
    RF: the Id of the right foot (used to know which foot moved first)
    root_init: the initial position of the base in the planning problem
    root_end: the final position of the base in the planning problem
    pb: a dictionary defining the SL1M problem
    coms: a list of CoM position at each phase, computed by SL1M
    footpos: a list of effector position at each phase, computed by SL1M
    allfeetpos: a list of effector position at each phase, computed by SL1M
    res: A dictionary containing the otput of SL1M
    """
    if hasattr(cfg, 'SCRIPT_ABSOLUTE_PATH'):
        scriptName = cfg.SCRIPT_ABSOLUTE_PATH
    else:
        scriptName = cfg.RBPRM_SCRIPT_PATH + "." + cfg.SCRIPT_PATH + '.' + cfg.DEMO_NAME
    scriptName += "_path"
    logger.warning("Run Guide script : %s", scriptName)
    module = importlib.import_module(scriptName)
    planner = module.PathPlanner()
    planner.run()
    # compute sequence of surfaces from guide path
    # ~ pathId, pb, coms, footpos, allfeetpos, res = solve(planner, cfg, cfg.DISPLAY_SL1M_SURFACES)
    pathId, pb, resultData = solve(planner, cfg, cfg.DISPLAY_SL1M_SURFACES)
    root_init = planner.ps.configAtParam(pathId, 0.001)[0:7]
    root_end = planner.ps.configAtParam(pathId, planner.ps.pathLength(pathId) - 0.001)[0:7]
    # ~ return sl1m.RF, root_init, root_end, pb, resultData
    return cfg.SL1M_GAIT[0], root_init, root_end, pb, resultData


def runLPScript(cfg):
    """
    Import and run a "stand alone" SL1M script. This script must contain a solve() method
    :param cfg:
    :return: [RF, root_init, root_end, pb, coms, footpos, allfeetpos, res]
    RF: the Id of the right foot (used to know which foot moved first)
    root_init: the initial position of the base in the planning problem
    root_end: the final position of the base in the planning problem
    pb: a dictionary defining the SL1M problem
    coms: a list of CoM position at each phase, computed by SL1M
    footpos: a list of effector position at each phase, computed by SL1M
    allfeetpos: a list of effector position at each phase, computed by SL1M
    res: A dictionary containing the otput of SL1M
    """
    #the following script must produce a
    if hasattr(cfg, 'SCRIPT_ABSOLUTE_PATH'):
        scriptName = cfg.SCRIPT_ABSOLUTE_PATH
    else:
        scriptName = 'scenarios.' + cfg.SCRIPT_PATH + '.' + cfg.DEMO_NAME
    logger.warning("Run LP script : %s", scriptName)
    cp = importlib.import_module(scriptName)
    pb, coms, footpos, allfeetpos, res = cp.solve()
    root_init = cp.root_init[0:7]
    root_end = cp.root_end[0:7]
    return cp.RF, root_init, root_end, pb, coms, footpos, allfeetpos, res


def generate_contact_sequence_sl1m(cfg):
    #RF,root_init,pb, coms, footpos, allfeetpos, res = runLPScript(cfg)
    RF, root_init, root_end, pb, resultData = runLPFromGuideScript(cfg)
    allfeetpos = resultData.all_feet_pos
    multicontact_api.switchToNumpyArray()
    # load scene and robot
    fb, v = initScene(cfg.Robot, cfg.ENV_NAME, True)
    q_init = cfg.IK_REFERENCE_CONFIG.tolist() + [0] * 6
    q_init[0:7] = root_init
    feet_height_init = allfeetpos[0][0][2]
    logger.info("feet height initial = %s", feet_height_init)
    #q_init[2] = feet_height_init + cfg.IK_REFERENCE_CONFIG[2]
    #q_init[2] += EPS_Z
    #q_init[2] = fb.referenceConfig[2] # 0.98 is in the _path script
    if v:
        v(q_init)

    cs = build_cs_from_sl1m(cfg.SL1M_USE_MIP, fb, cfg.IK_REFERENCE_CONFIG, root_end, pb, RF, allfeetpos,
                                cfg.SL1M_USE_ORIENTATION, cfg.SL1M_USE_INTERPOLATED_ORIENTATION, q_init=q_init)

    if cfg.DISPLAY_CS_STONES:
        displaySteppingStones(cs, v.client.gui, v.sceneName, fb)

    return cs, fb, v

def compute_orientation_for_feet_placement(fb, pb, pId, moving, RF, prev_contactPhase, use_interpolated_orientation):
    """
    Compute the rotation of the feet from the base orientation.
    :param fb: an instance of rbprm.Fullbody
    :param pb: the SL1M problem dictionary, containing all the phaseData
    :param pId: the Id of the current phase (SL1M index)
    :param moving: the Id of the moving feet
    :param RF: the Id of the right feet in the SL1M solver
    :param prev_contactPhase: the multicontact_api.ContactPhase of the previous phase
    :param use_interpolated_orientation: If False, the desired contact rotation is the current base orientation.
    If True, the desired contact rotation is the interpolation between the current base orientation
    and the one for the next phase
    :return: the rotation matrix of the new contact placement
    """
    quat0 = Quaternion(pb["phaseData"][pId]["rootOrientation"])
    if pId < len(pb["phaseData"]) - 1:
        quat1 = Quaternion(pb["phaseData"][pId + 1]["rootOrientation"])
    else:
        quat1 = Quaternion(pb["phaseData"][pId]["rootOrientation"])
    if use_interpolated_orientation:
        rot = quat0.slerp(0.5, quat1)
        # check if feets do not cross :
        if moving == RF:
            qr = rot
            ql = Quaternion(prev_contactPhase.contactPatch(fb.lfoot).placement.rotation)
        else:
            ql = rot
            qr = Quaternion(prev_contactPhase.contactPatch(fb.rfoot).placement.rotation)
        rpy = matrixToRpy((qr * (ql.inverse())).matrix())  # rotation from the left foot pose to the right one
        if rpy[2] > 0:  # yaw positive, feet are crossing
            rot = quat0  # rotation of the root, from the guide
    else:
        rot = quat0  # rotation of the root, from the guide
    return rot

def build_cs_from_sl1m(use_mip, fb, q_ref, root_end, pb, RF, allfeetpos, use_orientation, use_interpolated_orientation,
                       q_init = None, first_phase = None):
    """
    Build a multicontact_api.ContactSequence from the SL1M outputs.
    :param use_mip: should be True if the MIP solver was called, false otherwise
    (this change the data format of the outputs)
    :param fb: an instance of rbprm.Fullbody
    :param q_ref: the reference wholebody configuration of the robot
    :param root_end: the final base position
    :param pb: the SL1M problem dictionary, containing all the contact surfaces and data
    :param RF: the Id of the right feet in the SL1M formulation
    :param allfeetpos: the list of all foot position for each phase, computed by SL1M
    :param use_orientation: if True, change the contact yaw rotation to match the orientation of the base in the guide
    :param use_interpolated_orientation: if True, the feet yaw orientation will 'anticipate' the base orientation
    of the next phase
    :param q_init: the initial wholebody configuration (either this or first_phase should be provided)
    :param first_phase: the first multicontact_api.ContactPhase object (either this or q_init should be provided)
    :return: the multicontact_api.ContactSequence, with all ContactPhase created at the correct placement
    """
    # ~ if use_mip:
    return build_cs_from_sl1m_mip(fb, q_ref, root_end, pb, allfeetpos, use_orientation, use_interpolated_orientation,
     q_init, first_phase)
    # ~ else:
        # ~ return build_cs_from_sl1m_l1(fb, q_ref, root_end, pb, RF, allfeetpos, use_orientation, use_interpolated_orientation,
        # ~ q_init, first_phase)

def build_cs_from_sl1m_l1(fb, q_ref, root_end, pb, RF, allfeetpos, use_orientation, use_interpolated_orientation,
                       q_init = None, first_phase = None):
    """
    Build a multicontact_api.ContactSequence from the SL1M outputs, when not using the MIP formulation
    :param fb: an instance of rbprm.Fullbody
    :param q_ref: the reference wholebody configuration of the robot
    :param root_end: the final base position
    :param pb: the SL1M problem dictionary, containing all the contact surfaces and data
    :param RF: the Id of the right feet in the SL1M formulation
    :param allfeetpos: the list of all foot position for each phase, computed by SL1M
    :param use_orientation: if True, change the contact yaw rotation to match the orientation of the base in the guide
    :param use_interpolated_orientation: if True, the feet yaw orientation will 'anticipate' the base orientation
    of the next phase
    :param q_init: the initial wholebody configuration (either this or first_phase should be provided)
    :param first_phase: the first multicontact_api.ContactPhase object (either this or q_init should be provided)
    :return: the multicontact_api.ContactSequence, with all ContactPhase created at the correct placement
    """
    # init contact sequence with first phase : q_ref move at the right root pose and with both feet in contact
    # FIXME : allow to customize that first phase
    cs = ContactSequence(0)
    if q_init:
        addPhaseFromConfig(fb, cs, q_init, [fb.rLegId, fb.lLegId])
    elif first_phase:
        cs.append(first_phase)
    else:
        raise ValueError("build_cs_from_sl1m should have either q_init or first_phase argument defined")

    # loop over all phases of pb and add them to the cs :
    for pId in range(2, len(pb["phaseData"])):  # start at 2 because the first two ones are already done in the q_init
        prev_contactPhase = cs.contactPhases[-1]
        #n = normal(pb["phaseData"][pId])
        phase = pb["phaseData"][pId]
        moving = phase["moving"]
        movingID = fb.lfoot
        if moving == RF:
            movingID = fb.rfoot
        pos = allfeetpos[pId]  # array, desired position for the feet movingID
        pos[2] += EPS_Z  # FIXME it shouldn't be required !!
        # compute desired foot rotation :
        if use_orientation:
            rot = compute_orientation_for_feet_placement(fb, pb, pId, moving, RF, prev_contactPhase, use_interpolated_orientation)
        else:
            rot = Quaternion.Identity()
        placement = SE3()
        placement.translation = np.array(pos).T
        placement.rotation = rot.matrix()
        cs.moveEffectorToPlacement(movingID, placement)

    # final phase :
    # fixme : assume root is in the middle of the last 2 feet pos ...
    q_end = q_ref.tolist() + [0] * 6
    #p_end = (allfeetpos[-1] + allfeetpos[-2]) / 2.
    #for i in range(3):
    #    q_end[i] += p_end[i]
    q_end[0:7] = root_end
    feet_height_end = allfeetpos[-1][2]
    logger.info("feet height final = %s", feet_height_end)
    q_end[2] = feet_height_end + q_ref[2]
    q_end[2] += EPS_Z
    fb.setCurrentConfig(q_end)
    com = fb.getCenterOfMass()
    setFinalState(cs, com, q=q_end)

    return cs

def build_cs_from_sl1m_mip(fb, q_ref, root_end, pb, allfeetpos, use_orientation, use_interpolated_orientation,
                       q_init = None, first_phase = None):
    """
    Build a multicontact_api.ContactSequence from the SL1M outputs when using the MIP formulation
    :param fb: an instance of rbprm.Fullbody
    :param q_ref: the reference wholebody configuration of the robot
    :param root_end: the final base position
    :param pb: the SL1M problem dictionary, containing all the contact surfaces and data
    :param allfeetpos: the list of all foot position for each phase, computed by SL1M
    :param use_orientation: if True, change the contact yaw rotation to match the orientation of the base in the guide
    :param use_interpolated_orientation: if True, the feet yaw orientation will 'anticipate' the base orientation
    of the next phase
    :param q_init: the initial wholebody configuration (either this or first_phase should be provided)
    :param first_phase: the first multicontact_api.ContactPhase object (either this or q_init should be provided)
    :return: the multicontact_api.ContactSequence, with all ContactPhase created at the correct placement
    """
    # init contact sequence with first phase : q_ref move at the right root pose and with both feet in contact
    # FIXME : allow to customize that first phase
    num_steps = len(pb.phaseData) - 1 # number of contact repositionning
    limbs_names = fb.limbs_names
    num_effectors = len(limbs_names)
    logger.info(" limbs names : %s", limbs_names)
    cs = ContactSequence(0)
    if first_phase:
        cs.append(first_phase)
    elif q_init:
        # if only the configuration is provided, assume all effectors are in contact
        addPhaseFromConfig(fb, cs, q_init, limbs_names)
    else:
        raise ValueError("build_cs_from_sl1m should have either q_init or first_phase argument defined")
    logger.info("Initial phase added, contacts : %s ", cs.contactPhases[0].effectorsInContact())
    # loop for all effector placements, and create the required contact phases
    allfeetpos = [[el[i] for el in allfeetpos] for i in range(len(allfeetpos[0]))]
    previous_eff_placements = allfeetpos[0]
    if len(previous_eff_placements) != num_effectors:
        raise NotImplementedError("A phase in the output of SL1M do not have all the effectors in contact.")
    for pid, eff_placements in enumerate(allfeetpos[1:]):
        logger.info("Loop allfeetpos, id = %d", pid)
        if len(eff_placements) != num_effectors:
            raise NotImplementedError("A phase in the output of SL1M do not have all the effectors in contact.")
        switch = False # True if a repostionning have been detected
        for k, pos in enumerate(eff_placements):
            if norm(pos - previous_eff_placements[k]) > 1e-3:
                if switch:
                    raise NotImplementedError("Several contact changes between two adjacent phases in SL1M output")
                switch = True
                ee_name = fb.dict_limb_joint[limbs_names[k]]
                pos = fb.dict_offset[ee_name].actInv(pos); 
                logger.info("Move effector %s ", ee_name)
                logger.info("To position %s ", pos)
                placement = SE3.Identity()
                placement.translation = pos;
                # compute orientation of the contact from the surface normal:
                # ~ phase_data = pb.phaseData[pid+1] # +1 because the for loop start at id = 1
                phase_data = pb.phaseData[pid] # +1 because the for loop start at id = 1
                # ~ n = normal_from_ineq(phase_data.S[phase_data["id_surface"]])
                #TODO RETRIEVE SURFACE NORMAL
                n = array([0.,0.,1.])
                placement.rotation = rotationFromNormal(n)
                logger.debug("new contact placement : %s", placement)
                # TODO add yaw rotation from guide here !
                cs.moveEffectorToPlacement(ee_name, placement)

        #if not switch:
        #   raise RuntimeError("No contact changes between two adjacent phases in SL1M output")
        # assign com position to the last two phases :
        # swinging phase, same init and final position
        """
        cs.contactPhases[-2].c_init = coms[pid * 2]
        cs.contactPhases[-2].c_final = coms[pid * 2 + 1]
        # phase with all the contacts:
        cs.contactPhases[-1].c_init = coms[pid * 2 + 1]
        if pid * 2 + 2 < len(coms):
            cs.contactPhases[-1].c_final = coms[pid * 2 + 2]
        else:
            cs.contactPhases[-1].c_final = cs.contactPhases[-1].c_init
        """
        previous_eff_placements = eff_placements

    # final phase :
    # fixme : assume root is in the middle of the last 2 feet pos ...
    """
    q_end = q_ref.tolist() + [0] * 6
    # p_end = (allfeetpos[-1] + allfeetpos[-2]) / 2.
    # for i in range(3):
    #    q_end[i] += p_end[i]
    q_end[0:7] = root_end
    feet_height_end = allfeetpos[-1][0][2]
    logger.info("feet height final = %s", feet_height_end)
    q_end[2] = feet_height_end + q_ref[2]
    #q_end[2] += EPS_Z
    fb.setCurrentConfig(q_end)
    com = fb.getCenterOfMass()
    setFinalState(cs, com, q=q_end)
    """
    p_final = cs.contactPhases[-1]
    p_final.c_final = computeCenterOfSupportPolygonFromPhase(p_final, fb.DEFAULT_COM_HEIGHT)
    p_final.c_init = p_final.c_final
    return cs
