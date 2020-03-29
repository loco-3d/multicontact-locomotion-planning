try:
    from sl1m.planner import *
except ImportError:
    message = "ERROR: Cannot import SL1M python library.\n"
    message += "Did you correctly installed it?\n"
    message +="See https://gepgitlab.laas.fr/loco-3d/sl1m"
    raise ImportError(message)
from pinocchio.utils import *
import importlib
import multicontact_api
from multicontact_api import ContactSequence
from mlp.utils.cs_tools import addPhaseFromConfig, setFinalState
from mlp.viewer.display_tools import initScene, displaySteppingStones
from pinocchio.utils import matrixToRpy
from pinocchio import Quaternion, SE3
from hpp.corbaserver.rbprm.tools.surfaces_from_path import getSurfacesFromGuideContinuous
import random
from mlp.utils.requirements import Requirements
import logging
logging.basicConfig(format='[%(name)-12s] %(levelname)-8s: %(message)s')
logger = logging.getLogger("sl1m")
logger.setLevel(logging.WARNING) #DEBUG, INFO or WARNING
multicontact_api.switchToNumpyArray()

class ContactOutputsSl1m(Requirements):
    consistentContacts = True

Z_AXIS = np.array([0, 0, 1])
EPS_Z = 0.005  # offset added to feet z position, otherwise there is collisions with the ground

##### MOVE the next methods to a robot-specific file : #####
import os
# load  constraints
__ineq_right_foot = None
__ineq_left_foot = None
__ineq_right_foot_reduced = None
__ineq_left_foot_reduced = None


# add foot offset
def right_foot_constraints(transform):
    global __ineq_right_foot
    if __ineq_right_foot is None:
        filekin = os.environ[
            "INSTALL_HPP_DIR"] \
            + "/share/talos-rbprm/com_inequalities/feet_quasi_flat/talos_COM_constraints_in_RF_effector_frame_REDUCED.obj"
        obj = load_obj(filekin)
        __ineq_right_foot = as_inequalities(obj)
    transform2 = transform.copy()
    transform2[2, 3] += 0.105
    ine = rotate_inequalities(__ineq_right_foot, transform2)
    return (ine.A, ine.b)


def left_foot_constraints(transform):
    global __ineq_left_foot
    if __ineq_left_foot is None:
        filekin = os.environ[
            "INSTALL_HPP_DIR"] \
            + "/share/talos-rbprm/com_inequalities/feet_quasi_flat/talos_COM_constraints_in_LF_effector_frame_REDUCED.obj"
        obj = load_obj(filekin)
        __ineq_left_foot = as_inequalities(obj)
    transform2 = transform.copy()
    transform2[2, 3] += 0.105
    ine = rotate_inequalities(__ineq_left_foot, transform2)
    return (ine.A, ine.b)


__ineq_rf_in_rl = None
__ineq_lf_in_rf = None


# add foot offset
def right_foot_in_lf_frame_constraints(transform):
    global __ineq_rf_in_rl
    if __ineq_rf_in_rl is None:
        filekin = os.environ[
            "INSTALL_HPP_DIR"] \
            + "/share/talos-rbprm/relative_effector_positions/talos_RF_constraints_in_LF_quasi_flat_REDUCED.obj"
        obj = load_obj(filekin)
        __ineq_rf_in_rl = as_inequalities(obj)
    transform2 = transform.copy()
    ine = rotate_inequalities(__ineq_rf_in_rl, transform2)
    return (ine.A, ine.b)


def left_foot_in_rf_frame_constraints(transform):
    global __ineq_lf_in_rf
    if __ineq_lf_in_rf is None:
        filekin = os.environ[
            "INSTALL_HPP_DIR"] \
            + "/share/talos-rbprm/relative_effector_positions/talos_LF_constraints_in_RF_quasi_flat_REDUCED.obj"
        obj = load_obj(filekin)
        __ineq_lf_in_rf = as_inequalities(obj)
    transform2 = transform.copy()
    ine = rotate_inequalities(__ineq_lf_in_rf, transform2)
    return (ine.A, ine.b)


####################################


def normal(phase):
    s = phase["S"][0]
    n = cross(s[:, 1] - s[:, 0], s[:, 2] - s[:, 0])
    n /= norm(n)
    if n[2] < 0.:
        for i in range(3):
            n[i] = -n[i]
    logger.debug("normal %s", n)
    return n


def quatConfigFromMatrix(m):
    quat = Quaternion(m)
    return quatToConfig(quat)


def quatToConfig(quat):
    return [quat.x, quat.y, quat.z, quat.w]


# FIXME : HARDCODED stuff for talos in this method !
def gen_pb(root_init, R, surfaces, ref_root_height):
    logger.debug("surfaces = %s",surfaces)
    logger.info("number of surfaces : %d", len(surfaces))
    logger.info("number of rotation matrix for root : %d", len(R))
    nphases = len(surfaces)
    lf_0 = array(root_init[0:3]) + array([0, 0.085, -ref_root_height])  # values for talos !
    rf_0 = array(root_init[0:3]) + array([0, -0.085, -ref_root_height])  # values for talos !
    #init_floor_height = surfaces[0][0][2][0]
    # z value of the first surface in intersection with the rom in the initial configuration
    #lf_0[2] = init_floor_height
    #rf_0[2] = init_floor_height
    p0 = [lf_0, rf_0]
    logger.info("p0 used : %s", p0)
    res = {"p0": p0, "c0": None, "nphases": nphases}
    #print "number of rotations values : ",len(R)
    #print "R= ",R
    #TODO in non planar cases, K must be rotated
    phaseData = [{
        "moving":
        i % 2,
        "fixed": (i + 1) % 2,
        "K": [
            genKinematicConstraints(left_foot_constraints, right_foot_constraints, index=i, rotation=R, min_height=0.3)
            for _ in range(len(surfaces[i]))
        ],
        "relativeK": [
            genFootRelativeConstraints(right_foot_in_lf_frame_constraints,
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


def solve(planner, guide_step_size, guide_max_yaw, max_surface_area, ref_root_height, display_surfaces = False):
    from sl1m.fix_sparsity import solveL1
    #surfaces_dict = getAllSurfacesDict(planner.afftool)
    success = False
    maxIt = 50
    it = 0
    defaultStep = guide_step_size
    step = defaultStep
    variation = 0.4  # FIXME : put it in config file, +- bounds on the step size
    if hasattr(planner, "pathId"):
        pathId = planner.pathId
    elif hasattr(planner, "pId"):
        pathId = planner.pId
    else:
        pathId = planner.ps.numberPaths() - 1
    while not success and it < maxIt:
        if it > 0:
            step = defaultStep + random.uniform(-variation, variation)
        #configs = getConfigsFromPath (planner.ps, planner.pathId, step)
        #getSurfacesFromPath(planner.rbprmBuilder, configs, surfaces_dict, planner.v, True, False)
        viewer = planner.v
        if not hasattr(viewer, "client"):
            viewer = None
        R, surfaces = getSurfacesFromGuideContinuous(planner.rbprmBuilder,
                                                     planner.ps,
                                                     planner.afftool,
                                                     pathId,
                                                     viewer if display_surfaces else None,
                                                     step,
                                                     useIntersection=True,
                                                     max_yaw=guide_max_yaw,
                                                     max_surface_area=max_surface_area)
        pb = gen_pb(planner.q_init, R, surfaces, ref_root_height)
        try:
            pb, coms, footpos, allfeetpos, res = solveL1(pb, surfaces, None)
            success = True
        except:
            logger.warning("## Planner failed at iter : %d with step length = %f", it, step)
        it += 1
    if not success:
        raise RuntimeError("planner always fail.")
    return pathId, pb, coms, footpos, allfeetpos, res


def runLPFromGuideScript(cfg):
    #the following script must produce a
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
    pathId, pb, coms, footpos, allfeetpos, res = solve(planner, cfg.GUIDE_STEP_SIZE, cfg.GUIDE_MAX_YAW,
                                                       cfg.MAX_SURFACE_AREA,  planner.rbprmBuilder.ref_height,
                                                       cfg.DISPLAY_SL1M_SURFACES)
    root_init = planner.ps.configAtParam(pathId, 0.001)[0:7]
    root_end = planner.ps.configAtParam(pathId, planner.ps.pathLength(pathId) - 0.001)[0:7]
    return RF, root_init, root_end, pb, coms, footpos, allfeetpos, res


def runLPScript(cfg):
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
    RF, root_init, root_end, pb, coms, footpos, allfeetpos, res = runLPFromGuideScript(cfg)
    multicontact_api.switchToNumpyArray()
    # load scene and robot
    fb, v = initScene(cfg.Robot, cfg.ENV_NAME, True)
    q_init = cfg.IK_REFERENCE_CONFIG.tolist() + [0] * 6
    q_init[0:7] = root_init
    feet_height_init = allfeetpos[0][2]
    logger.info("feet height initial = %s", feet_height_init)
    q_init[2] = feet_height_init + cfg.IK_REFERENCE_CONFIG[2]
    q_init[2] += EPS_Z
    #q_init[2] = fb.referenceConfig[2] # 0.98 is in the _path script
    if v:
        v(q_init)

    # init contact sequence with first phase : q_ref move at the right root pose and with both feet in contact
    # FIXME : allow to customize that first phase
    cs = ContactSequence(0)
    addPhaseFromConfig(fb, cs, q_init, [fb.rLegId, fb.lLegId])

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
        if cfg.SL1M_USE_ORIENTATION:
            quat0 = Quaternion(pb["phaseData"][pId]["rootOrientation"])
            if pId < len(pb["phaseData"]) - 1:
                quat1 = Quaternion(pb["phaseData"][pId + 1]["rootOrientation"])
            else:
                quat1 = Quaternion(pb["phaseData"][pId]["rootOrientation"])
            if cfg.SL1M_USE_INTERPOLATED_ORIENTATION :
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
        else:
            rot = Quaternion.Identity()
        placement = SE3()
        placement.translation = np.array(pos).T
        placement.rotation = rot.matrix()
        cs.moveEffectorToPlacement(movingID, placement)

    # final phase :
    # fixme : assume root is in the middle of the last 2 feet pos ...
    q_end = cfg.IK_REFERENCE_CONFIG.tolist() + [0] * 6
    #p_end = (allfeetpos[-1] + allfeetpos[-2]) / 2.
    #for i in range(3):
    #    q_end[i] += p_end[i]
    q_end[0:7] = root_end
    feet_height_end = allfeetpos[-1][2]
    logger.info("feet height final = %s", feet_height_end)
    q_end[2] = feet_height_end + cfg.IK_REFERENCE_CONFIG[2]
    q_end[2] += EPS_Z
    fb.setCurrentConfig(q_end)
    com = fb.getCenterOfMass()
    setFinalState(cs, com, q=q_end)
    if cfg.DISPLAY_CS_STONES:
        displaySteppingStones(cs, v.client.gui, v.sceneName, fb)

    return cs, fb, v
