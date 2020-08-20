try:
    import sl1m.planner as sl1m
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


def foot_pose_from_guide(Robot, root_pos):
    return [array(root_pos[0:3]) + Robot.dict_ref_effector_from_root[limb_name] +
            Robot.dict_offset[Robot.dict_limb_joint[limb_name]].translation
            for limb_name in Robot.limbs_names]
    # FIXME: apply epsz along the normal


def initial_foot_pose_from_fullbody(fullbody, q_init):
    fullbody.setCurrentConfig(q_init)
    ee_names = [fullbody.dict_limb_joint[limb] for limb in fullbody.limbs_names]
    return [array(fullbody.getJointPosition(ee_name)[:3]) +
            fullbody.dict_offset[ee_name].translation
            for ee_name in ee_names]
    # FIXME: apply epsz along the normal


def gen_pb(Robot, initial_contacts, R, surfaces):
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
def com_in_limb_effector_frame_constraint(Robot, transform, limbId):
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
def foot_in_limb_effector_frame_constraint(Robot, transform, limbId, footId):
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

def genCOMConstraints(Robot, rotation, normals):
    return [com_in_limb_effector_frame_constraint(Robot, sl1m.default_transform_from_pos_normal_(rotation, sl1m.zero3, normals[idx]),idx)
            for idx in range(len(Robot.limbs_names))]

def genRelativeConstraints(Robot, rotation, normals):
    transforms = [sl1m.default_transform_from_pos_normal_(rotation, sl1m.zero3, normals[idx])
                  for idx in range(len(Robot.limbs_names))]
    res = []
    for limbId, transform in enumerate(transforms):
        res += [[(footId, foot_in_limb_effector_frame_constraint(Robot, transform, limbId, footId))
                 for footId in range(len(Robot.limbs_names)) if footId != limbId]]
    return res


def gen_pb_mip(Robot, R, surfaces):
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
    if cfg.SL1M_USE_MIP:
        num_eff = len(cfg.Robot.limbs_names) # number of effectors used to create contacts
        global __ineq_com
        global __ineq_relative
        __ineq_com = [None for _ in range(num_eff)]
        __ineq_relative = [[None for _ in range(num_eff - 1)] for __ in range(num_eff)]
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
                                                     max_surface_area=cfg.MAX_SURFACE_AREA,
                                                     use_all_limbs = cfg.SL1M_USE_MIP)
        if cfg.SL1M_USE_MIP:
            from sl1m.planner_l1_generic_equalities_as_ineq import solveMIPGurobi, initGlobals, posturalCost, targetCom, \
                retrieve_points_from_res, targetLegCenter, targetEndPos
            initGlobals(nEffectors=num_eff)
            pb = gen_pb_mip(cfg.Robot, R, surfaces)
            initCom = np.array(planner.q_init[0:3])
            endCom = np.array(planner.q_goal[0:3])
            pb, res, time = solveMIPGurobi(pb, surfaces, MIP=True, draw_scene=None, plot=True, l1Contact=False,
                                           initPos=initial_contacts, endPos=final_contacts,
                                           initCom=initCom, endCom=endCom,
                                           costs=[(1., posturalCost), (1., targetEndPos)],
                                           constraint_init_pos_surface = False)
            coms, footpos, allfeetpos = retrieve_points_from_res(pb, res)
            success = True # FIXME check this from mip outputs ?
        else:
            from sl1m.fix_sparsity import solveL1
            pb = gen_pb(cfg.Robot, initial_contacts, R, surfaces)
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
    pathId, pb, coms, footpos, allfeetpos, res = solve(planner, cfg, cfg.DISPLAY_SL1M_SURFACES)
    root_init = planner.ps.configAtParam(pathId, 0.001)[0:7]
    root_end = planner.ps.configAtParam(pathId, planner.ps.pathLength(pathId) - 0.001)[0:7]
    return sl1m.RF, root_init, root_end, pb, coms, footpos, allfeetpos, res


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
    if cfg.SL1M_USE_MIP:
        # if MIP is used, allfeetpos contains a list of all position and not just the new position
        feet_height_init = allfeetpos[0][0][2]
    else:
        feet_height_init = allfeetpos[0][2]
    logger.info("feet height initial = %s", feet_height_init)
    q_init[2] = feet_height_init + cfg.IK_REFERENCE_CONFIG[2]
    q_init[2] += EPS_Z
    #q_init[2] = fb.referenceConfig[2] # 0.98 is in the _path script
    if v:
        v(q_init)

    if cfg.SL1M_USE_MIP:
        cs = build_cs_from_sl1m_mip(fb, cfg.IK_REFERENCE_CONFIG, root_end, pb, RF, allfeetpos, coms,
                                cfg.SL1M_USE_ORIENTATION, cfg.SL1M_USE_INTERPOLATED_ORIENTATION, q_init=q_init)
    else:
        cs = build_cs_from_sl1m(fb, cfg.IK_REFERENCE_CONFIG, root_end, pb, RF, allfeetpos,
                            cfg.SL1M_USE_ORIENTATION, cfg.SL1M_USE_INTERPOLATED_ORIENTATION, q_init = q_init)

    if cfg.DISPLAY_CS_STONES:
        displaySteppingStones(cs, v.client.gui, v.sceneName, fb)

    return cs, fb, v

def compute_orientation_for_feet_placement(fb, pb, pId, moving, RF, prev_contactPhase, use_interpolated_orientation):
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


def build_cs_from_sl1m(fb, q_ref, root_end, pb, RF, allfeetpos, use_orientation, use_interpolated_orientation,
                       q_init = None, first_phase = None):
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

def build_cs_from_sl1m_mip(fb, q_ref, root_end, pb, RF, allfeetpos, coms, use_orientation, use_interpolated_orientation,
                       q_init = None, first_phase = None):
    # init contact sequence with first phase : q_ref move at the right root pose and with both feet in contact
    # FIXME : allow to customize that first phase
    num_steps = len(pb["phaseData"]) - 1 # number of contact repositionning
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
    previous_eff_placements = allfeetpos[0]
    if len(previous_eff_placements) != num_effectors:
        raise NotImplementedError("A phase in the output of SL1M do not have all the effectors in contact.")
    for pid, eff_placements in enumerate(allfeetpos[1:]):
        logger.info("Loop allfeetpos, id = %d", pid)
        if len(eff_placements) != num_effectors:
            raise NotImplementedError("A phase in the output of SL1M do not have all the effectors in contact.")
        switch = False # True if a repostionning have been detected
        for k, pos in enumerate(eff_placements):
            if norm(pos - previous_eff_placements[k]) > 1e-4:
                if switch:
                    raise NotImplementedError("Several contact changes between two adjacent phases in SL1M output")
                switch = True
                ee_name = fb.dict_limb_joint[limbs_names[k]]
                logger.info("Move effector %s ", ee_name)
                logger.info("To position %s ", pos)
                placement = SE3.Identity()
                placement.translation = pos
                # TODO compute rotation from guide here !
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
