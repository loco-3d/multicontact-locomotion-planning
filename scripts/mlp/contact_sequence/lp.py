import pinocchio as pin
from pinocchio import SE3, Quaternion
from pinocchio.utils import *
import inspect
import mlp.config as cfg
import multicontact_api
from multicontact_api import ContactPhaseHumanoid, ContactSequenceHumanoid
from mlp.utils.util import quatFromConfig,copyPhaseContacts,copyPhaseContactPlacements,contactPatchForEffector
import importlib
from mlp.utils.cs_tools import *
import numpy as np
from numpy.linalg import norm
from mlp.viewer.display_tools import initScene, displaySteppingStones
from pinocchio.utils import matrixToRpy
from tools.surfaces_from_path import getSurfacesFromPath,getSurfacesFromGuideContinuous,getAllSurfacesDict
from tools.plot_surfaces import draw
from sl1m.constants_and_tools import *
from sl1m.planner import *

from numpy import asmatrix, matrix, zeros, ones
from numpy import array, dot, stack, vstack, hstack, asmatrix, identity, cross, concatenate
from numpy.linalg import norm
import random

import eigenpy
eigenpy.switchToNumpyMatrix()

Z_AXIS = np.array([0,0,1]).T
VERBOSE = False
USE_ORIENTATION = True
EPS_Z = 0.005 # offset added to feet z position, otherwise there is collisions with the ground

def normal(phase):
    s = phase["S"][0]
    n = cross(s[:, 1] - s[:, 0], s[:, 2] - s[:, 0])
    n /= norm(n)
    if n[2] < 0.:
        for i in range(3):
            n[i] = -n[i]
    # ~ print "normal ", n
    return n


def quatConfigFromMatrix(m):
    quat = Quaternion(m)
    return quatToConfig(quat)

def quatToConfig(quat):
    return [quat.x,quat.y,quat.z,quat.w]    

# FIXME : HARDCODED stuff for talos in this method ! 
def gen_pb(root_init,R, surfaces):
    #~ kinematicConstraints = genKinematicConstraints(min_height = 0.6)
    kinematicConstraints = genKinematicConstraintsTalos(min_height = None)
    relativeConstraints = genFootRelativeConstraintsTalos()
    nphases = len(surfaces)
    lf_0 = array(root_init[0:3]) + array([0, 0.085,-0.98]) # values for talos ! 
    rf_0 = array(root_init[0:3]) + array([0,-0.085,-0.98]) # values for talos ! 
    p0 = [lf_0,rf_0];
    print "p0 used : ",p0
    res = { "p0" : p0, "c0" : None, "nphases": nphases}
    #print "surfaces = ",surfaces
    print "number of surfaces : ",len(surfaces)
    #print "number of rotations values : ",len(R)
    #print "R= ",R
    #TODO in non planar cases, K must be rotated
    phaseData = [ {"moving" : i%2, "fixed" : (i+1) % 2 , "K" : [genKinematicConstraints(index = i, transform = R, min_height = 0.3) for _ in range(len(surfaces[i]))], "relativeK" : [genFootRelativeConstraints(index = i, transform = R)[(i) % 2] for _ in range(len(surfaces[i]))], "rootOrientation" : R[i], "S" : surfaces[i] } for i in range(nphases)]
    res ["phaseData"] = phaseData
    return res 


def solve(tp):
    from sl1m.fix_sparsity import solveL1
    #surfaces_dict = getAllSurfacesDict(tp.afftool)         
    success = False
    maxIt = 50
    it = 0
    defaultStep = cfg.GUIDE_STEP_SIZE
    step = defaultStep
    variation = 0.4 # FIXME : put it in config file, +- bounds on the step size
    while not success and it < maxIt:
        if it > 0 :
            step = defaultStep + random.uniform(-variation,variation)
        #configs = getConfigsFromPath (tp.ps, tp.pathId, step)  
        #getSurfacesFromPath(tp.rbprmBuilder, configs, surfaces_dict, tp.v, True, False)
        R,surfaces = getSurfacesFromGuideContinuous(tp.rbprmBuilder,tp.ps,tp.afftool,tp.pathId,tp.v,step,useIntersection=True)
        pb = gen_pb(tp.q_init,R,surfaces)
        try:
            pb, coms, footpos, allfeetpos, res = solveL1(pb, surfaces, None)
            success = True
        except :  
            print "## Planner failed at iter : "+str(it)+" with step length = "+str(step)
        it += 1
    if not success :
        raise RuntimeError("planner always fail.") 
    return pb, coms, footpos, allfeetpos, res

def runLPFromGuideScript():
    #the following script must produce a
    if hasattr(cfg, 'SCRIPT_ABSOLUTE_PATH'):
        scriptName = cfg.SCRIPT_ABSOLUTE_PATH
    else :
        scriptName = 'scenarios.'+cfg.SCRIPT_PATH+'.'+cfg.DEMO_NAME
    scriptName += "_path"
    print "Run Guide script : ",scriptName
    tp = importlib.import_module(scriptName)
    # compute sequence of surfaces from guide path
    pb, coms, footpos, allfeetpos, res = solve(tp)
    root_init = tp.q_init[0:7]
    return RF,root_init, pb, coms, footpos, allfeetpos, res    


def runLPScript():
    #the following script must produce a
    if hasattr(cfg, 'SCRIPT_ABSOLUTE_PATH'):
        scriptName = cfg.SCRIPT_ABSOLUTE_PATH
    else :
        scriptName = 'scenarios.'+cfg.SCRIPT_PATH+'.'+cfg.DEMO_NAME
    print "Run LP script : ",scriptName
    cp = importlib.import_module(scriptName)
    pb, coms, footpos, allfeetpos, res = cp.solve()
    root_init = cp.root_init[0:7]
    return cp.RF,root_init, pb, coms, footpos, allfeetpos, res

def generateContactSequence():
    #RF,root_init,pb, coms, footpos, allfeetpos, res = runLPScript()
    RF,root_init,pb, coms, footpos, allfeetpos, res = runLPFromGuideScript()

    # load scene and robot
    fb,v = initScene(cfg.Robot,cfg.ENV_NAME,False)
    q_init = fb.referenceConfig[::] + [0]*6
    q_init[0:7] = root_init
    q_init[2] += EPS_Z
    #q_init[2] = fb.referenceConfig[2] # 0.98 is in the _path script
    v(q_init)

    # init contact sequence with first phase : q_ref move at the right root pose and with both feet in contact
    # FIXME : allow to customize that first phase
    cs = ContactSequenceHumanoid(0)
    addPhaseFromConfig(fb, v, cs, q_init, [fb.rLegId, fb.lLegId])

    # loop over all phases of pb and add them to the cs :
    for pId in range(2, len(pb["phaseData"])): # start at 2 because the first two ones are already done in the q_init
        prev_contactPhase = cs.contact_phases[-1]
        #n = normal(pb["phaseData"][pId])
        phase = pb["phaseData"][pId]
        moving = phase["moving"]
        movingID = fb.lfoot
        if moving == RF:
            movingID = fb.rfoot
        pos = allfeetpos[pId] # array, desired position for the feet movingID
        pos[2] += EPS_Z # FIXME it shouldn't be required !! 
        # compute desired foot rotation :
        if USE_ORIENTATION:
            if pId < len(pb["phaseData"]) - 1:
                quat0 = Quaternion(pb["phaseData"][pId]["rootOrientation"])
                quat1 = Quaternion(pb["phaseData"][pId+1]["rootOrientation"])
                rot = quat0.slerp(0.5,quat1)
                # check if feets do not cross :
                if moving == RF:
                    qr = rot
                    ql = Quaternion(prev_contactPhase.LF_patch.placement.rotation)
                else:
                    ql = rot
                    qr = Quaternion(prev_contactPhase.RF_patch.placement.rotation)
                rpy = matrixToRpy((qr * (ql.inverse())).matrix())  # rotation from the left foot pose to the right one
                if rpy[2, 0] > 0: # yaw positive, feet are crossing
                    rot = Quaternion(phase["rootOrientation"])  # rotation of the root, from the guide
            else:
                rot = Quaternion(phase["rootOrientation"])  # rotation of the root, from the guide
        else:
            rot = Quaternion.Identity()
        placement = SE3()
        placement.translation = np.matrix(pos).T
        placement.rotation = rot.matrix()
        moveEffectorToPlacement(fb,v,cs,movingID,placement,initStateCenterSupportPolygon = True)
    # final phase :
    # fixme : assume root is in the middle of the last 2 feet pos ...
    q_end = fb.referenceConfig[::]+[0]*6
    p_end = (allfeetpos[-1] + allfeetpos[-2]) / 2.
    for i in range(3):
        q_end[i] += p_end[i]
    setFinalState(cs,q=q_end)

    displaySteppingStones(cs,v.client.gui,v.sceneName,fb)

    return cs,fb,v


