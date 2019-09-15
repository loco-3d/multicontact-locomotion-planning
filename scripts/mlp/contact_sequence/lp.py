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
VERBOSE = False
USE_ORIENTATION = True

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
    RF,root_init,pb, coms, footpos, allfeetpos, res = runLPScript()

    # load scene and robot
    fb,v = initScene(cfg.Robot,cfg.ENV_NAME,False)
    q_init = fb.referenceConfig[::] + [0]*6
    q_init[0:7] = root_init
    #q_init[2] += fb.referenceConfig[2] - 0.98 # 0.98 is in the _path script
    v(q_init)

    # init contact sequence with first phase : q_ref move at the right root pose and with both feet in contact
    # FIXME : allow to customize that first phase
    cs = ContactSequenceHumanoid(0)
    addPhaseFromConfig(fb, v, cs, q_init, [fb.rLegId, fb.lLegId])

    # loop over all phases of pb and add them to the cs :
    for pId in range(2, len(pb["phaseData"])): # start at 2 because the first two ones are already done in the q_init
        #n = normal(pb["phaseData"][pId])
        phase = pb["phaseData"][pId]
        moving = phase["moving"]
        movingID = fb.lfoot
        if moving == RF:
            movingID = fb.rfoot
        pos = allfeetpos[pId] # array, desired position for the feet movingID
        pos[2] += 0.005 # FIXME it shouldn't be required !! 
        # compute desired foot rotation :
        if USE_ORIENTATION:
            if pId < len(pb["phaseData"]) - 1:
                quat0 = Quaternion(pb["phaseData"][pId]["rootOrientation"])
                quat1 = Quaternion(pb["phaseData"][pId+1]["rootOrientation"])
                rot = quat0.slerp(0.5,quat1)
            else:
                rot = Quaternion(phase["rootOrientation"])  # rotation of the root, from the guide
        else:
            rot = Quaternion.Identity()
        placement = SE3()
        placement.translation = np.matrix(pos).T
        placement.rotation = rot.matrix()
        moveEffectorToPlacement(fb,v,cs,movingID,placement)
    # final phase :
    # fixme : assume root is in the middle of the last 2 feet pos ...
    q_end = fb.referenceConfig[::]+[0]*6
    p_end = (allfeetpos[-1] + allfeetpos[-2]) / 2.
    for i in range(3):
        q_end[i] += p_end[i]
    setFinalState(cs,q=q_end)

    displaySteppingStones(cs,v.client.gui,v.sceneName,fb)

    return cs,fb,v


