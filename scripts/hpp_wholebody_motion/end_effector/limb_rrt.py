import hpp_wholebody_motion.config as cfg
import time
import os
import pinocchio as se3
from pinocchio import SE3
from pinocchio.utils import *
import numpy.linalg
from locomote import WrenchCone,SOC6,ContactSequenceHumanoid
import numpy as np
import math
VERBOSE = True
  
def stdVecToMatrix(std_vector):
    vec_l = []
    for vec in std_vector:
        vec_l.append(vec)

    res = np.hstack(tuple(vec_l))
    return res

def createStateFromPhase(fullBody,q,phase):
    contacts = []
    if phase.RF_patch.active:
        contacts += [cfg.Robot.rLegId]
    if phase.LF_patch.active:
        contacts += [cfg.Robot.lLegId]
    if phase.RH_patch.active:
        contacts += [cfg.Robot.rArmId]
    if phase.LH_patch.active:
        contacts += [cfg.Robot.lArmId]
    return fullBody.createState(q,contacts)


# TODO get current q from tsid instead of doing a projection (and next q too ? )
def generateLimbRRTPath(time_interval,placement_init,placement_end,q_init,q_end,phase_previous,phase,phase_next,fullBody,phaseId,eeName,viewer) :
    assert fullBody and "Cannot use limb-rrt method as fullBody object is not defined."
    assert phaseId%2 and "Can only generate limb-rrt for 'middle' phases, ID must be odd (check generation of the contact sequence in contact_sequence/rbprm.py"
    extraDof = int(fullBody.client.robot.getDimensionExtraConfigSpace())
    q_init = q_init.T.tolist()[0] + [0]*extraDof    
    q_end = q_end.T.tolist()[0] + [0]*extraDof

    # get corresponding state ID in fullBody
    """
    s0 = int(math.floor(phaseId/2.))    
    s1 = s0 + 1 
    if VERBOSE : 
        print "run limb-rrt between states "+str(s0)+" and "+str(s1)
        print "init config : ",fullBody.getConfigAtState(s0)
        print "end config  : ",fullBody.getConfigAtState(s1)
    """    
    s0 = createStateFromPhase(fullBody,q_init,phase_previous)
    s1 = createStateFromPhase(fullBody,q_end,phase_next)
    if VERBOSE : 
        print "New state added, q_init = ",q_init
        print "New state added, q_end = ",q_end
    
        
    # create a path in hpp corresponding to the discretized trajectory in phase :
    dt = phase.time_trajectory[1] - phase.time_trajectory[0]
    state_traj = stdVecToMatrix(phase.state_trajectory).transpose()
    control_traj = stdVecToMatrix(phase.control_trajectory).transpose()  
    c_t = state_traj[:,:3]
    v_t = state_traj[:-1,3:6]
    a_t = control_traj[:-1,:3]
    path_com_id = fullBody.generateComTraj(c_t.tolist(), v_t.tolist(), a_t.tolist(), dt)
    if VERBOSE :
        print "add com reference as hpp path with id : ",path_com_id

    # project this states to the new COM position in phase :
    com0 = c_t.tolist()[0]
    com1 = c_t.tolist()[-1]
    """
    successProj=fullBody.projectStateToCOM(s0,com0,maxNumSample=1000)
    assert successProj and "Error during projection of state"+str(s0)+" to com position : "+str(com0)
    successProj=fullBody.projectStateToCOM(s1,com1,maxNumSample=1000)
    assert successProj and "Error during projection of state"+str(s1)+" to com position : "+str(com1)
    q_init = fullBody.getConfigAtState(s0)
    q_end = fullBody.getConfigAtState(s1)
    if extraDof: 
        q_init[-extraDof:] = [0]*extraDof #TODO : fix this in the projection method
        q_end[-extraDof:] = [0]*extraDof
        fullBody.setConfigAtState(s0,q_init)
        fullBody.setConfigAtState(s1,q_end)
    """
    if VERBOSE : 
        fullBody.setCurrentConfig(fullBody.getConfigAtState(s0))
        print "init com : ",fullBody.getCenterOfMass()
        print "ref      : ",com0
        fullBody.setCurrentConfig(fullBody.getConfigAtState(s1))
        print "end  com : ",fullBody.getCenterOfMass()
        print "ref      : ",com1        
    
    # run limb-rrt in hpp : 
    paths_rrt_ids = fullBody.effectorRRTOnePhase(s0,s1,path_com_id,0)  
    #paths_rrt_ids = fullBody.generateEffectorBezierArray(s0,s1,path_com_id,1) 
    path_rrt_id= paths_rrt_ids[0]
    
    if viewer and cfg.DISPLAY_FEET_TRAJ:
        from hpp.gepetto import PathPlayer
        pp = PathPlayer (fullBody.client, viewer)   
        pp.displayPath(path_rrt_id,jointName=eeName)
    
    