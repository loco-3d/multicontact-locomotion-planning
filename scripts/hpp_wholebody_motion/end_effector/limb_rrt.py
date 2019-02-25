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
DISPLAY_RRT_PATH = True
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


def generateLimbRRTPath(q_init,q_end,phase_previous,phase,phase_next,fullBody,phaseId) :
    assert fullBody and "Cannot use limb-rrt method as fullBody object is not defined."
    assert phaseId%2 and "Can only generate limb-rrt for 'middle' phases, ID must be odd (check generation of the contact sequence in contact_sequence/rbprm.py"
    extraDof = int(fullBody.client.robot.getDimensionExtraConfigSpace())
    q_init = q_init.T.tolist()[0] + [0]*extraDof    
    q_end = q_end.T.tolist()[0] + [0]*extraDof

    # create nex states in fullBody corresponding to given configuration and set of contacts 
    s0 = createStateFromPhase(fullBody,q_init,phase_previous)
    s1 = createStateFromPhase(fullBody,q_end,phase_next)
    if VERBOSE : 
        print "New state added, q_init = ",q_init
        print "New state added, q_end = ",q_end
        contacts = fullBody.getAllLimbsInContact(s0)
        fullBody.setCurrentConfig(fullBody.getConfigAtState(s0))
        print "contact at init state : ",contacts
        for contact in contacts : 
            effName = cfg.Robot.dict_limb_joint[contact]
            print "contact position for joint "+str(effName)+" = "+str(fullBody.getJointPosition(effName)[0:3])
        contacts = fullBody.getAllLimbsInContact(s1)
        fullBody.setCurrentConfig(fullBody.getConfigAtState(s1))        
        print "contact at end  state : ",contacts
        for contact in contacts : 
            effName = cfg.Robot.dict_limb_joint[contact]
            print "contact position for joint "+str(effName)+" = "+str(fullBody.getJointPosition(effName)[0:3])
            
    
        
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
    successProj=fullBody.projectStateToCOM(s0,com0)
    assert successProj and "Error during projection of state"+str(s0)+" to com position : "+str(com0)
    successProj=fullBody.projectStateToCOM(s1,com1)
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
    paths_rrt_ids = fullBody.comRRTOnePhase(s0,s1,path_com_id,0)  
    if VERBOSE :
        print "Limb-rrt returned path(s) : ",paths_rrt_ids
    path_rrt_id= int(paths_rrt_ids[0])
    
    return path_rrt_id
    
    
def generateLimbRRTTraj(time_interval,placement_init,placement_end,q_init,q_end,phase_previous,phase,phase_next,fullBody,phaseId,eeName,viewer) :
    pathId = generateLimbRRTPath(q_init,q_end,phase_previous,phase,phase_next,fullBody,phaseId)
    
    if viewer and cfg.DISPLAY_FEET_TRAJ and DISPLAY_RRT_PATH:
        from hpp.gepetto import PathPlayer
        pp = PathPlayer (viewer)   
        pp.displayPath(pathId,jointName=fullBody.getLinkNames(eeName)[0])
        
    
    # TODO : make a HPPRefTraj object and return it
    return None