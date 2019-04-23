from pinocchio import SE3
import numpy as np
from numpy import cross
from numpy.linalg import norm
from pinocchio import SE3, Quaternion
import mlp.config as cfg
from mlp.utils.trajectories import cubicSplineTrajectory,quinticSplineTrajectory
import math

def quatFromConfig(q):
    return Quaternion(q[6],q[3],q[4],q[5])
    

def distPointLine(p_l,x1_l,x2_l):
    p= np.matrix(p_l)
    x1= np.matrix(x1_l)
    x2= np.matrix(x2_l)
    return norm(cross(p-x1,p-x2)) / norm(x2-x1)
    
    

def findPhase(cs,t):
    phase0 = cs.contact_phases[0]
    phasel = cs.contact_phases[-1]
    if t <= phase0.time_trajectory[0]:
        return 0
    elif t >= phasel.time_trajectory[-1]:
        return len(cs.contact_phases)-1


    id = [k for k, phase in enumerate(cs.contact_phases) if t >= phase.time_trajectory[0] and t <= phase.time_trajectory[-1]]
    assert len(id)>=1 or len(id) <= 2
    if len(id) == 2:
        return id[1]
    else:
        return id[0]
    

def SE3toVec(M):
    v = np.matrix(np.zeros((12, 1)))
    for j in range(3):
        v[j] = M.translation[j]
        v[j + 3] = M.rotation[j, 0]
        v[j + 6] = M.rotation[j, 1]
        v[j + 9] = M.rotation[j, 2]
    return v

# TODO : def SE3FromVec(s):

def MotiontoVec(M):
    v = np.matrix(np.zeros((6, 1)))
    for j in range(3):
        v[j] = M.linear[j]
        v[j + 3] = M.angular[j]
    return v


def stdVecToMatrix(std_vector):
    if len(std_vector) == 0:
        raise Exception("std_vector is Empty")
    vec_l = []
    for vec in std_vector:
        vec_l.append(vec)

    res = np.hstack(tuple(vec_l))
    return res

## helper method to deal with StateVectorState and other exposed c++ vectors : 
# replace vec[i] with value if it already exist or append the value
def appendOrReplace(vec,i,value):
    assert len(vec) >= i , "There is an uninitialized gap in the vector."
    if i < len(vec) :
        vec[i] = value
    else :
        vec.append(value)

def numpy2DToList(m):
    l = []
    for i in range(m.shape[1]):
        l += [m[:,i].T.tolist()[0]]
    return l    

# assume that q.size >= 7 with root pos and quaternion(x,y,z,w)
def SE3FromConfig(q):
    placement = SE3.Identity()
    placement.translation = q[0:3]
    r = Quaternion(q[6,0],q[3,0],q[4,0],q[5,0])
    placement.rotation = r.matrix()
    return placement

# cfg.Robot.MRsole_offset.actInv(p0.RF_patch.placement)
# get the joint position for the given phase with the given effector name
# Note that if the effector is not in contact the phase placement may be uninitialized (==Identity)
def JointPatchForEffector(phase,eeName):
    if eeName == cfg.Robot.rfoot :
        patch = phase.RF_patch.copy()
        patch.placement = cfg.Robot.MRsole_offset.actInv(patch.placement)
    elif eeName == cfg.Robot.lfoot :
        patch = phase.LF_patch.copy()
        patch.placement = cfg.Robot.MLsole_offset.actInv(patch.placement)
    elif eeName == cfg.Robot.rhand :
        patch = phase.RH_patch.copy()
        patch.placement = cfg.Robot.MRhand_offset.actInv(patch.placement)
    elif eeName == cfg.Robot.lhand :
        patch = phase.LH_patch.copy()
        patch.placement = cfg.Robot.MLhand_offset.actInv(patch.placement)   
    else :
        raise Exception("Unknown effector name")
    return patch

def JointPlacementForEffector(phase,eeName):
    return JointPatchForEffector(phase,eeName).placement

def getContactPlacement(phase,eeName):
    if eeName == cfg.Robot.rfoot :
        return phase.RF_patch.placement
    elif eeName == cfg.Robot.lfoot :
        return phase.LF_patch.placement
    elif eeName == cfg.Robot.rhand :
        return phase.RH_patch.placement
    elif eeName == cfg.Robot.lhand :
        return phase.LH_patch.placement
    else :
        raise Exception("Unknown effector name")
    return patch


def isContactActive(phase,eeName):
    if eeName == cfg.Robot.rfoot :
        return phase.RF_patch.active
    elif eeName == cfg.Robot.lfoot :
        return phase.LF_patch.active
    elif eeName == cfg.Robot.rhand :
        return phase.RH_patch.active
    elif eeName == cfg.Robot.lhand :
        return phase.LH_patch.active
    else :
        raise Exception("Unknown effector name") 

def isContactEverActive(cs,eeName):
    for phase in cs.contact_phases:
        if eeName == cfg.Robot.rfoot :
            if phase.RF_patch.active:
                return True
        elif eeName == cfg.Robot.lfoot :
            if phase.LF_patch.active:
                return True
        elif eeName == cfg.Robot.rhand :
            if phase.RH_patch.active:
                return True
        elif eeName == cfg.Robot.lhand :
            if phase.LH_patch.active:
                return True
        else :
            raise Exception("Unknown effector name") 
    return False
    
def effectorPositionFromHPPPath(fb,problem,eeName,pid,t):
    q = problem.configAtParam(pid,t)
    # compute effector pos from q : 
    fb.setCurrentConfig(q)
    p = fb.getJointPosition(eeName)[0:3] 
    return np.matrix(p).T

def genAMTrajFromPhaseStates(t_init,t_end,init_state,final_state,init_control = None,final_control = None):
    # build Angular moment cubic spline : 
    am_init = init_state[6:9]
    am_end = final_state[6:9]
    if init_control is None:
        dAm_init = np.zeros([3,1])
    else:
        dAm_init = init_control[3:6]
    if final_control is None:
        dAm_end = np.zeros([3,1])      
    else:
        dAm_end = final_control[3:6]
    return cubicSplineTrajectory(t_init,t_end,am_init,dAm_init,am_end,dAm_end)
    
def genCOMTrajFromPhaseStates(t_init,t_end,init_state,final_state,init_control = None,final_control = None):
    # build quintic spline for the com position : 
    p_init = init_state[0:3]
    p_end = final_state[0:3]
    v_init = init_state[3:6]
    v_end = final_state[3:6]
    if init_control is None:
        a_init = np.zeros([3,1])
    else:
        a_init = init_control[0:3]
    if final_control is None:
        a_end = np.zeros([3,1])       
    else:
        a_end = final_control[0:3]
    return quinticSplineTrajectory(t_init,t_end,p_init,v_init,a_init,p_end,v_end,a_end)
    
# fill state_trajectory and control_trajectory in order to connect init_state and final_state of the phase
# use quintic spline for the com position and cubic spline for the angular momentum
# The state_trajectory and control_trajectory in phase should be empty
# and the time_trajectory should start and end at the correct timings
def genSplinesForPhase(phase,init_control = None, final_control = None):
    init_state = phase.init_state
    final_state = phase.final_state    
    t_init = phase.time_trajectory[0]
    t_end = phase.time_trajectory[-1]
    com_traj = genCOMTrajFromPhaseStates(t_init,t_end,init_state,final_state,init_control,final_control)
    am_traj = genAMTrajFromPhaseStates(t_init,t_end,init_state,final_state,init_control,final_control)
    
    # Iiterate from t_init to t_end and fill the trajectories : 
    dt = cfg.SOLVER_DT
    numStep = int(round((t_end - t_init)/dt))
    for i in range(numStep):
        t = t_init + float(i)*dt
        if t > t_end: # may happen due to numerical imprecision
            t = t_end
        c,dc,ddc = com_traj(t)
        L,dL = am_traj(t)
        state = np.matrix(np.zeros(9)).T
        control= np.matrix(np.zeros(6)).T
        state[0:3] = c
        state[3:6] = dc
        control[0:3] = ddc
        state[6:9] = L
        control[3:6] = dL
        appendOrReplace(phase.state_trajectory,i ,state)
        appendOrReplace(phase.control_trajectory,i, control)
        appendOrReplace(phase.time_trajectory,i , t)
        
    return phase


def copyPhaseContacts(phase_in,phase_out):
    phase_out.RF_patch = phase_in.RF_patch
    phase_out.LF_patch = phase_in.LF_patch
    phase_out.RH_patch = phase_in.RH_patch
    phase_out.LH_patch = phase_in.LH_patch




def createStateFromPhase(fullBody,phase,q = None):
    if q is None:
        q = hppConfigFromMatrice(fullBody.client.robot,phase.reference_configurations[0])
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

def hppConfigFromMatrice(robot,q_matrix):
    q = q_matrix.T.tolist()[0]
    extraDof = robot.getConfigSize() - q_matrix.shape[0]
    assert extraDof >= 0 , "Changes in the robot model happened."
    if extraDof > 0:
        q += [0]*extraDof
    return q

def phasesHaveSameConfig(p0,p1):
    assert len(p0.reference_configurations) > 0 and "CS object given to croc method should store one reference_configuration in each contact_phase"
    assert len(p1.reference_configurations) > 0 and "CS object given to croc method should store one reference_configuration in each contact_phase"
    return np.array_equal(p0.reference_configurations[0],p1.reference_configurations[0])
