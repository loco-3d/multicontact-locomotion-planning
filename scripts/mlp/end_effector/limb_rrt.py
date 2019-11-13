import mlp.config as cfg
import time
import os
import pinocchio as pin
from pinocchio import SE3
from pinocchio.utils import *
import numpy.linalg
from multicontact_api import WrenchCone,SOC6,ContactSequenceHumanoid
import numpy as np
import math
from mlp.utils.util import stdVecToMatrix, createStateFromPhase,effectorPositionFromHPPPath
import eigenpy
import hpp_bezier_com_traj as bezier_com
from hpp_spline import bezier
from mlp.utils.polyBezier import PolyBezier
import quadprog
eigenpy.switchToNumpyMatrix()
from mlp.utils import trajectories
from mlp.end_effector.bezier_predef import generatePredefBeziers,generateSmoothBezierTraj

VERBOSE = 1
DISPLAY_RRT_PATH = True
DISPLAY_JOINT_LEVEL = True
# order to try weight values and number of variables : 
weights_vars = [[0.5,bezier_com.ConstraintFlag.ONE_FREE_VAR,1],[0.75,bezier_com.ConstraintFlag.ONE_FREE_VAR,1],[0.85,bezier_com.ConstraintFlag.ONE_FREE_VAR,1],[0.90,bezier_com.ConstraintFlag.ONE_FREE_VAR,1],[0.95,bezier_com.ConstraintFlag.ONE_FREE_VAR,1],[1.,bezier_com.ConstraintFlag.ONE_FREE_VAR,1],
              [0.5,bezier_com.ConstraintFlag.THREE_FREE_VAR,3],[0.75,bezier_com.ConstraintFlag.THREE_FREE_VAR,3],[0.85,bezier_com.ConstraintFlag.THREE_FREE_VAR,3],[0.90,bezier_com.ConstraintFlag.THREE_FREE_VAR,3],[0.95,bezier_com.ConstraintFlag.THREE_FREE_VAR,3],[1.,bezier_com.ConstraintFlag.THREE_FREE_VAR,3],
              [0.5,bezier_com.ConstraintFlag.FIVE_FREE_VAR,5],[0.75,bezier_com.ConstraintFlag.FIVE_FREE_VAR,5],[0.85,bezier_com.ConstraintFlag.FIVE_FREE_VAR,5],[0.90,bezier_com.ConstraintFlag.FIVE_FREE_VAR,5],[0.95,bezier_com.ConstraintFlag.FIVE_FREE_VAR,5],[1.,bezier_com.ConstraintFlag.FIVE_FREE_VAR,5]]
# if numTry is equal to a number in this list, recompute the the limb-rrt path. This list is made such that low weight/num vars are tried for several limb-rrt path before trying higher weight/num variables
recompute_rrt_at_tries=[1,4,7,10, 16,23]
for i in range(20):
    recompute_rrt_at_tries+=[23+i*len(weights_vars)]
# store the last limbRRT path ID computed        
current_limbRRT_id = None

def effectorCanRetry():
    return True

#min (1/2)x' P x + q' x  
#subject to  G x <= h
#subject to  C x  = d
def quadprog_solve_qp(P, q, G=None, h=None, C=None, d=None):
    #~ qp_G = .5 * (P + P.T)   # make sure P is symmetric
    qp_G = .5 * (P + P.T)   # make sure P is symmetric
    qp_a = -q
    if C is not None:
        if G is not None:
                qp_C = -vstack([C, G]).T
                qp_b = -hstack([d, h])   
        else:
                qp_C = -C.transpose()
                qp_b = -d 
        meq = C.shape[0]
    else:  # no equality constraint 
        qp_C = -G.T
        qp_b = -h
        meq = 0
        
    if VERBOSE > 1:
        print "quadprog matrix size : "
        print "G : ",qp_G.shape
        print "a : ",qp_a.shape
        print "C : ",qp_C.shape
        print "b : ",qp_b.shape
        print "meq = ",meq        
        
    qp_G = np.array(qp_G)
    qp_a = np.array(qp_a).flatten()
    qp_C = np.array(qp_C)
    qp_b = np.array(qp_b).flatten()
    if VERBOSE > 1:
        print "quadprog array size : "
        print "G : ",qp_G.shape
        print "a : ",qp_a.shape
        print "C : ",qp_C.shape
        print "b : ",qp_b.shape
        print "meq = ",meq
        
    return quadprog.solve_qp(qp_G, qp_a, qp_C, qp_b, meq)[0]



def generateLimbRRTPath(q_init,q_end,phase_previous,phase,phase_next,fullBody) :
    assert fullBody and "Cannot use limb-rrt method as fullBody object is not defined."
    extraDof = int(fullBody.client.robot.getDimensionExtraConfigSpace())
    q_init = q_init.T.tolist()[0] + [0]*extraDof    
    q_end = q_end.T.tolist()[0] + [0]*extraDof
    if not fullBody.isConfigValid(q_init)[0]:
        print "q_init in limb-rrt : ",q_end        
        raise ValueError( "init config is invalid in limb-rrt.")
    if not fullBody.isConfigValid(q_end)[0]:
        print "q_end in limb-rrt : ",q_end
        raise ValueError( "goal config is invalid in limb-rrt.")   
    # create nex states in fullBody corresponding to given configuration and set of contacts 
    s0 = createStateFromPhase(fullBody,phase_previous,q_init)
    s1 = createStateFromPhase(fullBody,phase_next,q_end)
    if VERBOSE > 1: 
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
    fullBody.setCurrentConfig(fullBody.getConfigAtState(s0))
    com0_fb = fullBody.getCenterOfMass()
    fullBody.setCurrentConfig(fullBody.getConfigAtState(s1))    
    com1_fb = fullBody.getCenterOfMass()        
     
    ## TEST, FIXME (force com path to start/end in the com position found from q_init and q_end. :  
    c_t[0,:] = np.matrix(com0_fb)
    c_t[-1,:] = np.matrix(com1_fb)
    com0 = c_t.tolist()[0]
    com1 = c_t.tolist()[-1]      
    if VERBOSE > 1: 
        print "init com : ",com0_fb
        print "init ref : ",com0
        print "end  com : ",com1_fb
        print "end  ref : ",com1       
    
    path_com_id = fullBody.generateComTraj(c_t.tolist(), v_t.tolist(), a_t.tolist(), dt)
    if VERBOSE :
        print "add com reference as hpp path with id : ",path_com_id

    # project this states to the new COM position in phase :
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

    
    # run limb-rrt in hpp : 
    if VERBOSE : 
        print "start limb-rrt ... "
    paths_rrt_ids = fullBody.comRRTOnePhase(s0,s1,path_com_id,10)  
    if VERBOSE :
        print "Limb-rrt returned path(s) : ",paths_rrt_ids
    path_rrt_id= int(paths_rrt_ids[0])
    
    return path_rrt_id
    
    
def generateLimbRRTTraj(time_interval,placement_init,placement_end,numTry,q_t,phase_previous=None,phase=None,phase_next=None,fullBody=None,eeName=None,viewer=None):
    t_begin = cfg.EFF_T_PREDEF + cfg.EFF_T_DELAY
    t_end = time_interval[1]-time_interval[0] - t_begin
    q_init = q_t[:,int(t_begin/cfg.IK_dt)] # after the predef takeoff
    q_end = q_t[:,int(t_end/cfg.IK_dt)]       
    pathId = generateLimbRRTPath(q_init,q_end,phase_previous,phase,phase_next,fullBody)
    
    if viewer and cfg.DISPLAY_FEET_TRAJ and DISPLAY_RRT_PATH:
        from hpp.gepetto import PathPlayer
        pp = PathPlayer (viewer)
        pp.displayPath(pathId,jointName=fullBody.getLinkNames(eeName)[0])

    return trajectories.HPPEffectorTrajectory(eeName,fullBody,fullBody.client.problem,pathId)

def computeDistanceCostMatrices(fb,pathId,pData,T,eeName,numPoints = 50):
    problem = fb.client.problem
    # build a matrice 3xnumPoints by sampling the given path : 
    step = problem.pathLength(pathId)/(numPoints-1);
    pts = np.matrix(np.zeros([3,numPoints]))
    for i in range(numPoints):
        p = effectorPositionFromHPPPath(fb,problem,eeName,pathId,float(step*i))
        pts[:,i] = p
    return bezier_com.computeEndEffectorDistanceCost(pData,T,numPoints,pts)


def generateLimbRRTOptimizedTraj(time_interval,placement_init,placement_end,numTry,q_t=None,phase_previous=None,phase=None,phase_next=None,fullBody=None,eeName=None,viewer=None):
    if numTry == 0 :
        return generateSmoothBezierTraj(time_interval,placement_init,placement_end)
    else :
        if q_t is None or phase_previous is None or phase is None or phase_next is None or not fullBody or not eeName :
            raise ValueError("Cannot compute LimbRRTOptimizedTraj for try >= 1 without optionnal arguments")
    if cfg.EFF_T_PREDEF > 0 : 
        predef_curves = generatePredefBeziers(time_interval,placement_init,placement_end)
    else :
        predef_curves = generateSmoothBezierTraj(time_interval,placement_init,placement_end).curves
    id_middle = int(math.floor(len(predef_curves.curves)/2.))
    predef_middle = predef_curves.curves[id_middle]
    pos_init = predef_middle(0)
    pos_end = predef_middle(predef_middle.max())
    if VERBOSE :
        print "generateLimbRRTOptimizedTraj, try number "+str(numTry)
        print "bezier takeoff end : ",pos_init
        print "bezier landing init : ",pos_end
    t_begin = predef_curves.times[id_middle]
    t_middle =  predef_middle.max()
    t_end = t_begin + t_middle
    if VERBOSE : 
        print "t begin : ",t_begin
        print "t end   : ",t_end
    q_init = q_t[:,int(math.floor(t_begin/cfg.IK_dt))] # after the predef takeoff
    id_end = int(math.ceil(t_end/cfg.IK_dt))-1
    if id_end >= q_t.shape[1]: # FIXME : why does it happen ? usually it's == to the size when the bug occur
        id_end = q_t.shape[1]-1
    q_end = q_t[:,id_end]        
    global current_limbRRT_id
    # compute new limb-rrt path if needed:
    if not current_limbRRT_id or (numTry in recompute_rrt_at_tries):
        current_limbRRT_id = generateLimbRRTPath(q_init,q_end,phase_previous,phase,phase_next,fullBody)    
        if viewer and cfg.DISPLAY_FEET_TRAJ and DISPLAY_RRT_PATH:
            from hpp.gepetto import PathPlayer
            pp = PathPlayer (viewer)
            pp.displayPath(current_limbRRT_id,jointName=fullBody.getLinkNames(eeName)[0],offset =cfg.Robot.dict_offset[eeName].translation.T.tolist()[0] )        
            
    # find weight and number of variable to use from the numTry :
    for offset in reversed(recompute_rrt_at_tries):
        if numTry >= offset:
            id = numTry - offset
            break
    if VERBOSE:
        print "weights_var id = ",id
    if id >= len(weights_vars):
        raise ValueError("Max number of try allow to find a collision-end effector trajectory reached.")
    weight = weights_vars[id][0]
    varFlag = weights_vars[id][1]
    numVars = weights_vars[id][2]
    if VERBOSE : 
        print "use weight "+str(weight)+" with num free var = "+str(numVars)
    # compute constraints for the end effector trajectories : 
    pData = bezier_com.ProblemData() 
    pData.c0_ = predef_middle(0)
    pData.dc0_ = predef_middle.derivate(0,1)
    pData.ddc0_ = predef_middle.derivate(0,2)
    pData.j0_ = predef_middle.derivate(0,3)
    pData.c1_ = predef_middle(predef_middle.max())
    pData.dc1_ = predef_middle.derivate(predef_middle.max(),1)
    pData.ddc1_ = predef_middle.derivate(predef_middle.max(),2)
    pData.j1_ = predef_middle.derivate(predef_middle.max(),3)    
    pData.constraints_.flag_ = bezier_com.ConstraintFlag.INIT_POS | bezier_com.ConstraintFlag.INIT_VEL | bezier_com.ConstraintFlag.INIT_ACC | bezier_com.ConstraintFlag.END_ACC | bezier_com.ConstraintFlag.END_VEL | bezier_com.ConstraintFlag.END_POS | bezier_com.ConstraintFlag.INIT_JERK | bezier_com.ConstraintFlag.END_JERK | varFlag
    Constraints = bezier_com.computeEndEffectorConstraints(pData,t_middle)
    Cost_smooth = bezier_com.computeEndEffectorVelocityCost(pData,t_middle)
    Cost_distance = computeDistanceCostMatrices(fullBody,current_limbRRT_id,pData,t_middle,eeName)
    # formulate QP matrices :
    # _ prefix = previous notation (in bezier_com_traj)
    # min        x' H x + 2 g' x
    # subject to A*x <= b    
    _A = Constraints.A
    _b = Constraints.b
    _H = ((1.-weight)*Cost_smooth.A + weight*Cost_distance.A)
    _g = ((1.-weight)*Cost_smooth.b + weight*Cost_distance.b) 
    if VERBOSE > 1:
        print "A = ",_A
        print "b = ",_b
        print "H = ",_H
        print "h = ",_g
    """  
    _A = np.array(_A)
    _b = np.array(_b)
    _H = np.array(_H)
    _g = np.array(_g)
    """
    
    # quadprog notation : 
    #min (1/2)x' P x + q' x  
    #subject to  G x <= h
    #subject to  C x  = d
    G = _A
    h = _b.flatten().T # remove the transpose when working with array
    P = _H * 2.
    q = (_g *2.).flatten().T
    
    if VERBOSE > 1:
        print "G = ",G
        print "h = ",h
        print "P = ",P
        print "q = ",q
        print "Shapes : "
        print "G : ",G.shape
        print "h : ",h.shape
        print "P : ",P.shape
        print "q : ",q .shape       
        
    # solve the QP :
    solved = False
    try :
        res = quadprog_solve_qp(P,q,G,h)
        solved = True
    except ValueError, e:
        print "Quadprog error : "
        print e.message
        raise ValueError("Quadprog failed to solve QP for optimized limb-RRT end-effector trajectory, for try number "+str(numTry))              
    if VERBOSE:
        print "Quadprog solved."
    # build a bezier curve from the result of quadprog : 
    vars = np.split(res,numVars) 
    wps = bezier_com.computeEndEffectorConstantWaypoints(pData,t_middle) # one wp per column 
    if VERBOSE:
        print "Constant waypoints computed."    
    id_firstVar = 4 # depend on the flag defined above, but for end effector we always use this ones ... 
    i=id_firstVar
    for x in vars:
        wps[:,i] = np.matrix(x).T
        i +=1
    if VERBOSE:
        print "Variables waypoints replaced by quadprog results."      
    bezier_middle = bezier(wps,t_middle)    
    # create concatenation with takeoff/landing 
    curves = predef_curves.curves[::]
    curves[id_middle] = bezier_middle
    pBezier = PolyBezier(curves)
    if VERBOSE :
        print "time interval     = ",time_interval[1]-time_interval[0]
        print "polybezier length = ",pBezier.max()
    ref_traj = trajectories.BezierTrajectory(pBezier,placement_init,placement_end,time_interval)    
    return ref_traj        