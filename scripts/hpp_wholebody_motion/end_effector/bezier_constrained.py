import hpp_wholebody_motion.config as cfg
import time
import os
from hpp_wholebody_motion.utils.polyBezier import *
import pinocchio as se3
from pinocchio import SE3
from pinocchio.utils import *
import numpy.linalg
from locomote import WrenchCone,SOC6,ContactSequenceHumanoid
import numpy as np
from numpy import array, zeros
from numpy.linalg import norm
from scipy.spatial import ConvexHull
from tools.disp_bezier import *
import hpp_spline
import hpp_bezier_com_traj as bezier_com
from hpp_wholebody_motion.utils import trajectories
import math
from tools.disp_bezier import *
import eigenpy
import quadprog
from numpy import array, dot, vstack, hstack, asmatrix, identity
import bezier_predef
import limb_rrt
eigenpy.switchToNumpyArray()


VERBOSE = True
DISPLAY = True
DISPLAY_RRT = True
DISPLAY_CONSTRAINTS = True



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
    return quadprog.solve_qp(qp_G, qp_a, qp_C, qp_b, meq)[0]


def writearray(f, a):
    for i in range(a.shape[0]):
        line = ""
        for j in range(a.shape[1]-1):
            line += str(a[i][j]) + " "
        line+= str(a[i][-1])+"\n"
        f.write(line)
    f.write("\n")

def saveProblem(pDef):
    f = open("test","w")
    # degree totaltime flag
    f.write(str(pDef.degree)+"\n")
    f.write(str(pDef.totalTime)+"\n")
    f.write(str(int(pDef.flag))+"\n")
    f.write("\n")
    writearray(f, pDef.start)
    writearray(f, pDef.end)
    writearray(f, pDef.splits)
    i = 0
    while(True):
        try:
            ineq = pDef.inequality(i)
            writearray(f, ineq.A)
            writearray(f, ineq.b)
            i+=1
        except:
            f.close()
            return
    f.write()
    f.close()
    

def toRotationMatrix(q):
    """
    Returns a (3*3) array (rotation matrix)
    representing the same rotation as the (normalized) quaternion.
    """
    rm=zeros((3,3))
    rm[0,0]=1-2*(q[2]**2+q[3]**2)
    rm[0,1]=2*q[1]*q[2]-2*q[0]*q[3]
    rm[0,2]=2*q[1]*q[3]+2*q[0]*q[2]
    rm[1,0]=2*q[1]*q[2]+2*q[0]*q[3]
    rm[1,1]=1-2*(q[1]**2+q[3]**2)
    rm[1,2]=2*q[2]*q[3]-2*q[0]*q[1]
    rm[2,0]=2*q[1]*q[3]-2*q[0]*q[2]
    rm[2,1]=2*q[2]*q[3]+2*q[0]*q[1]
    rm[2,2]=1-2*(q[1]**2+q[2]**2)
    return rm


def rot_quat_x(client,a):
    x = [1.,0.,0.]
    return client.rotationQuaternion(x,a)

def rot_mat_x(client,a):
    x = [1.,0.,0.]
    q = client.rotationQuaternion(x,a)
    return toRotationMatrix(q)



def display_box(viewer,client,a,b,y,z):
    ar_a = array(a)
    ar_b = array(b)
    x_len = norm(ar_b - ar_a)
    x_pos = ar_a + (ar_b  - ar_a) / 2
    rootName = "constraints_"
    list = viewer.client.gui.getNodeList()
    i=0
    name = rootName
    while list.count(name) > 0:
        name=rootName+"_"+str(i)
        i+=1       
    
    viewer.client.gui.addBox(name,x_len/ 2,y*x_len,z*x_len, [0.,1.,0.,0.3])
    config = x_pos.tolist()+rot_quat_x(client, ((ar_b - ar_a) / x_len).tolist() )
    viewer.client.gui.applyConfiguration(name,config)
    viewer.client.gui.addToGroup(name,viewer.sceneName)
    viewer.client.gui.refresh()


def to_ineq(client,a,b,y_r,z_r):
    a_r = array(a); b_r = array(b);
    normba = norm(b_r - a_r)
    x_dir = (b_r - a_r) / normba
    x_pos = a_r + (b_r  - a_r) / 2

    x = normba / 2.
    y = y_r * normba;
    z = z_r * normba;

    points = [ [x,-y,z], [x,-y,z], [-x,-y,z], [x,-y,-z], [x,y,-z], [x,y,z], [-x,y,z], [-x,y,-z] ]
    #transform
    rot = rot_mat_x(client,x_dir.tolist())
    points = [rot.dot(array(el)) + x_pos for el in points]
    ineq = ConvexHull(points).equations
    return ineq[:,:-1],-ineq[:,-1]



##############################################" CALCUL ##################################""

# a et b sont les extremites du rrt
# y le poids initial alloue a la largeur (si la distance ab vaut 1, initialement la largeur vaut y)
# z le poids initial alloue a la hauteur (si la distance ab vaut 1, initialement la hauteur vaut y)
# sizeObject dimension de securite de l'effecteur
def large_col_free_box(client,a,b,y = 0.3 ,z = 0.3, sizeObject = 0.05, margin = 0.):
    # margin distance is not so good, better expand and then reduce box
    # d abord o nessaie de trouver une boite sans collsion
    collision = True
    a_r = array(a); b_r = array(b); y_r = y; z_r = z
    x_dir = (b_r - a_r) / norm(b_r - a_r)  
    distance = 0
    maxiter = 100
    while(collision and maxiter > 0):
        maxiter = maxiter -1
        distance = client.isBoxAroundAxisCollisionFree(a_r.tolist(),b_r.tolist(),[y_r,z_r],margin)
        collision = not distance > 0
        if(collision):
            y_r = y_r* 0.5; z_r = z_r* 0.5 #todo handle penetration to be smarter
    if collision:
        print "failed"
        return -1
    # now for the largest box possible
    else:           
        maxiter = 100
        while(not collision and distance > 0.01 and maxiter > 0):
            maxiter = maxiter - 1
            #find meaning of distance
            x_len = norm(b_r - a_r)
            x_dir = (b_r - a_r) / x_len
            scale = x_len + (distance) /(2* x_len)
            x_pos = a_r + (b_r  - a_r) / 2
            tmp_a_r = (x_pos - x_dir * scale * x_len / 2.)
            tmp_b_r = (x_pos + x_dir * scale * x_len / 2.)                 
            distance2 = client.isBoxAroundAxisCollisionFree(tmp_a_r.tolist(),tmp_b_r.tolist(),[y_r,z_r],margin)                        
            collision = not distance2 > 0
            if not collision:
                break
            else:              
                if abs(distance2 - distance) < 0.01 or distance2 > distance:
                    break
                a_r = tmp_a_r[:]
                b_r = tmp_b_r[:]          
                distance = distance2
    # now we have reached maximum uniform scaling, so we play a bit along each axis.
    eps = 0.05



    maxiter = 20
    collision = False
    while(not collision and maxiter>0):
        maxiter =  maxiter -1;
        # start with b
        tmp_b_r = b_r + x_dir * eps
        # adapt scaling of y and z
        x_len = norm(b_r - a_r)
        tmp_y_r = (x_len * y_r) / (x_len + eps)
        tmp_z_r = (x_len * z_r) / (x_len + eps)
        distance = client.isBoxAroundAxisCollisionFree(a_r.tolist(),tmp_b_r.tolist(),[tmp_y_r,tmp_z_r],margin)                        
        collision = not distance > 0
        if collision:
            break
        else:
            b_r = tmp_b_r[:]
            y_r = tmp_y_r     
            z_r = tmp_z_r    
    maxiter = 20
    collision = False
    while(not collision  and maxiter>0):
        maxiter =  maxiter -1;
        # start with a
        tmp_a_r = a_r - x_dir * eps
        x_len = norm(b_r - a_r)
        tmp_y_r = (x_len * y_r) / (x_len + eps)
        tmp_z_r = (x_len * z_r) / (x_len + eps)
        distance = client.isBoxAroundAxisCollisionFree(tmp_a_r.tolist(),b_r.tolist(),[tmp_y_r,tmp_z_r],margin)                        
        collision = not distance > 0
        if collision:
            break
        else:
            a_r = tmp_a_r[:]  
            y_r = tmp_y_r     
            z_r = tmp_z_r       


    maxiter = 50
    collision = False
    while(not collision  and maxiter>0):
        maxiter =  maxiter -1;
        # start with a
        tmp_y_r = y_r + y_r * 0.05
        distance = client.isBoxAroundAxisCollisionFree(a_r.tolist(),b_r.tolist(),[tmp_y_r,z_r],margin)                        
        collision = not distance > 0
        if collision:
            break
        else:
            y_r = tmp_y_r     

    maxiter = 50
    collision = False
    while(not collision  and maxiter>0):
        maxiter =  maxiter -1;
        # start with a
        tmp_z_r = z_r + z_r * 0.05
        distance = client.isBoxAroundAxisCollisionFree(a_r.tolist(),b_r.tolist(),[tmp_y_r,z_r],margin)                        
        collision = not distance > 0
        if collision:
            break
        else:
            z_r = tmp_z_r     

    #removing offset

    a_r = (a_r + x_dir*sizeObject/2.).tolist()
    b_r = (b_r - x_dir*sizeObject/2.).tolist()        

    return (a_r, b_r, y_r, z_r), to_ineq(client,a_r, b_r, y_r, z_r)

    
def computeInequalitiesAroundLine(fullBody,p_from,p_to,viewer):
    if VERBOSE :
        print "compute constrained for segment : "+str(p_from)+" -> "+str(p_to)
    (a, b, y, z),(H,h) = large_col_free_box(fullBody.clientRbprm.rbprm,p_from,p_to)
    if DISPLAY_CONSTRAINTS :
        display_box(viewer,fullBody.clientRbprm.rbprm,a,b,y,z)
    return H,h.reshape(-1,1)

def effPosFromConfig(fullBody,q,eeName):
    fullBody.setCurrentConfig(q)
    p = fullBody.getJointPosition(eeName)
    return p[0:3]

def computeProblemConstraints(pData,fullBody,pathId,t,eeName,viewer):
    pDef = hpp_spline.problemDefinition()
    # set up problem definition : 
    pDef.flag =  int(hpp_spline.constraint_flag.INIT_POS) |  int(hpp_spline.constraint_flag.INIT_VEL) |int(hpp_spline.constraint_flag.INIT_ACC)|int(hpp_spline.constraint_flag.INIT_JERK) |int(hpp_spline.constraint_flag.END_POS) |  int(hpp_spline.constraint_flag.END_VEL) |int(hpp_spline.constraint_flag.END_ACC)|int(hpp_spline.constraint_flag.END_JERK)
    pDef.costFlag = hpp_spline.derivative_flag.VELOCITY    
    pDef.start = pData.c0_
    pDef.end = pData.c1_
    curveConstraints = hpp_spline.curve_constraints()
    curve_constraints.init_vel = pData.dc0_
    curve_constraints.init_acc = pData.ddc0_
    curve_constraints.init_jerk = pData.j0_
    curve_constraints.end_vel = pData.dc1_
    curve_constraints.end_acc = pData.ddc1_
    curve_constraints.end_jerk = pData.j1_
    pDef.curveConstraints = curveConstraints
    # get all the waypoints from the limb-rrt
    wps,t_paths = fullBody.client.problem.getWaypoints(pathId)
    # approximate the switching times (infos from limb-rrt)
    if len(t_norm)>2 :
        splits=[]
        t_ratio = t/t_paths[-1] # ratio between the imposed time of the bezier curve (t) and the "time" (a pseudo distance) of the solution of the rrt 
        for i in range(1,len(t_norm)-1):
            ti = t_norm[i]*t_ratio
            if ti > t:
                ti = t
            splits+= [ti]
            if len(splits) > 1:
                if splits[-1] == splits[-2] :
                    print "Error in bezier_constrained : two adjacent constrained have the same switch time !!"
        pDef.splits = np.array([splits])
        if VERBOSE:
            print "number of switch between constraints : ",len(splits)
            print "splits timings : ",splits
    else : 
        if VERBOSE :
            print "Only one constraint set for the whole trajectory."
    # compute constraints around each line of the limb-rrt solution : 
    q_from = wps[0]
    p_from = effPosFromConfig(fullBody,q_from,eeName)
    for i in range(1,len(wps)):
        q_to = wps[i]
        p_to = effPosFromConfig(fullBody,q_to,eeName)
        A,b = computeInequalitiesAroundLine(fullBody,p_from,p_to,viewer)
        if VERBOSE :
            print "Inequalities computed."
        pDef.addInequality(A,b)        
        p_from = p_to[::]
        
    return pDef


def generateConstrainedBezierTraj(time_interval,placement_init,placement_end,q_t,predefTraj,phase_previous,phase,phase_next,fullBody,phaseId,eeName,viewer):
    t_total = time_interval[1]-time_interval[0]
    predef_curves = predefTraj.curves
    bezier_takeoff = predef_curves.curves[predef_curves.idFirstNonZero()]
    bezier_landing = predef_curves.curves[predef_curves.idLastNonZero()]
    id_middle = int(math.floor(len(predef_curves.curves)/2.))
    bezier_mid_predef= predef_curves.curves[id_middle]
    pos_init = bezier_takeoff(bezier_takeoff.max())
    pos_end = bezier_landing(0)
    if VERBOSE :
        print "bezier takeoff end : ",pos_init
        print "bezier landing init : ",pos_end
    t_begin = predef_curves.times[id_middle]
    t_middle = bezier_mid_predef.max()
    t_end = t_begin + t_middle
    if VERBOSE : 
        print "t begin : ",t_begin
        print "t end   : ",t_end
    q_init = q_t[int(t_begin/cfg.IK_dt)]
    q_end = q_t[int(t_end/cfg.IK_dt)]

    # compute limb-rrt path : 
    pathId = limb_rrt.generateLimbRRTPath(q_init,q_end,phase_previous,phase,phase_next,fullBody,phaseId)
            
    if viewer and cfg.DISPLAY_FEET_TRAJ and DISPLAY_RRT:
        from hpp.gepetto import PathPlayer
        pp = PathPlayer (viewer)   
        pp.displayPath(pathId,jointName=fullBody.getLinkNames(eeName)[0])    
    
    # compute constraints for the end effector trajectories : 
    pData = bezier_com.ProblemData() 
    pData.c0_ = bezier_takeoff(bezier_takeoff.max())
    pData.dc0_ = bezier_takeoff.derivate(bezier_takeoff.max(),1)
    pData.ddc0_ = bezier_takeoff.derivate(bezier_takeoff.max(),2)
    pData.j0_ = bezier_takeoff.derivate(bezier_takeoff.max(),3)
    pData.c1_ = bezier_landing(0)
    pData.dc1_ = bezier_landing.derivate(0,1)
    pData.ddc1_ = bezier_landing.derivate(0,2)
    pData.j1_ = bezier_landing.derivate(0,3)    
    pData.constraints_.flag_ = bezier_com.ConstraintFlag.INIT_POS | bezier_com.ConstraintFlag.INIT_VEL | bezier_com.ConstraintFlag.INIT_ACC | bezier_com.ConstraintFlag.END_ACC | bezier_com.ConstraintFlag.END_VEL | bezier_com.ConstraintFlag.END_POS | bezier_com.ConstraintFlag.INIT_JERK | bezier_com.ConstraintFlag.END_JERK 

    
    # now compute additional inequalities around the rrt path : 
    pDef = computeProblemConstraints(pData,fullBody,pathId,t_middle,eeName,viewer)
    solved = False
    # loop and increase the number of variable control point until a solution is found
    flagData = pData.constraints_.flag_
    flags = [bezier_com.ConstraintFlag.ONE_FREE_VAR,None, bezier_com.ConstraintFlag.THREE_FREE_VAR,None,bezier_com.ConstraintFlag.FIVE_FREE_VAR]
    numVars = 1
    while not solved and numVars <=5:
        pData.constraints_.flag_ = flagData | flags[numVars-1]
        ineqEff = bezier_com.computeEndEffectorConstraints(pData,t_middle)
        Hg = bezier_com.computeEndEffectorCost(pData,t_middle)
        res = bezier_com.computeEndEffector(pData,t_middle) #only used for comparison/debug ?
        bezier_unconstrained = res.c_of_t  
        pDef.degree = bezier_unconstrained.degree
        ineqConstraints = hpp_spline.generate_problem(pDef)  
        
        # _ prefix = previous notation (in bezier_com_traj)
        # min        x' H x + 2 g' x
        # subject to A*x <= b
        _A = np.vstack([ineqEff.A,ineqConstraints.A])
        _b = np.vstack([ineqEff.b,ineqConstraints.b])
        _H = Hg.A
        _g = Hg.b
        #_H = ineqConstraints.cost.A
        #_g = ineqConstraints.cost.b
        
        # quadprog notation : 
        #min (1/2)x' P x + q' x  
        #subject to  G x <= h
        #subject to  C x  = d
        G = _A
        h = _b.reshape((-1))
        P = _H * 2.
        q = (_g *2.).flatten()
        try :
            if VERBOSE:
                print "try to solve quadprog with "+str(numVars)+" variable"
            res = quadprog_solve_qp(P,q,G,h)
            solved = True
        except ValueError:
            if VERBOSE:
                print "Failed."            
            numVars += 2
    if solved :
        if VERBOSE : 
            print "Succeed."
    else :
        print "Constrained End effector Bezier method failed for all numbers of variables control points."
        # TODO : what to do here ?? 
    
    # retrieve the result of quadprog and create a bezier curve : 
    vars = np.split(res,numVars) 
    wps = bezier_unconstrained.waypoints()
    id_firstVar = 4
    i=id_firstVar
    for x in vars:
        wps[:,i] = x
        i +=1
    
    bezier_middle = hpp_spline.bezier(wps,t_middle)    
    # create concatenation with takeoff/landing 
    curves = []
    curves.append(bezier_takeoff)
    curves.append(bezier_middle)
    curves.append(bezier_landing)
    pBezier = PolyBezier(curves)
    if VERBOSE :
        print "time interval     = ",time_interval[1]-time_interval[0]
        print "polybezier length = ",pBezier.max()
    ref_traj = trajectories.BezierTrajectory(pBezier,placement_init,placement_end,time_interval)    
    return ref_traj    