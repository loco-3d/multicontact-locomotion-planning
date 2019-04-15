import mlp.config as cfg
import time
import os
from mlp.utils.polyBezier import *
import pinocchio as pin
from pinocchio import SE3, Quaternion
from pinocchio.utils import *
import numpy.linalg
from locomote import WrenchCone,SOC6,ContactSequenceHumanoid
import numpy as np
from numpy import array, dot, vstack, hstack, asmatrix, identity
from numpy.linalg import norm, inv
from scipy.spatial import ConvexHull
from tools.disp_bezier import *
import hpp_spline
import hpp_bezier_com_traj as bezier_com
from mlp.utils import trajectories
import math
from tools.disp_bezier import *
import eigenpy
import quadprog
import bezier_predef
import limb_rrt
from mlp.utils.util import  SE3FromConfig,distPointLine
eigenpy.switchToNumpyArray()


VERBOSE = True
DISPLAY = True
DISPLAY_RRT = True
DISPLAY_CONSTRAINTS = True
DISPLAY_JOINT_LEVEL = False

# id of each triangles, to be used with a points list of the form :
# points = [ [x,-y,-z], [x,-y,z], [-x,-y,z], [-x,-y,-z], [x,y,-z], [x,y,z], [-x,y,z], [-x,y,-z] ]
# see (hpp-rbprm-corba/src/hpp/rbprm/rbprmBuilder.impl.cc : MeshObstacleBox
box_points = [  # front
    [-1.0, -1.0,  1.0,],
    [ 1.0, -1.0,  1.0,],
    [ 1.0,  1.0,  1.0,],
    [-1.0,  1.0,  1.0,],
    # back
    [-1.0, -1.0, -1.0,],
    [ 1.0, -1.0, -1.0,],
    [ 1.0,  1.0, -1.0,],
    [-1.0,  1.0, -1.0 ]]
triangles_ids = [
        # front
		[0, 1, 2,],
		[2, 3, 0,],
		# right
		[1, 5, 6,],
		[6, 2, 1,],
		# back
		[7, 6, 5,],
		[5, 4, 7,],
		# left
		[4, 0, 3,],
		[3, 7, 4,],
		# bottom
		[4, 5, 1,],
		[1, 0, 4,],
		# top
		[3, 2, 6,],
		[6, 7, 3]]


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
    

def rot_quat_x(client,a):
    x = [1.,0.,0.]
    return Quaternion(np.matrix(x).T,np.matrix(a).T).coeffs().T.tolist()[0]

def rot_mat_x(client,a):
    x = [1.,0.,0.]
    return Quaternion(np.matrix(x).T,np.matrix(a).T).matrix()



def display_box(viewer,points,groupName):
    boxRootName = groupName+"/box"
    list = viewer.client.gui.getNodeList()
    i=0
    boxName = boxRootName+"_"+str(i)
    while list.count(boxName+"/tri_0") > 0:
        i+=1               
        boxName=boxRootName+"_"+str(i)
        
    for tId in range(len(triangles_ids)):
        name = boxName+"/tri_"+str(tId)
        viewer.client.gui.addTriangleFace(name,points[triangles_ids[tId][0]].tolist(),points[triangles_ids[tId][1]].tolist(),points[triangles_ids[tId][2]].tolist(),[0.,1.,0.,0.3])
        viewer.client.gui.addToGroup(name,groupName)

def to_ineq(points):
    ineq = ConvexHull(points).equations
    return ineq[:,:-1],-ineq[:,-1]



##############################################" CALCUL ##################################""
"""
# a et b sont les extremites du rrt
# y le poids initial alloue a la largeur (si la distance ab vaut 1, initialement la largeur vaut y)
# z le poids initial alloue a la hauteur (si la distance ab vaut 1, initialement la hauteur vaut y)
# sizeObject dimension de securite de l'effecteur
def large_col_free_box(client,a,b,y = 0.3 ,z = 0.3, sizeObject = [0.,0.,0.], margin = 0.):
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
    print "a_r = ",a_r
    print "b_r = ",b_r
    print "x_dir = ",x_dir
    a_r = (a_r + x_dir*sizeObject/2.).tolist()
    b_r = (b_r - x_dir*sizeObject/2.).tolist()        

    return (a_r, b_r, y_r, z_r)
"""

def dimWithSizeObject(dim,size):
    res = []
    for k in range(3):
        for i in range(2):
            res += [dim[2*k+i] + size[k]]
    return res

# TODO : sizeObject not taken in account
def computeBoxVertices(client,center,x_dir,dim,sizeObject):
    # compute the vertices of this box: 
    #points = [ [dim[1],-dim[2],-dim[4]], [dim[1],-dim[2],dim[5]], [-dim[0],-dim[2],dim[5]], [-dim[0],-dim[2],-dim[4]], [dim[1],dim[3],-dim[4]], [dim[1],dim[3],dim[5]], [-dim[0],dim[3],dim[5]], [-dim[0],dim[3],-dim[4]] ]    
    points = []
    for sign in box_points : 
        point = []
        for i in range(3):
            if sign[i] < 0 : # required because dimensions are not symetrical
                point += [-dim[i*2]]
            else :
                point += [dim[i*2+1]]        
        points += [point]
    # transform this points to the position/orientation of the box : 
    rot = rot_mat_x(client,x_dir.tolist())
    t_c_w = SE3.Identity() # transform center of box in world frame
    t_c_w.translation=np.matrix(center).T
    t_c_w.rotation = rot
    pointsTransform = []
    for p in points:
        t_p_c = SE3.Identity() # vertice position in box frame
        t_p_c.translation = np.matrix(p).T
        pointsTransform += [t_c_w.act(t_p_c).translation.T[0]] 
        
    return pointsTransform


# a et b sont les extremites du rrt
# x_r,y_r,z_r : ratio of the maximal size of the box along each axis (x = direction of the segment)
def large_col_free_box(client,a,b,maxX = 0.2,maxY = 0.05 ,maxZ = 0.05, sizeObject = [0,0,0],margin = 0.):
    a_r = np.array(a)
    b_r = np.array(b)
    center = (b_r+a_r) / 2.
    x = norm(b_r - a_r)
    x_dir = (b_r -a_r)/x
    MIN_VALUE = 0.001    
    x = (x/2.) + MIN_VALUE # initial half-length (+margin)
    x_origin = x
    """
    # transform the sizeObject from the world frame to the x_dir one (ignoring value on z axis):
    size_r = np.array(sizeObject)    
    objPoints = []
    for sign in box_points :         
        objPoints += [np.array(sign)*size_r]   
    inv_rot = inv(rot_mat_x(client,x_dir.tolist()))    
    objPoints = [inv_rot.dot(array(el)) for el in objPoints]
    # look for the max displacement in the objBox along each axis and save it as the offset for this axis
    size_r = np.array([0.]*3)
    for p in objPoints:
        for i in range(3):
            if abs(p[i]) > size_r[i]:
                size_r[i] = abs(p[i])            
    if VERBOSE : 
        print "x_dir = ",x_dir.tolist()
        #print "obj points after transform : ",objPoints
        print "sizeObject after  transform : ",size_r.tolist()
    """    
    maxs = [maxX,maxX,maxY,maxY,maxZ,maxZ]
    
    if VERBOSE : 
        print "compute constraints for segment : "+str(a)+" -> "+str(b)
        print "center = ",center
        print "x init = ",x
        print "direction : ",x_dir
        print "maxs = ",maxs
    x_max = maxs[0] # initial max bounds of dichotomy 
    y_max = maxs[2]
    z_max = maxs[4]
    y_prev = 0.001 # initial min bounds of dichotomy
    z_prev = 0.001
    x_prev = x # original lenght of the rrt solution
    
    collision = not client.isBoxAroundAxisCollisionFree(center.tolist(),x_dir.tolist(),[x,x,y_prev,y_prev,z_prev,z_prev],sizeObject,margin)
    #assert not collision and "initial box is already in collision, limb-rrt path cannot be approximated by a straight line."    
    if collision :
        print "!! initial box is already in collision, limb-rrt path cannot be approximated by a straight line." 
        return computeBoxVertices(client,center,x_dir,[x,x,y_prev,y_prev,z_prev,z_prev],sizeObject)
    # By dichotomy : find the maximal value for y,z with a uniform scaling leading to a collision-free box
    it = 0
    found = False
    eps = 0.01
    while not found and it < 100:
        y = (y_max - y_prev)/2. + y_prev
        z = (z_max - z_prev)/2. + z_prev
        #print "try "+str(y)+" , "+str(z)
        collision = not client.isBoxAroundAxisCollisionFree(center.tolist(),x_dir.tolist(),[x,x,y,y,z,z],sizeObject,margin)
        if collision :
            #print "collision"
            y_max = y
            z_max = z
        else :
            #print "no collision"
            if y-y_prev <= eps and z-z_prev <= eps :
                #print "end dichotomy"
                found = True
            else :
                y_prev = y
                z_prev = z
        it += 1
        
    if VERBOSE : 
        print "after dichotomy for uniform scaling : "
        print "y = ",y
        print "z = ",z
    
    #Now we do the same along x axis : 
    found = True
    it = 0
    while not found and it < 100:
        x = (x_max - x_prev)/2. + x_prev
        collision = not client.isBoxAroundAxisCollisionFree(center.tolist(),x_dir.tolist(),[x,x,y,y,z,z],sizeObject,margin)
        if collision :
            x_max = x
        else :
            if x-x_prev <= eps  :
                found = True
            else :
                x_prev = x
        it += 1
        
    if VERBOSE : 
        print "x = ",x
       
    # now we try iteratively in each direction (non symetric) :
    success = True
    it = 0
    step = 0.01
    eps = 0.001
    dim = [x,x,y,y,z,z]
    
    while success and it < 100 :
        success = False # true if at least one axis have been made successfully bigger
        for i in range(6):
            if dim[i] < maxs[i] :
                dim[i] += step
                collision = not client.isBoxAroundAxisCollisionFree(center.tolist(),x_dir.tolist(),dim,sizeObject,margin)
                if collision :
                    dim[i] -= step
                else :
                    success = True
        it += 1
        if not success and step > eps: #reduce the step
            success = True
            step /= 10.
            
    if VERBOSE : 
        print "dimensions after iterative resizing : ",dim
    """    
    # reduce dimensions according to sizeObject :
    for k in range(3):
        for i in range(2):
            dim[k*2+i] -= size_r[k]
            # assure that the initial path do not go outside the box after resizing : 
            if k == 0 :
                if dim[i] <x_origin:
                    dim[i] = x_origin             
            elif dim[k*2+i] <MIN_VALUE:
                dim[k*2+i] = MIN_VALUE
               
    if VERBOSE : 
        print "dimensions after applying sizeObject : ",dim
    """    
    points = computeBoxVertices(client,center,x_dir,dim,sizeObject)
    if VERBOSE :
        print "final points list : ",points
    return points


### compute the inequalities constraints around the given line
# The line (p_from, p_to) must be expressed at the contact position
# but the constraints (H,h) are expressed at the joint level
# the display is done before the transform and thus show the constraints at the contact level
def computeInequalitiesAroundLine(fullBody,p_from,p_to,eeName,groupName,viewer):
    a = p_from.translation.T.tolist()[0]
    b = p_to.translation.T.tolist()[0]
    # size of the end effector (-x,x,-y,y,-z,z)
    size_diagonal = math.sqrt(cfg.Robot.dict_size[eeName][0]**2 + cfg.Robot.dict_size[eeName][1]**2)  #TODO margin ??
    #size = [size_diagonal/2., size_diagonal/2.,0.001]
    size = [-cfg.Robot.dict_size[eeName][0]/2.,cfg.Robot.dict_size[eeName][0]/2.,
            -cfg.Robot.dict_size[eeName][1]/2.,cfg.Robot.dict_size[eeName][1]/2.,
            -0.001,0.001]
    #size=[0,0,0] #FIXME debug    
    """
    size_r = np.array(size)
    # rotate size vector according to initial rotation of the effector :
    if VERBOSE : 
        print "rotation init : ",p_from.rotation
    size_r = p_from.rotation.dot(size_r)
    """
    points = large_col_free_box(fullBody.clientRbprm.rbprm,a,b,sizeObject=size)
    # Display the box before the size reduction : 
    #if DISPLAY_CONSTRAINTS and not DISPLAY_JOINT_LEVEL:
    #    display_box(viewer,points,groupName)
    
    """
    pointsReduced = []
    rot_init = p_from.rotation
    for i in range(len(box_points)):
        #pointsReduced += [points[i]-rot_init.dot((size_r*array(box_points[i]))/2.)]
        if VERBOSE :
            print "for point "+str(i)+" shift of "+str(-((size_r*np.array(box_points[i]))))
        pointsReduced += [points[i]-((size_r*np.array(box_points[i])))]
    """    
    # display the box after size reduction 
    if DISPLAY_CONSTRAINTS and not DISPLAY_JOINT_LEVEL:
        display_box(viewer,points,groupName)
    # transform the points back to joint level
    pc = SE3.Identity() # take Identity rotation #FIXME probably not the best idea ... 
    pointsJoint = []
    for point in points :
        pc.translation = np.matrix(point).T
        pointsJoint += [cfg.Robot.dict_offset[eeName].actInv(pc).translation.T.tolist()[0]]
    if DISPLAY_CONSTRAINTS and DISPLAY_JOINT_LEVEL:
        display_box(viewer,pointsJoint,groupName)
    H,h = to_ineq(pointsJoint)
    return H,h.reshape(-1,1)

def contactPlacementFromConfig(fullBody,q,eeName):
    fullBody.setCurrentConfig(q)
    q = fullBody.getJointPosition(eeName)
    p = SE3FromConfig(q)
    # transform to contact position (from joint position)
    # p*=cfg.Robot.dict_offset[eeName] # test ??
    tr = p.translation + cfg.Robot.dict_offset[eeName].translation
    p.translation = tr
    return p

# filter wp : remove "useless" waypoint. 
# compute the distance betwen w2 and the segment w1->w3, if inferior to a treshold, w2 is deleted from the list
def filterWPs(fullBody,eeName,wps,t):
    EPS = 0.01
    res = []
    res_t = []
    res += [wps[0]]
    res_t += [t[0]]
    pts = []
    for wp in wps :
        pts += [contactPlacementFromConfig(fullBody,wp,eeName).translation.T[0]]
    i = 1
    i_begin = i-1
    i_mid = [i]
    while i < len(wps)-1 :
        i_end = i+1        
        use = False
        #print "- while i = ",i
        for id in i_mid :
            d = distPointLine(pts[id],pts[i_begin],pts[i_end])
            #print "d = ",d
            if d > EPS:
                use = True
        if use :
            res += [wps[i]]
            res_t += [t[i]]  
            i_begin = i
            i_mid = []
        i+=1
        i_mid += [i]
        
    res += [wps[-1]]
    res_t += [t[-1]]
    
    if VERBOSE :
        print "# filter waypoints, before : "+str(len(wps))+" after : "+str(len(res))
    return res,res_t    
"""
# filter wp : remove "useless" waypoint. 
# A waypoint is classified useless if :
# - the directions of the two lines (previous -> wp  and wp -> next ) are close
# - the segment previous-> wp is really small
def filterWPs(fullBody,eeName,wps,t):
    ANGLE_EPS = 0.05 # after a cos(angle)
    SIZE_EPS = 0.001
    res = []
    res_t = []
    res += [wps[0]]
    res_t += [t[0]]   
    for i in range(1,len(wps)-1):
        q_prev = wps[i-1]
        p_prev = contactPlacementFromConfig(fullBody,q_prev,eeName).translation.T[0] 
        q = wps[i]
        p = contactPlacementFromConfig(fullBody,q,eeName).translation.T[0]    
        q_next = wps[i+1]
        p_next = contactPlacementFromConfig(fullBody,q_next,eeName).translation.T[0]  
        a = p - p_prev
        b = p_next - p 
        length = norm(a)
        ua = a/length
        ub = b/norm(b)
        angle = 1 - abs(ua.dot(ub))
        if length >= SIZE_EPS and angle >= ANGLE_EPS :
            res += [wps[i]]
            res_t += [t[i]]            
    res += [wps[-1]]
    res_t += [t[-1]]
    
    if VERBOSE :
        print "# filter waypoints, before : "+str(len(wps))+" after : "+str(len(res))
    return res,res_t
"""


def computeProblemConstraints(pData,fullBody,pathId,t,eeName,viewer):
    groupName = "constraints_"+str(pathId)
    if DISPLAY_CONSTRAINTS :
        viewer.client.gui.createGroup(groupName)
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
    wps,t_paths = filterWPs(fullBody,eeName,wps,t_paths)    
    # approximate the switching times (infos from limb-rrt)
    if len(t_paths)>2 :
        splits=[]
        t_ratio = t/t_paths[-1] # ratio between the imposed time of the bezier curve (t) and the "time" (a pseudo distance) of the solution of the rrt 
        for i in range(1,len(t_paths)-1):
            ti = t_paths[i]*t_ratio
            if ti > t:
                ti = t
            splits+= [ti]
            if len(splits) > 1:
                if splits[-1] == splits[-2] :
                    print "Error in bezier_constrained : two adjacent constrained have the same switch time !!"
        if VERBOSE:
            print "number of switch between constraints : ",len(splits)
            print "splits timings : ",splits
        pDef.splits = np.array([splits]).T       
    else : 
        if VERBOSE :
            print "Only one constraint set for the whole trajectory."
    # compute constraints around each line of the limb-rrt solution : 
    q_from = wps[0]
    p_from = contactPlacementFromConfig(fullBody,q_from,eeName)
    # debug test : 
    #p_from.translation = pData.c0_ # not in the right frame ! 
    for i in range(1,len(wps)):
        q_to = wps[i]
        p_to = contactPlacementFromConfig(fullBody,q_to,eeName)
        #if i == len(wps)-1 : # debug test
        #    p_from.translation = pData.c1_            
        A,b = computeInequalitiesAroundLine(fullBody,p_from,p_to,eeName,groupName,viewer)
        if VERBOSE :
            print "Inequalities computed."
        pDef.addInequality(A,b)        
        p_from = p_to.copy()
    if DISPLAY_CONSTRAINTS :
        viewer.client.gui.addToGroup(groupName,viewer.sceneName)   
        viewer.client.gui.refresh()
        
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
        if DISPLAY_JOINT_LEVEL :
            pp.displayPath(pathId,jointName=fullBody.getLinkNames(eeName)[0]) 
        else :
            #TODO
            pp.displayPath(pathId,jointName=fullBody.getLinkNames(eeName)[0],offset =cfg.Robot.dict_offset[eeName].translation.T.tolist()[0] )            
    
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
        print "Return predef trajectory (may be in collision)."
        return predefTraj # FIXME throw an error instead of this ? require changes in wb scripts
    
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