import pinocchio as se3
from pinocchio import SE3, Quaternion
import tsid
import numpy as np
from numpy.linalg import norm as norm
import os
from rospkg import RosPack
import time
import commands
import gepetto.corbaserver
import hpp_wholebody_motion.config as cfg
import locomote
from locomote import WrenchCone,SOC6,ContactPatch, ContactPhaseHumanoid, ContactSequenceHumanoid
from hpp_wholebody_motion.utils import trajectories
import hpp_wholebody_motion.end_effector.bezier_predef as EETraj
import hpp_wholebody_motion.viewer.display_tools as display_tools
import math

def SE3toVec(M):
    v = np.matrix(np.zeros((12, 1)))
    for j in range(3):
        v[j] = M.translation[j]
        v[j + 3] = M.rotation[j, 0]
        v[j + 6] = M.rotation[j, 1]
        v[j + 9] = M.rotation[j, 2]
    return v

def MotiontoVec(M):
    v = np.matrix(np.zeros((6, 1)))
    for j in range(3):
        v[j] = M.linear[j]
        v[j + 3] = M.angular[j]
    return v

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

def createContactForEffector(invdyn,robot,phase,eeName):
    size = cfg.Robot.dict_size[eeName]
    transform = cfg.Robot.dict_offset[eeName]          
    lxp = size[0]/2. + transform.translation[0,0]  # foot length in positive x direction
    lxn = size[0]/2. - transform.translation[0,0]  # foot length in negative x direction
    lyp = size[1]/2. + transform.translation[1,0]  # foot length in positive y direction
    lyn = size[1]/2. - transform.translation[1,0]  # foot length in negative y direction
    lz =  transform.translation[2,0]   # foot sole height with respect to ankle joint                                                    
    contactNormal = np.matrix([0., 0., 1.]).T  # direction of the normal to the contact surface
    contactNormal = transform.rotation * contactNormal
    contact_Point = np.matrix(np.ones((3, 4)))
    contact_Point[0, :] = [-lxn, -lxn, lxp, lxp]
    contact_Point[1, :] = [-lyn, lyp, -lyn, lyp]
    contact_Point[2, :] = [lz]*4
    # build ContactConstraint object
    contact = tsid.Contact6d("contact_"+eeName, robot, eeName, contact_Point, contactNormal, cfg.MU, cfg.fMin, cfg.fMax,cfg.w_forceRef)
    contact.setKp(cfg.kp_contact * np.matrix(np.ones(6)).transpose())
    contact.setKd(2.0 * np.sqrt(cfg.kp_contact) * np.matrix(np.ones(6)).transpose())
    ref = JointPatchForEffector(phase,eeName).placement
    contact.setReference(ref)
    invdyn.addRigidContact(contact)
    if cfg.WB_VERBOSE :
        print "create contact for effector ",eeName
        print "contact placement : ",ref            
    return contact
"""
# build a dic with keys = effector names used in the cs, value = contact constraints objects
def createContactsConstraintsDic(cs,robot):
    res = {}
    for eeName in cfg.Robot.dict_limb_joint.values():
        if isContactEverActive(cs,eeName):
            # build contact points matrice from offset : 
            # TODO : put offset in dic, need to refactor a lot of code ...
            if eeName == cfg.Robot.rfoot:
                size = [cfg.Robot.rLegx, cfg.Robot.rLegy]
                transform = cfg.Robot.MRsole_offset.copy()
            elif eeName == cfg.Robot.lfoot:
                size = [cfg.Robot.lLegx, cfg.Robot.lLegy]
                transform = cfg.Robot.MLsole_offset.copy()
            elif eeName == cfg.Robot.rhand:
                size = [cfg.Robot.rArmx, cfg.Robot.rArmy]
                transform = cfg.Robot.MRhand_offset.copy()
            elif eeName == cfg.Robot.rfoot:
                size = [cfg.Robot.lArmx, cfg.Robot.lArmy]
                transform = cfg.Robot.MLhand_offset.copy()
            else :
                raise Exception("Unknown effector name")             
            lxp = size[0] + transform.translation[0,0]  # foot length in positive x direction
            lxn = size[0] - transform.translation[0,0]  # foot length in negative x direction
            lyp = size[1] + transform.translation[1,0]  # foot length in positive y direction
            lyn = size[1] - transform.translation[1,0]  # foot length in negative y direction
            lz =  transform.translation[2,0]   # foot sole height with respect to ankle joint                                                    
            contactNormal = np.matrix([0., 0., 1.]).T  # direction of the normal to the contact surface
            contactNormal = transform.rotation * contactNormal
            contact_Point = np.matrix(np.ones((3, 4)))
            contact_Point[0, :] = [-lxn, -lxn, lxp, lxp]
            contact_Point[1, :] = [-lyn, lyp, -lyn, lyp]
            contact_Point[2, :] = [lz]*4
            # build ContactConstraint object
            contact = tsid.Contact6d("contact_"+eeName, robot, eeName, contact_Point, contactNormal, cfg.MU, cfg.fMin, cfg.fMax,cfg.w_forceRef)
            contact.setKp(cfg.kp_contact * np.matrix(np.ones(6)).transpose())
            contact.setKd(2.0 * np.sqrt(cfg.kp_contact) * np.matrix(np.ones(6)).transpose())
            if cfg.WB_VERBOSE :
                print "create contact constraint for effector ",eeName
                print "contact points : ",contact_Point            
                print "contact normal : ",contactNormal   
            res.update({eeName:contact})
    return res
"""

# build a dic with keys = effector names used in the cs, value = Effector tasks objects
def createEffectorTasksDic(cs,robot):
    res = {}
    for eeName in cfg.Robot.dict_limb_joint.values():
        if isContactEverActive(cs,eeName):
            # build effector task object
            effectorTask = tsid.TaskSE3Equality("task-"+eeName, robot, eeName)
            effectorTask.setKp(cfg.kp_Eff * np.matrix(np.ones(6)).transpose())
            effectorTask.setKd(2.0 * np.sqrt(cfg.kp_Eff) * np.matrix(np.ones(6)).transpose()) 
            res.update({eeName:effectorTask})
    return res

def generateEEReferenceTraj(robot,robotData,t,phase,phase_next,eeName,viewer = None):
    time_interval = [t, phase.time_trajectory[-1]]    
    placements = []
    placement_init = robot.position(robotData, robot.model().getJointId(eeName))
    placement_end = JointPatchForEffector(phase_next,eeName).placement
    placements.append(placement_init)
    placements.append(placement_end)    
    if cfg.USE_BEZIER_EE :         
        ref_traj = EETraj.generateBezierTraj(placement_init,placement_end,time_interval)
    else : 
        ref_traj = trajectories.SmoothedFootTrajectory(time_interval, placements) 
    if cfg.WB_VERBOSE :
        print "t interval : ",time_interval
        print "positions : ",placements        
    if viewer and cfg.DISPLAY_FEET_TRAJ :
        display_tools.displaySE3Traj(ref_traj,viewer,eeName+"_traj",cfg.Robot.dict_limb_color_traj[eeName] ,time_interval ,cfg.Robot.dict_offset[eeName])
    return ref_traj


def generateWholeBodyMotion(cs,viewer=None):
    if not viewer :
        print "No viewer linked, cannot display end_effector trajectories."
    print "Start TSID ... " 

    rp = RosPack()
    urdf = rp.get_path(cfg.Robot.packageName)+'/urdf/'+cfg.Robot.urdfName+cfg.Robot.urdfSuffix+'.urdf'
    if cfg.WB_VERBOSE:
        print "load robot : " ,urdf    
    #srdf = "package://" + package + '/srdf/' +  cfg.Robot.urdfName+cfg.Robot.srdfSuffix + '.srdf'
    robot = tsid.RobotWrapper(urdf, se3.StdVec_StdString(), se3.JointModelFreeFlyer(), False)
    if cfg.WB_VERBOSE:
        print "robot loaded in tsid"
    
    """
    ### for gepetto viewer .. but Fix me!!
    robot_display = se3.RobotWrapper(urdf, se3.StdVec_StdString(), se3.JointModelFreeFlyer())
    l = commands.getstatusoutput("ps aux |grep 'gepetto-gui'|grep -v 'grep'|wc -l")
    if int(l[1]) == 0:
        os.system('gepetto-gui &')
    time.sleep(1)
    cl = gepetto.corbaserver.Client()
    gui = cl.gui
    robot_display.initDisplay(loadModel=True)
    ###
    """
    
    q = cs.contact_phases[0].reference_configurations[0].copy()
    v = np.matrix(np.zeros(robot.nv)).transpose()
    """
    ###
    robot_display.displayCollisions(False)
    robot_display.displayVisuals(True)
    robot_display.display(q)
    viewer_tsid = robot_display.viewer
    gui = viewer_tsid.gui
    gui.refresh()
    ###
    """
    t = 0.0  # time
    q_t = []
    invdyn = tsid.InverseDynamicsFormulationAccForce("tsid", robot, False)
    invdyn.computeProblemData(t, q, v)
    data = invdyn.data()
    
    if cfg.WB_VERBOSE:
        print "initialize tasks : "   
    comTask = tsid.TaskComEquality("task-com", robot)
    comTask.setKp(cfg.kp_com * np.matrix(np.ones(3)).transpose())
    comTask.setKd(2.0 * np.sqrt(cfg.kp_com) * np.matrix(np.ones(3)).transpose())
    invdyn.addMotionTask(comTask, cfg.w_com, 1, 0.0)     
    
    com_ref = robot.com(invdyn.data())
    trajCom = tsid.TrajectoryEuclidianConstant("traj_com", com_ref)    
    
    postureTask = tsid.TaskJointPosture("task-joint-posture", robot)
    postureTask.setKp(cfg.kp_posture * cfg.gain_vector)    
    postureTask.setKd(2.0 * np.sqrt(cfg.kp_posture* cfg.gain_vector) )
    postureTask.mask(cfg.masks_posture)         
    invdyn.addMotionTask(postureTask, cfg.w_posture,1, 0.0)
    q_ref = q
    trajPosture = tsid.TrajectoryEuclidianConstant("traj_joint", q_ref[7:])    
    
    orientationRootTask = tsid.TaskSE3Equality("task-orientation-root", robot, 'root_joint')
    mask = np.matrix(np.ones(6)).transpose()
    mask[0:3] = 0
    mask[5] = cfg.YAW_ROT_GAIN 
    orientationRootTask.setKp(cfg.kp_rootOrientation * mask)
    orientationRootTask.setKd(2.0 * np.sqrt(cfg.kp_rootOrientation* mask) )
    invdyn.addMotionTask(orientationRootTask, cfg.w_rootOrientation,1, 0.0)
    root_ref = robot.position(data, robot.model().getJointId( 'root_joint'))
    trajRoot = tsid.TrajectorySE3Constant("traj-root", root_ref)

    usedEffectors = []
    for eeName in cfg.Robot.dict_limb_joint.values() : 
        if isContactEverActive(cs,eeName):
            usedEffectors.append(eeName)
    # init effector task objects : 
    dic_effectors_tasks = createEffectorTasksDic(cs,robot)
    effectorTraj = tsid.TrajectorySE3Constant("traj-effector", SE3.Identity()) # trajectory doesn't matter as it's only used to get the correct struct and size
    
    # initempty dic to store effectors trajectories : 
    dic_effectors_trajs={}
    for eeName in dic_effectors_tasks:
        dic_effectors_trajs.update({eeName:None})
        
    # add initial contacts : 
    dic_contacts={}
    for eeName in usedEffectors:
        if isContactActive(cs.contact_phases[0],eeName):
            contact = createContactForEffector(invdyn,robot,cs.contact_phases[0],eeName)              
            dic_contacts.update({eeName:contact})
    
    solver = tsid.SolverHQuadProg("qp solver")
    solver.resize(invdyn.nVar, invdyn.nEq, invdyn.nIn)
     
    # time check
    dt = cfg.IK_dt  
    if cfg.WB_VERBOSE:
        print "dt : ",dt
    com_desired =[]
    last_display = 0 
    if cfg.WB_VERBOSE:
        print "tsid initialized, start control loop"
        #raw_input("Enter to start the motion (motion displayed as it's computed, may be slower than real-time)")
    time_start = time.time()
    t = 0.0
    for pid in range(cs.size()):
        if cfg.WB_VERBOSE :
            print "## for phase : ",pid
            print "t = ",t
        phase = cs.contact_phases[pid]
        if pid < cs.size()-1:
            phase_next = cs.contact_phases[pid+1]
        else : 
            phase_next = None
        if pid >0:
            phase_prev = cs.contact_phases[pid-1]
        else : 
            phase_prev = None            
        time_interval = [t, phase.time_trajectory[-1]]
        # generate com ref traj from phase : 
        com_init = np.matrix(np.zeros((9, 1)))
        com_init[0:3, 0] = robot.com(invdyn.data())
        com_traj = trajectories.SmoothedCOMTrajectory("com_smoothing", phase, com_init, dt) # cubic interpolation from timeopt dt to tsid dt
        # add root's orientation ref from reference config : 
        if phase_next :
            root_traj = trajectories.TrajectorySE3LinearInterp(SE3FromConfig(phase.reference_configurations[0]),SE3FromConfig(phase_next.reference_configurations[0]),time_interval)
        else : 
            root_traj = trajectories.TrajectorySE3LinearInterp(SE3FromConfig(phase.reference_configurations[0]),SE3FromConfig(phase.reference_configurations[0]),time_interval)
            
        # add newly created contacts : 
        for eeName in usedEffectors:
            if phase_prev and not isContactActive(phase_prev,eeName) and isContactActive(phase,eeName) :
                invdyn.removeTask(dic_effectors_tasks[eeName].name, 0.0) # remove se3 task for this contact
                dic_effectors_trajs.update({eeName:None}) # delete reference trajectory for this task
                if cfg.WB_VERBOSE :
                    print "remove se3 task : "+dic_effectors_tasks[eeName].name                
                contact = createContactForEffector(invdyn,robot,phase,eeName)    
                dic_contacts.update({eeName:contact})
 
        # add se3 tasks for end effector not in contact that will be in contact next phase: 
        for eeName,task in dic_effectors_tasks.iteritems() :        
            if phase_next and not isContactActive(phase,eeName)  and isContactActive(phase_next,eeName): 
                if cfg.WB_VERBOSE :
                    print "add se3 task for "+eeName
                invdyn.addMotionTask(task, cfg.w_eff, 1, 0.0)
                #create reference trajectory for this task : 
                ref_traj = generateEEReferenceTraj(robot,invdyn.data(),t,phase,phase_next,eeName,viewer)  
                dic_effectors_trajs.update({eeName:ref_traj})

            
                  
        # start removing the contact that will be broken in the next phase : 
        for eeName,contact in dic_contacts.iteritems() :        
            if phase_next and isContactActive(phase,eeName) and not isContactActive(phase_next,eeName) : 
                transition_time = phase.time_trajectory[-1] - t 
                if cfg.WB_VERBOSE :
                    print "\nTime %.3f Start breaking contact %s. transition time : %.3f\n" % (t, contact.name,transition_time)
                invdyn.removeRigidContact(contact.name, transition_time)            
        
        
        if cfg.WB_STOP_AT_EACH_PHASE :
            raw_input('start simulation')
        # loop with increasing time for this phase : 
        while t < phase.time_trajectory[-1] - dt :
            # set traj reference for current time : 
            # com 
            sampleCom = trajCom.computeNext()
            sampleCom.pos(com_traj(t)[0].T)
            sampleCom.vel(com_traj(t)[1].T)
            com_desired = com_traj(t)[0].T
            #print "com desired : ",com_desired.T
            comTask.setReference(sampleCom)
            # posture
            samplePosture = trajPosture.computeNext()
            #print "postural task ref : ",samplePosture.pos()
            postureTask.setReference(samplePosture)
            # root orientation : 
            sampleRoot = trajRoot.computeNext()
            sampleRoot.pos(SE3toVec(root_traj(t)[0]))
            sampleRoot.vel(MotiontoVec(root_traj(t)[1]))
            orientationRootTask.setReference(sampleRoot)
            
            # end effector (if they exists)
            for eeName,traj in dic_effectors_trajs.iteritems():
                if traj:
                    sampleEff = effectorTraj.computeNext()
                    sampleEff.pos(SE3toVec(traj(t)[0]))
                    sampleEff.vel(MotiontoVec(traj(t)[1]))
                    dic_effectors_tasks[eeName].setReference(sampleEff)
        
            HQPData = invdyn.computeProblemData(t, q, v)
            if cfg.WB_VERBOSE and t < phase.time_trajectory[0]+dt:
                print "final data for phase ",pid
                HQPData.print_all()
        
            sol = solver.solve(HQPData)
            tau = invdyn.getActuatorForces(sol)
            dv = invdyn.getAccelerations(sol)
        
            if cfg.WB_VERBOSE and int(t/dt) % cfg.IK_PRINT_N == 0:
                print "Time %.3f" % (t)
                for eeName,contact in dic_contacts.iteritems():
                    if invdyn.checkContact(contact.name, sol):
                        f = invdyn.getContactForce(contact.name, sol)
                        print "\tnormal force %s: %.1f" % (contact.name.ljust(20, '.'), contact.getNormalForce(f))
            
                print "\ttracking err %s: %.3f" % (comTask.name.ljust(20, '.'), norm(comTask.position_error, 2))
                for eeName,task in dic_effectors_tasks.iteritems():
                    print "\ttracking err %s: %.3f" % (task.name.ljust(20, '.'), norm(task.position_error, 2))
                print "\t||v||: %.3f\t ||dv||: %.3f" % (norm(v, 2), norm(dv))
        
            v_mean = v + 0.5 * dt * dv
            v += dt * dv
            q = se3.integrate(robot.model(), q, dt * v_mean)
            q_t += [q]
            t += dt
            
            """
            if int(t/dt) % cfg.IK_DISPLAY_N == 0:
                if last_display > 0 : 
                    elapsed_time = time.time() - last_display
                    delta = (dt*cfg.IK_DISPLAY_N) - elapsed_time
                    if delta > 0 :
                        time.sleep(delta)
                    #else : 
                        #print "/!\ computation not real time : "+str((elapsed_time/(dt*cfg.IK_DISPLAY_N)) )+" times slower than the real time. "
                robot_display.display(q)
                last_display = time.time()
            """   
            
        
            if norm(dv) > 1e6 or norm(v) > 1e6 :
                print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                print "/!\ ABORT : controler unstable at t = "+str(t)+"  /!\ "
                print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"                
                return q_t
            if math.isnan(norm(dv)) or math.isnan(norm(v)) :
                print "!!!!!!    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                print "/!\ ABORT : nan   at t = "+str(t)+"  /!\ "
                print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"                
                return q_t                
            
    time_end = time.time() - time_start
    print "Whole body motion generated in : "+str(time_end)+" s."
    if cfg.WB_VERBOSE:
        print "\nFinal COM Position  ", robot.com(invdyn.data()).T
        print "Desired COM Position", cs.contact_phases[-1].final_state.T
    
    return q_t

   
    