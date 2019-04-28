import pinocchio as pin
from pinocchio import SE3, Quaternion,Force
import tsid
import numpy as np
from numpy.linalg import norm as norm
import os
from rospkg import RosPack
import time
import commands
import gepetto.corbaserver
import mlp.config as cfg
import multicontact_api
from multicontact_api import WrenchCone,SOC6,ContactPatch, ContactPhaseHumanoid, ContactSequenceHumanoid
from mlp.utils import trajectories
from mlp.utils.computation_tools import shiftZMPtoFloorAltitude
import mlp.viewer.display_tools as display_tools
import math
from mlp.utils.wholebody_result import Result
from mlp.utils.util import * 
from mlp.end_effector import generateEndEffectorTraj,effectorCanRetry
    
def createContactForEffector(invdyn,robot,phase,eeName):
    size = cfg.IK_eff_size[eeName]
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
    contact = tsid.Contact6d("contact_"+eeName, robot, eeName, contact_Point, contactNormal, cfg.MU, cfg.fMin, cfg.fMax)
    contact.setKp(cfg.kp_contact * np.matrix(np.ones(6)).transpose())
    contact.setKd(2.0 * np.sqrt(cfg.kp_contact) * np.matrix(np.ones(6)).transpose())
    ref = JointPlacementForEffector(phase,eeName)
    contact.setReference(ref)
    invdyn.addRigidContact(contact,cfg.w_forceRef)
    if cfg.WB_VERBOSE :
        print "create contact for effector ",eeName
        print "contact placement : ",ref       
        print "contact_normal : ",contactNormal
        print "contact points : \n",contact_Point        
    return contact

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


def computeCOMRefFromPhase(phase,time_interval):
    #return trajectories.SmoothedCOMTrajectory("com_reference", phase, com_init, dt) # cubic interpolation from timeopt dt to tsid dt
    com_ref = trajectories.TwiceDifferentiableEuclidianTrajectory("com_reference")
    # rearrange discretized points from phase to numpy matrices :
    N = len(phase.time_trajectory)
    timeline = np.matrix(np.zeros(N))
    c = np.matrix(np.zeros([3,N]))
    dc = np.matrix(np.zeros([3,N]))
    ddc = np.matrix(np.zeros([3,N]))  
    for i in range(N):
        timeline[0,i] = phase.time_trajectory[i]
        c[:,i] = phase.state_trajectory[i][0:3]
        dc[:,i] = phase.state_trajectory[i][3:6]
        ddc[:,i] = phase.control_trajectory[i][0:3]
    com_ref.computeFromPoints(timeline,c,dc,ddc)
    return com_ref

def computeAMRefFromPhase(phase,time_interval):
    #return trajectories.SmoothedCOMTrajectory("com_reference", phase, com_init, dt) # cubic interpolation from timeopt dt to tsid dt
    am_ref = trajectories.DifferentiableEuclidianTrajectory("am_reference")
    # rearrange discretized points from phase to numpy matrices :
    N = len(phase.time_trajectory)
    timeline = np.matrix(np.zeros(N))
    L = np.matrix(np.zeros([3,N]))
    dL = np.matrix(np.zeros([3,N]))
    for i in range(N):
        timeline[0,i] = phase.time_trajectory[i]
        L[:,i] = phase.state_trajectory[i][6:9]
        dL[:,i] = phase.control_trajectory[i][3:6]
    am_ref.computeFromPoints(timeline,L,dL)
    return am_ref


def generateWholeBodyMotion(cs,fullBody=None,viewer=None):
    if not viewer :
        print "No viewer linked, cannot display end_effector trajectories."
    print "Start TSID ... " 

    rp = RosPack()
    urdf = rp.get_path(cfg.Robot.packageName)+'/urdf/'+cfg.Robot.urdfName+cfg.Robot.urdfSuffix+'.urdf'
    if cfg.WB_VERBOSE:
        print "load robot : " ,urdf    
    #srdf = "package://" + package + '/srdf/' +  cfg.Robot.urdfName+cfg.Robot.srdfSuffix + '.srdf'
    robot = tsid.RobotWrapper(urdf, pin.StdVec_StdString(), pin.JointModelFreeFlyer(), False)
    if cfg.WB_VERBOSE:
        print "robot loaded in tsid."
    # FIXME : tsid robotWrapper don't have all the required methods, only pinocchio have them
    pinRobot  = pin.RobotWrapper.BuildFromURDF(urdf, pin.StdVec_StdString(), pin.JointModelFreeFlyer(), False)

    q = cs.contact_phases[0].reference_configurations[0][:robot.nq].copy()
    v = np.matrix(np.zeros(robot.nv)).transpose()
    t = 0.0  # time
    # init states list with initial state (assume joint velocity is null for t=0)
    invdyn = tsid.InverseDynamicsFormulationAccForce("tsid", robot, False)
    invdyn.computeProblemData(t, q, v)
    data = invdyn.data()
    
    if cfg.EFF_CHECK_COLLISION : # initialise object needed to check the motion
        from mlp.utils import check_path
        validator = check_path.PathChecker(viewer,fullBody,cs,len(q),cfg.WB_VERBOSE)
        
    if cfg.WB_VERBOSE:
        print "initialize tasks : "   
    comTask = tsid.TaskComEquality("task-com", robot)
    comTask.setKp(cfg.kp_com * np.matrix(np.ones(3)).transpose())
    comTask.setKd(2.0 * np.sqrt(cfg.kp_com) * np.matrix(np.ones(3)).transpose())
    invdyn.addMotionTask(comTask, cfg.w_com, cfg.level_com, 0.0)     
    
    com_ref = robot.com(invdyn.data())
    trajCom = tsid.TrajectoryEuclidianConstant("traj_com", com_ref)  
    
    amTask = tsid.TaskAMEquality("task-am", robot)
    amTask.setKp(cfg.kp_am * np.matrix([1.,1.,0.]).T)    
    amTask.setKd(2.0 * np.sqrt(cfg.kp_am* np.matrix([1.,1.,0.]).T ))
    invdyn.addTask(amTask, cfg.w_am,cfg.level_am)
    trajAM = tsid.TrajectoryEuclidianConstant("traj_am", np.matrix(np.zeros(3)).T)     
    
    postureTask = tsid.TaskJointPosture("task-joint-posture", robot)
    postureTask.setKp(cfg.kp_posture * cfg.gain_vector)    
    postureTask.setKd(2.0 * np.sqrt(cfg.kp_posture* cfg.gain_vector) )
    postureTask.mask(cfg.masks_posture)         
    invdyn.addMotionTask(postureTask, cfg.w_posture,cfg.level_posture, 0.0)
    q_ref = q
    trajPosture = tsid.TrajectoryEuclidianConstant("traj_joint", q_ref[7:])    
    
    orientationRootTask = tsid.TaskSE3Equality("task-orientation-root", robot, 'root_joint')
    mask = np.matrix(np.ones(6)).transpose()
    mask[0:3] = 0
    mask[5] = cfg.YAW_ROT_GAIN 
    orientationRootTask.setKp(cfg.kp_rootOrientation * mask)
    orientationRootTask.setKd(2.0 * np.sqrt(cfg.kp_rootOrientation* mask) )
    invdyn.addMotionTask(orientationRootTask, cfg.w_rootOrientation,cfg.level_rootOrientation, 0.0)
    root_ref = robot.position(data, robot.model().getJointId( 'root_joint'))
    trajRoot = tsid.TrajectorySE3Constant("traj-root", root_ref)

    usedEffectors = []
    for eeName in cfg.Robot.dict_limb_joint.values() : 
        if isContactEverActive(cs,eeName):
            usedEffectors.append(eeName)
    # init effector task objects : 
    dic_effectors_tasks = createEffectorTasksDic(cs,robot)
    effectorTraj = tsid.TrajectorySE3Constant("traj-effector", SE3.Identity()) # trajectory doesn't matter as it's only used to get the correct struct and size
    
    # init empty dic to store effectors trajectories : 
    dic_effectors_trajs={}
    for eeName in usedEffectors:
        dic_effectors_trajs.update({eeName:None})

    # add initial contacts : 
    dic_contacts={}
    for eeName in usedEffectors:
        if isContactActive(cs.contact_phases[0],eeName):
            contact = createContactForEffector(invdyn,robot,cs.contact_phases[0],eeName)              
            dic_contacts.update({eeName:contact})
            
    if cfg.PLOT: # init a dict storing all the reference trajectories used (for plotting)
        stored_effectors_ref={}
        for eeName in dic_effectors_tasks:
            stored_effectors_ref.update({eeName:[]})       
    
    solver = tsid.SolverHQuadProg("qp solver")
    solver.resize(invdyn.nVar, invdyn.nEq, invdyn.nIn)
    
    # define nested function used in control loop
    def storeData(k_t,res,q,v,dv,invdyn,sol): 
        # store current state
        res.q_t[:,k_t] = q
        res.dq_t[:,k_t] = v                
        res.ddq_t[:,k_t] = dv                             
        res.tau_t[:,k_t] = invdyn.getActuatorForces(sol) # actuator forces, with external forces (contact forces)
        #store contact info (force and status)
        if cfg.IK_store_contact_forces :
            for eeName,contact in dic_contacts.iteritems():
                if invdyn.checkContact(contact.name, sol): 
                    res.contact_forces[eeName][:,k_t] = invdyn.getContactForce(contact.name, sol)
                    res.contact_normal_force[eeName][:,k_t] = contact.getNormalForce(res.contact_forces[eeName][:,k_t])
                    res.contact_activity[eeName][:,k_t] = 1
        # store centroidal info (real one and reference) :
        if cfg.IK_store_centroidal:
            pcom, vcom, acom = pinRobot.com(q,v,dv) 
            res.c_t[:,k_t] = pcom
            res.dc_t[:,k_t] = vcom
            res.ddc_t[:,k_t] = acom
            res.L_t[:,k_t] = pinRobot.centroidalMomentum(q,v).angular
            #res.dL_t[:,k_t] = pinRobot.centroidalMomentumVariation(q,v,dv) # FIXME : in robot wrapper, use * instead of .dot() for np matrices            
            pin.dccrba(pinRobot.model, pinRobot.data, q, v)
            res.dL_t[:,k_t] = Force(pinRobot.data.Ag.dot(dv)+pinRobot.data.dAg.dot(v)).angular
            # same for reference data : 
            res.c_reference[:,k_t] = com_desired
            res.dc_reference[:,k_t] = vcom_desired
            res.ddc_reference[:,k_t] = acom_desired
            res.L_reference[:,k_t] = L_desired
            res.dL_reference[:,k_t] = dL_desired 
            if cfg.IK_store_zmp : 
                tau = pin.rnea(pinRobot.model,pinRobot.data,q,v,dv) # tau without external forces, only used for the 6 first
                #res.tau_t[:6,k_t] = tau[:6]
                phi0 = pinRobot.data.oMi[1].act(Force(tau[:6]))
                res.wrench_t[:,k_t] = phi0.vector
                res.zmp_t[:,k_t] = shiftZMPtoFloorAltitude(cs,res.t_t[k_t],phi0)
                # same but from the 'reference' values : 
                Mcom = SE3.Identity()
                Mcom.translation = com_desired
                Fcom = Force.Zero()
                Fcom.linear = cfg.MASS*(acom_desired - cfg.GRAVITY)
                Fcom.angular = dL_desired
                F0 = Mcom.act(Fcom)
                res.wrench_reference[:,k_t] = F0.vector 
                res.zmp_reference[:,k_t] = shiftZMPtoFloorAltitude(cs,res.t_t[k_t],F0)
        if cfg.IK_store_effector: 
            for eeName in usedEffectors: # real position (not reference)
                res.effector_trajectories[eeName][:,k_t] = SE3toVec(robot.position(invdyn.data(), robot.model().getJointId(eeName)))        
        return res
        
    def printIntermediate(v,dv,invdyn,sol):
        print "Time %.3f" % (t)
        for eeName,contact in dic_contacts.iteritems():
            if invdyn.checkContact(contact.name, sol):
                f = invdyn.getContactForce(contact.name, sol)
                print "\tnormal force %s: %.1f" % (contact.name.ljust(20, '.'), contact.getNormalForce(f))
    
        print "\ttracking err %s: %.3f" % (comTask.name.ljust(20, '.'), norm(comTask.position_error, 2))
        for eeName,task in dic_effectors_tasks.iteritems():
            print "\ttracking err %s: %.3f" % (task.name.ljust(20, '.'), norm(task.position_error, 2))
        print "\t||v||: %.3f\t ||dv||: %.3f" % (norm(v, 2), norm(dv))      


    def checkDiverge(res,v,dv):
        if norm(dv) > 1e6 or norm(v) > 1e6 :
            print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
            print "/!\ ABORT : controler unstable at t = "+str(t)+"  /!\ "
            print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
            raise ValueError("ABORT : controler unstable at t = "+str(t))
        if math.isnan(norm(dv)) or math.isnan(norm(v)) :
            print "!!!!!!    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
            print "/!\ ABORT : nan   at t = "+str(t)+"  /!\ "
            print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"  
            raise ValueError("ABORT : controler unstable at t = "+str(t))            
    def stopHere():
        if cfg.WB_ABORT_WHEN_INVALID :
            return res.resize(phase_interval[0]),pinRobot
        elif cfg.WB_RETURN_INVALID : 
            return res.resize(k_t),pinRobot         
   
    # time check
    dt = cfg.IK_dt  
    if cfg.WB_VERBOSE:
        print "dt : ",dt
    res = Result(robot.nq,robot.nv,cfg.IK_dt,eeNames=usedEffectors,cs=cs)
    N = res.N    
    last_display = 0 
    if cfg.WB_VERBOSE:
        print "tsid initialized, start control loop"
        #raw_input("Enter to start the motion (motion displayed as it's computed, may be slower than real-time)")
    time_start = time.time()

    # For each phases, create the necessary task and references trajectories :
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
        t_phase_begin = res.phases_intervals[pid][0]*dt
        t_phase_end = res.phases_intervals[pid][-1]*dt
        time_interval = [t_phase_begin, t_phase_end]
        if cfg.WB_VERBOSE:
            print "time_interval ",time_interval
        # generate com ref traj from phase : 
        com_init = np.matrix(np.zeros((9, 1)))
        com_init[0:3, 0] = robot.com(invdyn.data())
        com_traj = computeCOMRefFromPhase(phase,time_interval)
        
        am_traj = computeAMRefFromPhase(phase,time_interval)
        # add root's orientation ref from reference config : 
        if phase_next :
            root_traj = trajectories.TrajectorySE3LinearInterp(SE3FromConfig(phase.reference_configurations[0]),SE3FromConfig(phase_next.reference_configurations[0]),time_interval)
        else : 
            root_traj = trajectories.TrajectorySE3LinearInterp(SE3FromConfig(phase.reference_configurations[0]),SE3FromConfig(phase.reference_configurations[0]),time_interval)
            
        # add newly created contacts : 
        for eeName in usedEffectors:
            if phase_prev and not isContactActive(phase_prev,eeName) and isContactActive(phase,eeName) :
                invdyn.removeTask(dic_effectors_tasks[eeName].name, 0.0) # remove pin task for this contact
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
                invdyn.addMotionTask(task, cfg.w_eff, cfg.level_eff, 0.0)
                #create reference trajectory for this task : 
                placement_init = robot.position(invdyn.data(), robot.model().getJointId(eeName))
                placement_end = JointPlacementForEffector(phase_next,eeName)                
                ref_traj = generateEndEffectorTraj(time_interval,placement_init,placement_end,0)
                if cfg.WB_VERBOSE :
                    print "t interval : ",time_interval
                    print "positions : ",placements                 
                dic_effectors_trajs.update({eeName:ref_traj})

        # start removing the contact that will be broken in the next phase :
        # (This tell the solver that it should start minimzing the contact force on this contact, and ideally get to 0 at the given time)
        for eeName,contact in dic_contacts.iteritems() :        
            if phase_next and isContactActive(phase,eeName) and not isContactActive(phase_next,eeName) : 
                transition_time = t_phase_end - t - dt/2.
                if cfg.WB_VERBOSE :
                    print "\nTime %.3f Start breaking contact %s. transition time : %.3f\n" % (t, contact.name,transition_time)
                invdyn.removeRigidContact(contact.name, transition_time)            
        
        
        if cfg.WB_STOP_AT_EACH_PHASE :
            raw_input('start simulation')
        # save values at the beginning of the current phase
        q_begin = q.copy()
        v_begin = v.copy()
        phaseValid = False
        swingPhase = False # will be true if an effector move during this phase
        iter_for_phase = -1
        # iterate until a valid motion for this phase is found (ie. collision free and which respect joint-limits)
        while not phaseValid :
            if iter_for_phase >=0 :
                # reset values to their value at the beginning of the current phase
                q = q_begin.copy()
                v = v_begin.copy()
            iter_for_phase += 1
            if cfg.WB_VERBOSE:
                print "Start simulation for phase "+str(pid)+", try number :  "+str(iter_for_phase)
            # loop to generate states (q,v,a) for the current contact phase :
            if pid == cs.size()-1 : # last state
                phase_interval = res.phases_intervals[pid]
            else :
                phase_interval = res.phases_intervals[pid][:-1]
            for k_t in phase_interval :
                t = res.t_t[k_t]
                # set traj reference for current time : 
                # com 
                sampleCom = trajCom.computeNext()
                com_desired = com_traj(t)[0]
                vcom_desired = com_traj(t)[1]
                acom_desired = com_traj(t)[2]
                sampleCom.pos(com_desired)
                sampleCom.vel(vcom_desired)
                sampleCom.acc(acom_desired)
                comTask.setReference(sampleCom)
                
                # am 
                sampleAM = trajAM.computeNext()
                L_desired = am_traj(t)[0]
                dL_desired = am_traj(t)[1] 
                sampleAM.pos(L_desired)
                sampleAM.vel(dL_desired)
                amTask.setReference(sampleAM)
                
                # posture
                samplePosture = trajPosture.computeNext()
                #print "postural task ref : ",samplePosture.pos()
                postureTask.setReference(samplePosture)
                
                # root orientation : 
                sampleRoot = trajRoot.computeNext()
                sampleRoot.pos(SE3toVec(root_traj(t)[0]))
                sampleRoot.vel(MotiontoVec(root_traj(t)[1]))
                orientationRootTask.setReference(sampleRoot)
                
                if cfg.WB_VERBOSE == 2:
                    print "### references given : ###"
                    print "com  pos : ",sampleCom.pos()
                    print "com  vel : ",sampleCom.vel()
                    print "com  acc : ",sampleCom.acc()
                    print "AM   pos : ",sampleAM.pos()
                    print "AM   vel : ",sampleAM.vel()
                    print "root pos : ",sampleRoot.pos()
                    print "root vel : ",sampleRoot.vel()
                
                # end effector (if they exists)
                for eeName,traj in dic_effectors_trajs.iteritems():
                    if traj:
                        swingPhase = True # there is an effector motion in this phase
                        sampleEff = effectorTraj.computeNext()
                        sampleEff.pos(SE3toVec(traj(t)[0]))
                        sampleEff.vel(MotiontoVec(traj(t)[1]))
                        dic_effectors_tasks[eeName].setReference(sampleEff)
                        if cfg.WB_VERBOSE == 2:
                            print "effector "+str(eeName)+" pos : "+str(sampleEff.pos())
                            print "effector "+str(eeName)+" vel : "+str(sampleEff.vel()) 
                        if cfg.IK_store_effector:
                            res.effector_references[eeName][:,k_t] = SE3toVec(traj(t)[0])
                    elif cfg.IK_store_effector:
                        if k_t == 0: 
                            res.effector_references[eeName][:,k_t] = SE3toVec(robot.position(invdyn.data(), robot.model().getJointId(eeName)))
                        else:
                            res.effector_references[eeName][:,k_t] = res.effector_references[eeName][:,k_t-1]
            
                # solve HQP for the current time
                HQPData = invdyn.computeProblemData(t, q, v)
                if cfg.WB_VERBOSE and t < phase.time_trajectory[0]+dt:
                    print "final data for phase ",pid
                    HQPData.print_all()
            
                sol = solver.solve(HQPData)
                dv = invdyn.getAccelerations(sol)
                res = storeData(k_t,res,q,v,dv,invdyn,sol)
                # update state
                v_mean = v + 0.5 * dt * dv
                v += dt * dv
                q = pin.integrate(robot.model(), q, dt * v_mean)    

                if cfg.WB_VERBOSE and int(t/dt) % cfg.IK_PRINT_N == 0:
                    printIntermediate(v,dv,invdyn,sol)
                checkDiverge(res,v,dv)
                
                
            # end while t \in phase_t (loop for the current contact phase) 
            if swingPhase and cfg.EFF_CHECK_COLLISION :
                phaseValid,t_invalid = validator.check_motion(res.q_t[:,phase_interval[0]:k_t])
                #if iter_for_phase < 3 :# FIXME : debug only, force limb-rrt
                #    phaseValid = False
                if not phaseValid :
                    print "Phase "+str(pid)+" not valid at t = "+ str(t_invalid)
                    if t_invalid <= cfg.EFF_T_PREDEF or t_invalid >= ((t_phase_end-t_phase_begin) - cfg.EFF_T_PREDEF):
                        print "Motion is invalid during predefined phases, cannot change this."
                        return stopHere()
                    if effectorCanRetry() : 
                        print "Try new end effector trajectory."  
                        try:
                            for eeName,oldTraj in dic_effectors_trajs.iteritems():
                                if oldTraj: # update the traj in the map
                                    placement_init = JointPlacementForEffector(phase_prev,eeName)
                                    placement_end = JointPlacementForEffector(phase_next,eeName)  
                                    ref_traj = generateEndEffectorTraj(time_interval,placement_init,placement_end,iter_for_phase+1,res.q_t[:,phase_interval[0]:k_t],phase_prev,phase,phase_next,fullBody,eeName,viewer)
                                    dic_effectors_trajs.update({eeName:ref_traj}) 
                        except ValueError,e :
                            print "ERROR in generateEndEffectorTraj :"
                            print e.message
                            return stopHere()                    
                    else : 
                        print "End effector method choosen do not allow retries, abort here."
                        return stopHere()
            else : # no effector motions, phase always valid (or bypass the check)
                phaseValid = True
                if cfg.WB_VERBOSE :
                    print "Phase "+str(pid)+" valid."
            if phaseValid:
                # display all the effector trajectories for this phase
                if viewer and cfg.DISPLAY_FEET_TRAJ :
                    for eeName,ref_traj in dic_effectors_trajs.iteritems():
                        if ref_traj :
                            display_tools.displaySE3Traj(ref_traj,viewer,eeName+"_traj_"+str(pid),cfg.Robot.dict_limb_color_traj[eeName] ,time_interval ,cfg.Robot.dict_offset[eeName])                               
                            viewer.client.gui.setVisibility(eeName+"_traj_"+str(pid),'ALWAYS_ON_TOP')                
                            if cfg.PLOT: # add current ref_traj to the list for plotting
                                stored_effectors_ref[eeName] +=[ref_traj]
        #end while not phaseValid 
    
    # run for last state (with the same references as the previous state)
    time_end = time.time() - time_start
    print "Whole body motion generated in : "+str(time_end)+" s."
    if cfg.WB_VERBOSE:
        print "\nFinal COM Position  ", robot.com(invdyn.data()).T
        print "Desired COM Position", cs.contact_phases[-1].final_state.T
        
    # store last state : #FIXME


    if cfg.PLOT:
        from mlp.utils import plot
        plot.plotEffectorRef(stored_effectors_ref,dt)   # plot inside this file, as we have access to the real Bezier object and not only the discretization stored in 'res' struct         
    
    assert (k_t == res.N-1) and "res struct not fully filled."
    return res,pinRobot

   
    