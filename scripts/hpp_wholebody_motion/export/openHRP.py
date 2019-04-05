# Authors : Justin Carpentier, Pierre Fernbach
## ONLY WORK FOR HRP-2 !! 

import os
import hpp_wholebody_motion.config as cfg
import pinocchio as se3 
from pinocchio import SE3, rnea
from pinocchio.utils import *
import numpy as np
from rospkg import RosPack
#import tsid # required for robot wrapper ! FIXME
from pinocchio.robot_wrapper import RobotWrapper
from hpp_wholebody_motion.utils.computation_tools import *

class Struct():
    None
    
DT = 0.005
    
# hardcoded ID for HRP-2 : 
ff = range(7)
chest = range(7,9)
head = range(9,11)
l_arm = range(11,18)
r_arm = range(18,25)
l_leg = range(25,31)
r_leg = range(31,37)


def computeResultTrajectory(robot,cs, q_t_list, v_t_list, a_t_list):
    # start by adapting the time step between the IK and the the one of open hrp (0.005)
    assert DT>= cfg.IK_dt and "The dt used when computing q(t) must be superior or equal to 0.005 for this export."
    assert (DT % cfg.IK_dt)==0 and "The dt used when computing q(t) must be a multiple of 0.005."
    n = int(DT/cfg.IK_dt)
    N_ik = len(q_t_list)
    N = N_ik/n
    #print "Export openHRP, ratio between dt :  ",n
    #print "Num point for IK : ",N_ik
    #print "Num point for openHRP : ",N
    q_t = np.matrix(np.zeros([q_t_list[0].shape[0],N]))
    v_t = np.matrix(np.zeros([v_t_list[0].shape[0],N]))
    a_t = np.matrix(np.zeros([a_t_list[0].shape[0],N]))
    for i in range(N):
        q_t[:,i] = q_t_list[i*n] 
        v_t[:,i] = v_t_list[i*n]    
        a_t[:,i] = a_t_list[i*n]       
    # build timeline vector : 
    t_t = [i*DT for i in range(q_t.shape[1])]
    #print "timeline size : ",len(t_t)
    model = robot.model
    data = robot.data

    ZMP_t = np.matrix(np.zeros([3,N]))
    waist_t = np.matrix(np.zeros([3,N]))
    pcom_t = np.matrix(np.empty([3,N]))
    vcom_t = np.matrix(np.empty([3,N]))
    acom_t = np.matrix(np.empty([3,N]))
    tau_t = np.matrix(np.empty([robot.nv,N]))

    wrench_t = np.matrix(np.empty([6,N]))

    waist_orientation_t = np.matrix(np.empty([3,N]))

    # Sample end effector traj
    ee_t = dict()
    for limb_name in cfg.Robot.limbs_names:
        ee_t[cfg.Robot.dict_limb_joint[limb_name]] = []
        
    ee_names = list(ee_t.viewkeys())

    for k in range(N):
        q = q_t[:,k]
        v = v_t[:,k]
        a = a_t[:,k]

        #M = robot.mass(q)
        #b = robot.biais(q,v)

        #robot.dynamics(q,v,0*v)
        se3.rnea(model,data,q,v,a)
        #se3.forwardKinematics(model,data,q)
        #robot.computeJacobians(q)

        pcom, vcom, acom = robot.com(q,v,a)

        # Update EE placements
        for ee in ee_names:
            #ee_t[ee].append(Mee[ee](q,update_kinematics=False).copy())
            ee_t[ee].append(robot.placement(q, model.getJointId(ee)).copy())

        # Update CoM data
        pcom_t[:,k] = pcom
        vcom_t[:,k] = vcom
        acom_t[:,k] = acom

        #oXi_s = robot.data.oMi[1].inverse().np.T
        #phi0 = oXi_s * (M[:6,:] * a + b[:6])
        tau_t[:,k] = data.tau
        phi0 = data.oMi[1].act(se3.Force(data.tau[:6]))
        ZMP_t[:,k] = shiftZMPtoFloorAltitude(cs,t_t[k],phi0)

        waist_t[:,k] = robot.data.oMi[1].translation
        waist_orientation_t[:,k] = matrixToRpy(robot.data.oMi[1].rotation)

    result = Struct()

    result.t_t = t_t
    result.ZMP_t = ZMP_t
    result.waist_t = waist_t
    result.waist_orientation_t = waist_orientation_t

    result.pcom_t = pcom_t
    result.vcom_t = vcom_t
    result.acom_t = acom_t

    result.wrench_t = wrench_t
    result.tau_t = tau_t

    result.q_t = q_t
    result.v_t = v_t
    result.a_t = a_t

    result.ee_t = ee_t

    if cfg.PLOT:
        from hpp_wholebody_motion.utils import plot
        plot.plotZMP(cs,result.ZMP_t,result.pcom_t)
        
    return result

def writeOpenHRPConfig(robot,q,t=0.):
    def write_vector(vector, delim):
        line = ""
        for k in range(vector.shape[0]):
            line += delim + str(vector[k,0])

        return line

    line = str(t)
    delim = " "
    q_openhrp = []

    # RL
    line += write_vector(q[r_leg], delim)
    q_openhrp.append(q[r_leg].tolist())

    # LL
    line += write_vector(q[l_leg], delim)
    q_openhrp.append(q[l_leg].tolist())

    # Chest
    line += write_vector(q[chest], delim)
    q_openhrp.append(q[chest].tolist())

    # Head
    line += write_vector(q[head], delim)
    q_openhrp.append(q[head].tolist())

    # RA
    line += write_vector(q[r_arm], delim)
    q_openhrp.append(q[r_arm].tolist())

    # LA
    line += write_vector(q[l_arm], delim)
    q_openhrp.append(q[l_arm].tolist())

    # Fingers
    line += write_vector(np.matrix(np.zeros([10, 1])), delim)
    q_openhrp.append(np.matrix(np.zeros([10, 1])).tolist())

    q_openhrp = np.matrix(np.concatenate(q_openhrp))

    return q_openhrp,line

def generateOpenHRPMotion(robot, data, path, project_name):
    def write_vector(vector, delim):
        line = ""
        for k in range(vector.shape[0]):
            line += delim + str(vector[k,0])

        return line
    
    timeline = data.t_t
    N = len(timeline)
    print "in generateOpenHRPMotion, N= ",N
    dt = DT
 
    delim = " "
    eol = "\n"

    filename_prefix = path + '/' + project_name

    ## ZMP trajectory ##
    filename_zmp = filename_prefix + '.zmp'
    file_zmp = open(filename_zmp, "w")

    ZMP_waist = data.ZMP_t - data.waist_t

    for k in range(N):
        line = str(timeline[k])
        for i in range(3):
            line += delim + str(ZMP_waist[i,k])

        line += eol
        file_zmp.write(line)

    file_zmp.close()
    print "write file : ",filename_zmp
    ## Posture trajectory ##
    filename_pos = filename_prefix + '.pos'
    file_pos = open(filename_pos, "w")

    qout_l = []
    for k in range(N):
        line = str(timeline[k])
        q_openhrp = []

        # RL
        line += write_vector(data.q_t[r_leg,k], delim)
        q_openhrp.append(data.q_t[r_leg,k].tolist())

        # LL
        line += write_vector(data.q_t[l_leg,k], delim)
        q_openhrp.append(data.q_t[l_leg,k].tolist())

        # Chest
        line += write_vector(data.q_t[chest,k], delim)
        q_openhrp.append(data.q_t[chest,k].tolist())

        # Head
        line += write_vector(data.q_t[head,k], delim)
        q_openhrp.append(data.q_t[head,k].tolist())

        # RA
        line += write_vector(data.q_t[r_arm,k], delim)
        q_openhrp.append(data.q_t[r_arm,k].tolist())

        # LA
        line += write_vector(data.q_t[l_arm,k], delim)
        q_openhrp.append(data.q_t[l_arm,k].tolist())

        # Fingers
        line += write_vector(np.matrix(np.zeros([10,1])), delim)
        q_openhrp.append(np.matrix(np.zeros([10,1])).tolist())

        line += eol
        file_pos.write(line)

        qout_l.append(np.matrix(np.concatenate(q_openhrp)))

    file_pos.close()
    print "write file : ",filename_pos

    ## Waist orientation
    filename_hip = filename_prefix + '.hip'
    file_hip = open(filename_hip, "w")

    for k in range(N):
        line = str(timeline[k])

        line += write_vector(data.waist_orientation_t[:,k], delim)

        line += eol
        file_hip.write(line)
    file_hip.close()
    print "write file : ",filename_hip

    return qout_l


def writeKinematicsData(robot, data, path, project_name):
    def write_vector(vector, delim):
        line = ""
        for k in range(vector.shape[0]):
            line += delim + str(vector[k,0])

        return line

    timeline = data.t_t
    N = len(timeline)
    print "in write kinematic, number of points : ",N
    dt = DT
    delim = " "
    eol = "\n"

    filename_prefix = path + '/' + project_name

    ## Config trajectory ##
    filename_config = filename_prefix + '_config.csv'
    file_config = open(filename_config, "w")

    for k in range(N):
        line = str(timeline[k])
        line += write_vector(data.q_t[:,k], delim)
        line += eol
        file_config.write(line)

    file_config.close()
    print "write file : ",filename_config

    ## Vel trajectory ##
    filename_vel = filename_prefix + '_vel.csv'
    file_vel = open(filename_vel, "w")

    for k in range(N):
        line = str(timeline[k])
        line += write_vector(data.v_t[:,k], delim)
        line += eol
        file_vel.write(line)

    file_vel.close()
    print "write file : ",filename_vel

    ## Acc trajectory ##
    filename_acc = filename_prefix + '_acc.csv'
    file_acc = open(filename_acc, "w")

    for k in range(N):
        line = str(timeline[k])
        line += write_vector(data.a_t[:,k], delim)
        line += eol
        file_acc.write(line)

    file_acc.close()
    print "write file : ",filename_acc


def export(cs,q_t,v_t,a_t):
    assert len(q_t) == len(v_t) and len(q_t) == len(a_t) and "all states vector must have the same size"
    rp = RosPack()
    urdf = rp.get_path(cfg.Robot.packageName)+'/urdf/'+cfg.Robot.urdfName+cfg.Robot.urdfSuffix+'.urdf'
    if cfg.WB_VERBOSE:
        print "load robot : " ,urdf    
    #srdf = "package://" + package + '/srdf/' +  cfg.Robot.urdfName+cfg.Robot.srdfSuffix + '.srdf'
    robot = RobotWrapper(urdf, se3.StdVec_StdString(), se3.JointModelFreeFlyer(), False)
    if cfg.WB_VERBOSE:
        print "robot loaded in export OpenHRP"    
  
    results = computeResultTrajectory(robot,cs, q_t, v_t, a_t)
    path = cfg.EXPORT_PATH+"/openHRP"
    q_openhrp_l = generateOpenHRPMotion(robot, results, path, cfg.DEMO_NAME)
    writeKinematicsData(robot, results, path, cfg.DEMO_NAME)
    