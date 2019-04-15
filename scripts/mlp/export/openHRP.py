# Authors : Justin Carpentier, Pierre Fernbach
## ONLY WORK FOR HRP-2 !! 

import os
import mlp.config as cfg
import pinocchio as pin 
from pinocchio import SE3, rnea
from pinocchio.utils import *
import numpy as np
from rospkg import RosPack
from pinocchio.robot_wrapper import RobotWrapper
from mlp.utils.computation_tools import *

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


def computeWaistData(robot,res):
    N = len(res.t_t)
   
    model = robot.model
    data = robot.data

    res.waist_t = np.matrix(np.zeros([3,N]))
    res.waist_orientation_t = np.matrix(np.empty([3,N]))

    for k in range(N):
        q = res.q_t[:,k]
        v = res.dq_t[:,k]
        a = res.ddq_t[:,k]

        pin.rnea(model,data,q,v,a)
        pcom, vcom, acom = robot.com(q,v,a)
        
        res.waist_t[:,k] = robot.data.oMi[1].translation
        res.waist_orientation_t[:,k] = matrixToRpy(robot.data.oMi[1].rotation)

    return res

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

def generateOpenHRPMotion(res, path, project_name,useRefZMP = False):
    def write_vector(vector, delim):
        line = ""
        for k in range(vector.shape[0]):
            line += delim + str(vector[k,0])

        return line
    if useRefZMP:
        ZMP_t = res.zmp_reference
    else:
        ZMP_t = res.zmp_t
        
    timeline = res.t_t
    N = len(timeline)
    print "in generateOpenHRPMotion, N= ",N
    dt = DT
 
    delim = " "
    eol = "\n"

    filename_prefix = path + '/' + project_name

    ## ZMP trajectory ##
    filename_zmp = filename_prefix + '.zmp'
    file_zmp = open(filename_zmp, "w")

    ZMP_waist = ZMP_t - res.waist_t

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
        line += write_vector(res.q_t[r_leg,k], delim)
        q_openhrp.append(res.q_t[r_leg,k].tolist())

        # LL
        line += write_vector(res.q_t[l_leg,k], delim)
        q_openhrp.append(res.q_t[l_leg,k].tolist())

        # Chest
        line += write_vector(res.q_t[chest,k], delim)
        q_openhrp.append(res.q_t[chest,k].tolist())

        # Head
        line += write_vector(res.q_t[head,k], delim)
        q_openhrp.append(res.q_t[head,k].tolist())

        # RA
        line += write_vector(res.q_t[r_arm,k], delim)
        q_openhrp.append(res.q_t[r_arm,k].tolist())

        # LA
        line += write_vector(res.q_t[l_arm,k], delim)
        q_openhrp.append(res.q_t[l_arm,k].tolist())

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

        line += write_vector(res.waist_orientation_t[:,k], delim)

        line += eol
        file_hip.write(line)
    file_hip.close()
    print "write file : ",filename_hip

    return qout_l


def writeKinematicsData(res, path, project_name):
    def write_vector(vector, delim):
        line = ""
        for k in range(vector.shape[0]):
            line += delim + str(vector[k,0])

        return line

    timeline = res.t_t
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
        line += write_vector(res.q_t[:,k], delim)
        line += eol
        file_config.write(line)

    file_config.close()
    print "write file : ",filename_config

    ## Vel trajectory ##
    filename_vel = filename_prefix + '_vel.csv'
    file_vel = open(filename_vel, "w")

    for k in range(N):
        line = str(timeline[k])
        line += write_vector(res.dq_t[:,k], delim)
        line += eol
        file_vel.write(line)

    file_vel.close()
    print "write file : ",filename_vel

    ## Acc trajectory ##
    filename_acc = filename_prefix + '_acc.csv'
    file_acc = open(filename_acc, "w")

    for k in range(N):
        line = str(timeline[k])
        line += write_vector(res.ddq_t[:,k], delim)
        line += eol
        file_acc.write(line)

    file_acc.close()
    print "write file : ",filename_acc


def export(cs,res):
    rp = RosPack()
    urdf = rp.get_path(cfg.Robot.packageName)+'/urdf/'+cfg.Robot.urdfName+cfg.Robot.urdfSuffix+'.urdf'
    if cfg.WB_VERBOSE:
        print "load robot : " ,urdf    
    #srdf = "package://" + package + '/srdf/' +  cfg.Robot.urdfName+cfg.Robot.srdfSuffix + '.srdf'
    robot = RobotWrapper(urdf, pin.StdVec_StdString(), pin.JointModelFreeFlyer(), False)
    if cfg.WB_VERBOSE:
        print "robot loaded in export OpenHRP"    
  
    results = computeWaistData(robot,res)
    path = cfg.EXPORT_PATH+"/openHRP/"+cfg.DEMO_NAME
    if not os.path.exists(path):
        os.makedirs(path)    
    q_openhrp_l = generateOpenHRPMotion(results, path, cfg.DEMO_NAME,useRefZMP=cfg.openHRP_useZMPref)
    writeKinematicsData(results, path, cfg.DEMO_NAME)
    