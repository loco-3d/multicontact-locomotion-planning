# Copyright 2018, LAAS-CNRS
# Author: Pierre Fernbach

import numpy as np


class PathChecker():
    def __init__(self, fullBody, cs, nq, dt, verbose=False):
        self.cs = cs
        self.fullBody = fullBody  # with effector collision disabled
        self.nq = nq  # without extradof, size of configs in q_t
        self.configSize = fullBody.getConfigSize()
        self.dt = dt
        self.verbose = verbose
        self.extraDof = int(fullBody.client.robot.getDimensionExtraConfigSpace())


    def checkConfig(self, q_m):
        """
        convert to correct format for hpp, add extra dof if necessary
        return valid,message  : valid = bool, message = string
        :param q_m: configuration of size nq represented as a numpy array
        :return:
        """
        q = [0] * self.configSize
        q[:self.nq] = q_m.tolist()
        res = self.fullBody.isConfigValid(q)
        return res[0], res[1]


    def check_motion(self, q_t):
        """
        check if the given joint trajectory is valid, according to the Validation methods defined in self.fullBody
        if Verbose = True : check the complete motion before returning,
        if False stop at the first invalid
        :param q_t: any curve object from the package Curves, of dimension nq
        :return: a bool (True if the complete trajectory is valid, False otherwise) and the time of the first invalid configuration
        """
        always_valid = True
        first_invalid = None
        t = q_t.min()
        while t <= q_t.max():
            valid, mess = self.checkConfig(q_t(t))
            if not valid:
                if always_valid:  # first invalid config
                    always_valid = False
                    first_invalid = t
                if self.verbose:
                    print("Invalid config at t= ", t)
                    print(mess)
                else:
                    return always_valid, first_invalid
            t += self.dt
        return always_valid, first_invalid

    ############# old stuffs (need to be updated if necessary : #############
    """
    
    def qAtT(self, t, q_t):
    for it in range(1, len(q_t) - 1):
        if t > (self.dt * (it - 0.5)) and t <= (self.dt * (it + 0.5)):
            return q_t[it]


    def check_postures(self,verbose = True):
        bad_phases = []
        success_global=True
        for phaseId in range(self.cs.size()):
            phase = self.cs.contactPhases[phaseId]
            t_phase = phase.time_trajectory[0]
            q = self.qAtT(t_phase)
            res = self.fullBody.isConfigValid(q)
            if not res[0] :
                success_global = False
                if verbose:
                    print "Posture at Phase  "+str(phaseId)+" (t= "+str(t_phase)+") is in collision ! "
                    print res[1]
                bad_phases += [phaseId]            
        return success_global, bad_phases
    
    def check_predefined(self,time_predef,verbose = True):
        bad_phases = []
        success_global=True
        for phaseId in range(self.cs.size()):
            phase = self.cs.contactPhases[phaseId]
            t_phase = phase.time_trajectory[0]
            q = self.qAtT(t_phase)
            t = 0
            while t < time_predef : 
                q = self.qAtT(t_phase + t)
                res = self.fullBody.isConfigValid(q)
                if not res[0] :
                    success_global = False
                    if verbose:
                        print "Predef trajectory at Phase  "+str(phaseId)+" (t= "+str(t_phase)+") is in collision ! "
                        print res[1]
                    bad_phases += [phaseId]   
                t += 0.01
            t_phase = phase.time_trajectory[-1]
            t = 0
            while t < time_predef : 
                q = self.qAtT(t_phase - t)
                res = self.fullBody.isConfigValid(q)
                if not res[0] :
                    success_global = False
                    if verbose:
                        print "Predef trajectory at Phase  "+str(phaseId)+" (t= "+str(t_phase)+") is in collision ! "
                        print res[1]
                    bad_phases += [phaseId]   
                t += 0.01            
        return success_global, bad_phases    
    
    def check_valid(self,verbose = True):
        print "## Check if valid"        
        bad_phases = []
        q = [0]*self.fullBody.getConfigSize()
        success_global = True
        for i in range(self.q_t.shape[1]):
            q[:self.size_conf] = self.q_t[:,i].tolist()
            q = HPPQuaternion(q)
            res = self.fullBody.isConfigValid(q)
            if not res[0]:
                pid = self.phaseOfId(i)
                if not pid:
                    raise Exception( "Motion is invalid during the  << go to half sitting >> part")        
                success_global = False
                if verbose:
                    print "Invalid at iter "+str(i)+" ; t= ",str(self.t_t[i])
                    print res[1]
                if bad_phases.count(pid) == 0:
                    bad_phases += [pid]
                
        return success_global, bad_phases  
    
    def check_collision(self,verbose = True):
        print "## Collision check"        
        bad_phases = []
        q = [0]*self.fullBody.getConfigSize()
        success_global = True
        for i in range(self.q_t.shape[1]):
            q[:self.size_conf] = self.q_t[:,i].tolist()
            q = HPPQuaternion(q)
            res = self.fullBody.isConfigValid(q)
            if not res[0]:
                if res[1].count("Collision") > 0 :
                    success_global = False
                    if verbose:
                        print "In collision at iter "+str(i)+" ; t= ",str(self.t_t[i])
                        print res[1]
                    pid = self.phaseOfId(i)
                    if bad_phases.count(pid) == 0:
                        bad_phases += [pid]
                
        return success_global, bad_phases      
    
    def check_joint_limit(self,verbose = True):
        print "## Check joint limit"        
        bad_phases = []
        q = [0]*self.fullBody.getConfigSize()
        success_global = True
        for i in range(self.q_t.shape[1]):
            q[:self.size_conf] = self.q_t[:,i].tolist()
            q = HPPQuaternion(q)
            res = self.fullBody.isConfigValid(q)
            if not res[0]:
                if res[1].count("out of range") > 0 :
                    success_global = False
                    if verbose:
                        print "don't respect joint limit at iter "+str(i)+" ; t= ",str(self.t_t[i])
                        print res[1]
                    pid = self.phaseOfId(i)
                    if bad_phases.count(pid) == 0:
                        bad_phases += [pid]
                
        return success_global, bad_phases  
    
        
    def showConfigAtId(self,id):
        q = [0]*self.fullBody.getConfigSize()
        q[:self.size_conf] = self.q_t[:,id].tolist()
        q = HPPQuaternion(q)        
        self.r(q)

        
    def showConfigAtTime(self,t):
        q = self.qAtT(t)      
        self.r(q) 
  

    def checkAdditionalJointsConstraints(self,list_of_id,mins,maxs):
        valid = True
        for i in range(self.q_t.shape[1]):
            for ijoint in range(len(list_of_id)):
                if self.q_t[list_of_id[ijoint],i] < mins[ijoint]:
                    valid = False
                    print "MIN bound violated for joint "+str(list_of_id[ijoint])+" value = "+str(self.q_t[list_of_id[ijoint],i])+" at time = "+str(self.t_t[i])
                if self.q_t[list_of_id[ijoint],i] > maxs[ijoint]:
                    valid = False
                    print "MAX bound violated for joint "+str(list_of_id[ijoint])+" value = "+str(self.q_t[list_of_id[ijoint],i])+" at time = "+str(self.t_t[i])
        
        
        return valid
    """
