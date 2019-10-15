# Copyright 2018, LAAS-CNRS
# Author: Pierre Fernbach

import mlp.config as cfg
import numpy as np

class PathChecker():
    
    def __init__(self,fullBody,cs,nq,verbose = False):
        self.cs = cs
        self.fullBody = fullBody # with effector collision disabled
        self.nq = nq # without extradof, size of configs in q_t 
        self.configSize = fullBody.getConfigSize()
        self.dt = cfg.IK_dt
        self.check_step = int(cfg.CHECK_DT/cfg.IK_dt)
        if self.check_step < 1 :
            self.check_step = 1
        self.verbose = verbose
        self.extraDof =  int(fullBody.client.robot.getDimensionExtraConfigSpace())
    
    # convert to correct format for hpp, add extra dof if necessary
    # return valid,message  : valid = bool, message = string
    def checkConfig(self,q_m):
        q = [0]*self.configSize
        q[:self.nq] = q_m.T.tolist()[0]
        res = self.fullBody.isConfigValid(q)
        return res[0],res[1]
    
    
    
    def qAtT(self,t,q_t):
        for it in range(1,len(q_t)-1):
            if t>(self.dt*(it-0.5)) and t<=(self.dt*(it+0.5)):
                return q_t[it]    
    
    
  
    def phaseOfT(self,t_switch):
        for i in range(self.cs.size()):
            p = self.cs.contact_phases[i]
            if t_switch>=p.time_trajectory[0] and t_switch<=p.time_trajectory[-1]:
                return i
        
    
    def phaseOfId(self,id):
        t_switch = self.dt*float(id)
        return self.phaseOfT(t_switch)
        
    # return a bool (true = valid) and the time of the first invalid q (None if always valid): 
    # if Verbose = True : check the complete motion before returning,
    # if False stop at the first invalid
    def check_motion(self,q_t):
        always_valid = True
        first_invalid = None
        print "q_t len : ",q_t.shape[1]
        i = -self.check_step
        while i < q_t.shape[1]-1 :
            i += self.check_step
            if i >= q_t.shape[1]:
                i = q_t.shape[1]-1
            print "i in check motion : ",i
            valid,mess = self.checkConfig(q_t[:,i])
            if not valid : 
                if always_valid : # first invalid config
                    always_valid = False
                    first_invalid = self.dt*(float(i))
                if self.verbose:
                    print "Invalid config at t= ",self.dt*(float(i))
                    print mess                    
                else:
                    return always_valid,first_invalid
        return always_valid,first_invalid
    
    
    
    ############# old stuffs (need to be updated if necessary : #############
    """
    def check_postures(self,verbose = True):
        bad_phases = []
        success_global=True
        for phaseId in range(self.cs.size()):
            phase = self.cs.contact_phases[phaseId]
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
            phase = self.cs.contact_phases[phaseId]
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
            q[:self.size_conf] = self.q_t[:,i].transpose().tolist()[0]
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
            q[:self.size_conf] = self.q_t[:,i].transpose().tolist()[0]
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
            q[:self.size_conf] = self.q_t[:,i].transpose().tolist()[0]
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
        q[:self.size_conf] = self.q_t[:,id].transpose().tolist()[0]
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