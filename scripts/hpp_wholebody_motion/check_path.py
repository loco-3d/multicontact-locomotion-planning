# Copyright 2018, LAAS-CNRS
# Author: Pierre Fernbach

from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from config import *

import numpy as np
def pinnochioQuaternion(q):
    assert len(q)>6, "config vector size must be superior to 7"
    w = q[3]
    q[3:6] = q[4:7]
    q[6] = w
    return q

def HPPQuaternion(q):
    assert len(q)>6, "config vector size must be superior to 7"
    w = q[6]
    q[4:7] = q[3:6]
    q[3] = w
    return q


class PathChecker():
    
    def __init__(self,r,fullBody,cs,q_t,t_t):
        self.r = r
        self.cs = cs
        self.q_t = q_t
        self.t_t = t_t
        self.size_conf = q_t.shape[0]
        self.fullBody = fullBody # with effector collision disabled
    
    
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
        
    def qAtT(self,t):
        for it in range(1,len(self.t_t)-1):
            if t>((self.t_t[it]+self.t_t[it-1])/2.) and t<=((self.t_t[it]+self.t_t[it+1])/2.):
                q = [0]*self.fullBody.getConfigSize() 
                q[:self.size_conf] = self.q_t[:,it].transpose().tolist()[0]
                q = HPPQuaternion(q)
                #print "closer t : ",self.t_t[it]
                #print "at id : ",it
                return q
        
    def showConfigAtTime(self,t):
        q = self.qAtT(t)      
        self.r(q) 
    
    def phaseOfT(self,t_switch):
        for i in range(self.cs.size()):
            p = self.cs.contact_phases[i]
            if t_switch>=p.time_trajectory[0] and t_switch<=p.time_trajectory[-1]:
                return i
        
    
    def phaseOfId(self,id):
        t_switch = self.t_t[id]
        return self.phaseOfT(t_switch)
        
     


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
    