import numpy as np
import hpp_wholebody_motion.config as cfg
class Result:
    
    nq = cfg.nq
    nv = cfg.nv
    def __init__(self,cs,eeNames=[],t_begin = 0):
        N = int(round(cs.contact_phases[-1].time_trajectory[-1]/cfg.IK_dt)) + 1 
        self.N = N
        self.t_t = np.array([t_begin + i*cfg.IK_dt for i in range(N)])
        self.q_t = np.matrix(np.zeros([self.nq,N]))
        self.dq_t = np.matrix(np.zeros([self.nv,N]))
        self.ddq_t = np.matrix(np.zeros([self.nv,N]))
        self.tau_t = np.matrix(np.zeros([self.nv-6,N]))
        self.c_t = np.matrix(np.zeros([3,N]))  
        self.dc_t = np.matrix(np.zeros([3,N]))
        self.ddc_t = np.matrix(np.zeros([3,N]))
        self.L_t = np.matrix(np.zeros([3,N]))
        self.dL_t = np.matrix(np.zeros([3,N]))
        self.c_tracking_error = np.matrix(np.zeros([3,N]))
        self.c_reference = np.matrix(np.zeros([3,N]))
        self.dc_reference = np.matrix(np.zeros([3,N]))
        self.ddc_reference = np.matrix(np.zeros([3,N])) 
        self.L_reference = np.matrix(np.zeros([3,N]))
        self.dL_reference = np.matrix(np.zeros([3,N]))        
        self.wrench_t = np.matrix(np.zeros([6,N]))
        self.wrench_reference = np.matrix(np.zeros([6,N]))        
        self.zmp_t = np.matrix(np.zeros([3,N]))
        self.zmp_reference = np.matrix(np.zeros([3,N]))
        if len(eeNames)==0:
            eeNames = cfg.Robot.dict_limb_joint.values()
        self.eeNames = eeNames
        self.contact_forces = {}
        self.contact_normal_force={}
        self.effector_trajectories = {}
        self.effector_references = {}
        self.effector_tracking_error = {}
        self.contact_activity = {}
        for ee in self.eeNames : 
            self.contact_forces.update({ee:np.matrix(np.zeros([12,N]))}) 
            self.contact_normal_force.update({ee:np.matrix(np.zeros([1,N]))})              
            self.effector_trajectories.update({ee:np.matrix(np.zeros([12,N]))}) 
            self.effector_references.update({ee:np.matrix(np.zeros([12,N]))})
            self.effector_tracking_error.update({ee:np.matrix(np.zeros([6,N]))})
            self.contact_activity.update({ee:np.matrix(np.zeros([1,N]))})
        self.phases_intervals = self.buildPhasesIntervals(cs)    
     
    # By definition of a contact sequence, at the state at the transition time between two contact phases
    # belong to both contact phases
    # This mean that phases_intervals[i][-1] == phases_intervals[i+1][0]
    def buildPhasesIntervals(self,cs):
        intervals = []
        k = 0
        dt = cfg.IK_dt
        for phase in cs.contact_phases :
            duration = phase.time_trajectory[-1]-phase.time_trajectory[0]
            n_phase = int(round(duration/dt))
            interval = range(k,k+n_phase+1)
            k += n_phase
            intervals += [interval]
        return intervals
    
    def resizePhasesIntervals(self,N):
        new_intervals = []
        for interval in self.phases_intervals:
            if interval[-1] <= N:
                new_intervals+= [interval]
            else :
                n = N - interval[0]
                reduced_interval = interval[:n]
                new_intervals += [reduced_interval]
                break
        return new_intervals
    
    def fillAllValues(self,k,other,k_other=None):
        if not k_other:
            k_other = k
        self.t_t[k] = other.t_t[k_other]
        self.q_t [:,k] =other.q_t[:,k_other]
        self.dq_t[:,k] =other.dq_t[:,k_other]
        self.ddq_t[:,k] =other.ddq_t[:,k_other]
        self.tau_t[:,k] =other.tau_t[:,k_other]
        self.c_t[:,k] =other.c_t[:,k_other]
        self.dc_t[:,k] =other.dc_t[:,k_other]
        self.ddc_t[:,k] =other.ddc_t[:,k_other]
        self.L_t[:,k] =other.L_t[:,k_other]
        self.dL_t[:,k] =other.dL_t[:,k_other]
        self.c_tracking_error[:,k] =other.c_tracking_error[:,k_other]
        self.c_reference[:,k] =other.c_reference[:,k_other]
        self.dc_reference[:,k] =other.dc_reference[:,k_other] 
        self.ddc_reference[:,k] =other.ddc_reference[:,k_other]
        self.L_reference[:,k] =other.L_t[:,k_other]
        self.dL_reference[:,k] =other.dL_t[:,k_other]        
        self.wrench_t[:,k] =other.wrench_t[:,k_other]
        self.zmp_reference[:,k] =other.zmp_t[:,k_other]
        self.wrench_reference[:,k] =other.wrench_t[:,k_other]
        self.zmp_t[:,k] =other.zmp_t[:,k_other]        
        for ee in self.eeNames : 
            self.contact_forces[ee][:,k] =other.contact_forces[ee][:,k_other]
            self.contact_normal_force[ee][:,k] = other.contact_normal_force[ee][:,k_other]            
            other.effector_trajectories[ee][:,k] =other.effector_trajectories[ee][:,k_other]
            other.effector_references[ee][:,k] =other.effector_references[ee][:,k_other]
            self.effector_tracking_error[ee][:,k] =other.effector_tracking_error[ee][:,k_other]
            self.contact_activity[ee][:,k] =other.contact_activity[ee][:,k_other]
    
            
    def resize(self,N):
        self.N = N
        self.t_t = self.t_t[:N]
        self.q_t = self.q_t[:,:N]
        self.dq_t = self.dq_t[:,:N]
        self.ddq_t = self.ddq_t[:,:N]
        self.tau_t = self.tau_t[:,:N]
        self.c_t = self.c_t[:,:N]
        self.dc_t = self.dc_t[:,:N]
        self.ddc_t = self.ddc_t[:,:N]
        self.L_t = self.L_t[:,:N]
        self.dL_t = self.dL_t[:,:N]
        self.c_tracking_error = self.c_tracking_error[:,:N]
        self.c_reference = self.c_reference[:,:N]
        self.dc_reference = self.dc_reference[:,:N]
        self.ddc_reference = self.ddc_reference[:,:N]
        self.L_reference = self.L_t[:,:N]
        self.dL_reference = self.dL_t[:,:N]        
        self.wrench_t = self.wrench_t[:,:N]
        self.zmp_t = self.zmp_t[:,:N] 
        self.wrench_reference = self.wrench_t[:,:N]
        self.zmp_reference = self.zmp_t[:,:N]        
        for ee in self.eeNames : 
            self.contact_forces[ee] = self.contact_forces[ee][:,:N]     
            self.contact_normal_force[ee] = self.contact_normal_force[ee][:,:N]                            
            self.effector_trajectories[ee] = self.effector_trajectories[ee][:,:N] 
            self.effector_references[ee] = self.effector_references[ee][:,:N] 
            self.effector_tracking_error[ee] = self.effector_tracking_error[ee][:,:N]
            self.contact_activity[ee] = self.contact_activity[ee][:,:N]
        self.phases_intervals = self.resizePhasesIntervals(N)
        return self