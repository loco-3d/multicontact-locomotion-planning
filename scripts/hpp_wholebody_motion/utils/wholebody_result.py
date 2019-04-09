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
        self.wrench_t = np.matrix(np.zeros([6,N]))
        self.zmp_t = np.matrix(np.zeros([6,N]))
        
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
            self.effector_trajectories.update({ee:np.matrix(np.zeros([12,N]))}) #FIXME : use SE3, (store in R^7 or in R^4x4 ??)
            self.effector_references.update({ee:np.matrix(np.zeros([12,N]))}) #FIXME : use SE3
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
                reduced_interval = interval[:n+1]
                new_intervals += [reduced_interval]
                break
        return new_intervals
            
    def resize(self,N):
        self.N = N
        self.t_t= np.resize(self.t_t,[N])
        self.q_t= np.resize(self.q_t,[self.nq,N])
        self.dq_t= np.resize(self.dq_t ,[self.nv,N])
        self.ddq_t= np.resize(self.ddq_t ,[self.nv,N])
        self.tau_t= np.resize(self.tau_t ,[self.nv-6,N])
        self.c_t= np.resize(self.c_t ,[3,N])
        self.dc_t= np.resize(self.dc_t ,[3,N])
        self.ddc_t= np.resize(self.ddc_t ,[3,N])
        self.L_t= np.resize(self.L_t ,[3,N])
        self.dL_t= np.resize(self.dL_t ,[3,N])
        self.c_tracking_error= np.resize(self.c_tracking_error ,[3,N])
        self.c_reference= np.resize(self.c_reference ,[3,N])
        self.wrench_t= np.resize(self.wrench_t ,[6,N])
        self.zmp_t= np.resize(self.zmp_t ,[3,N])      
        for ee in self.eeNames : 
            self.contact_forces[ee]= np.resize(self.contact_forces[ee] ,[12,N])     
            self.contact_normal_force[ee]= np.resize(self.contact_normal_force[ee] ,[1,N])                            
            self.effector_trajectories[ee]= np.resize(self.effector_trajectories[ee] ,[12,N]) 
            self.effector_references[ee]= np.resize(self.effector_references[ee] ,[12,N]) 
            self.effector_tracking_error[ee]= np.resize(self.effector_tracking_error[ee] ,[6,N])
            self.contact_activity[ee]= np.resize(self.contact_activity[ee] ,[1,N])
        self.phases_intervals = self.resizePhasesIntervals(N)
        return self