import numpy as np
import hpp_wholebody_motion.config as cfg
class Result:
    
    nq = cfg.nq
    nv = cfg.nv
    def __init__(self,N=0,eeNames=[],t_begin = 0):
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
            
            
    def resize(self,N):
        self.N = N
        self.t_t.resize([1,N])
        self.q_t.resize([self.nq,N])
        self.dq_t.resize([self.nv,N])
        self.ddq_t.resize([self.nv,N])
        self.tau_t.resize([self.nv-6,N])
        self.c_t.resize([3,N])
        self.dc_t.resize([3,N])
        self.ddc_t.resize([3,N])
        self.L_t.resize([3,N])
        self.dL_t.resize([3,N])
        self.c_tracking_error.resize([3,N])
        self.c_reference.resize([3,N])
        
        for ee in self.eeNames : 
            self.contact_forces[ee].resize([12,N])     
            self.contact_normal_force[ee].resize([1,N])                            
            self.effector_trajectories[ee].resize([12,N]) 
            self.effector_references[ee].resize([12,N]) 
            self.effector_tracking_error[ee].resize([6,N])
            self.contact_activity[ee].resize([1,N])
        