import numpy as np
class Result:
    
    
    def __init__(self,nq,nv,dt,eeNames,N=None,cs=None,t_begin = 0,nu = None):
        self.dt = dt
        if cs :
            self.N = int(round(cs.contact_phases[-1].time_trajectory[-1]/self.dt)) + 1             
        elif N :
            self.N = N            
        else : 
            raise RuntimeError("Result constructor must be called with either a contactSequence object or a number of points")
        N = self.N
        self.nq = nq
        self.nv = nv
        if not nu : 
            nu = nv -6 
        self.nu = nu
        self.t_t = np.array([t_begin + i*self.dt for i in range(N)])
        self.q_t = np.matrix(np.zeros([self.nq,N]))
        self.dq_t = np.matrix(np.zeros([self.nv,N]))
        self.ddq_t = np.matrix(np.zeros([self.nv,N]))
        self.tau_t = np.matrix(np.zeros([self.nu,N]))
        self.c_t = np.matrix(np.zeros([3,N]))  
        self.dc_t = np.matrix(np.zeros([3,N]))
        self.ddc_t = np.matrix(np.zeros([3,N]))
        self.L_t = np.matrix(np.zeros([3,N]))
        self.dL_t = np.matrix(np.zeros([3,N]))
        self.c_reference = np.matrix(np.zeros([3,N]))
        self.dc_reference = np.matrix(np.zeros([3,N]))
        self.ddc_reference = np.matrix(np.zeros([3,N])) 
        self.L_reference = np.matrix(np.zeros([3,N]))
        self.dL_reference = np.matrix(np.zeros([3,N]))        
        self.wrench_t = np.matrix(np.zeros([6,N]))
        self.wrench_reference = np.matrix(np.zeros([6,N]))        
        self.zmp_t = np.matrix(np.zeros([3,N]))
        self.zmp_reference = np.matrix(np.zeros([3,N]))
        self.eeNames = eeNames
        self.contact_forces = {}
        self.contact_normal_force={}
        self.effector_trajectories = {}
        self.effector_references = {}
        self.contact_activity = {}
        for ee in self.eeNames : 
            self.contact_forces.update({ee:np.matrix(np.zeros([12,N]))}) 
            self.contact_normal_force.update({ee:np.matrix(np.zeros([1,N]))})              
            self.effector_trajectories.update({ee:np.matrix(np.zeros([12,N]))}) 
            self.effector_references.update({ee:np.matrix(np.zeros([12,N]))})
            self.contact_activity.update({ee:np.matrix(np.zeros([1,N]))})
        if cs:
            self.phases_intervals = self.buildPhasesIntervals(cs)    
        else :
            print "Result constructor called without contactSequence object, phase_interval member not initialized"
     
    # By definition of a contact sequence, at the state at the transition time between two contact phases
    # belong to both contact phases
    # This mean that phases_intervals[i][-1] == phases_intervals[i+1][0]
    def buildPhasesIntervals(self,cs):
        intervals = []
        k = 0
        for phase in cs.contact_phases :
            duration = phase.time_trajectory[-1]-phase.time_trajectory[0]
            n_phase = int(round(duration/self.dt))
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
            self.effector_trajectories[ee][:,k] =other.effector_trajectories[ee][:,k_other]
            self.effector_references[ee][:,k] =other.effector_references[ee][:,k_other]
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
            self.contact_activity[ee] = self.contact_activity[ee][:,:N]
        self.phases_intervals = self.resizePhasesIntervals(N)
        return self
    
    def exportNPZ(self,path,name):
        import os
        if not os.path.exists(path):
            os.makedirs(path)
        filename = path+"/"+name       
        np.savez_compressed(filename,N=self.N,nq=self.nq,nv=self.nv,nu = self.nu,dt=self.dt,t_t=self.t_t,
                 q_t=self.q_t,dq_t=self.dq_t,ddq_t=self.ddq_t,tau_t=self.tau_t,
                 c_t=self.c_t,dc_t=self.dc_t,ddc_t=self.ddc_t,L_t=self.L_t,dL_t=self.dL_t,
                 c_reference=self.c_reference,dc_reference=self.dc_reference,ddc_reference=self.ddc_reference,
                 L_reference=self.L_reference,dL_reference=self.dL_reference,
                 wrench_t=self.wrench_t,zmp_t=self.zmp_t,wrench_reference=self.wrench_reference,zmp_reference=self.zmp_reference,
                 eeNames=self.eeNames,contact_forces=self.contact_forces,contact_normal_force=self.contact_normal_force,
                 effector_trajectories=self.effector_trajectories,effector_references=self.effector_references,
                 contact_activity=self.contact_activity,phases_intervals=self.phases_intervals)
        
        print "Results exported to ",filename
        
    def qAtT(self,t):
        k = int(round(t/self.dt))
        assert self.t_t[k] == t and "Error in computation of time in Result struct."
        return self.q_t[:,k]

def loadFromNPZ(filename):
    f=np.load(filename)
    N = f['N'].tolist()
    nq = f['nq'].tolist()
    nv = f['nv'].tolist()
    nu = f['nu'].tolist()
    dt = f['dt'].tolist()
    eeNames = f['eeNames'].tolist()
    res = Result(nq,nv,dt,eeNames=eeNames,N=N,nu=nu)
    res.t_t = f['t_t']
    res.q_t  =np.asmatrix(f['q_t'])
    res.dq_t =np.asmatrix(f['dq_t'])
    res.ddq_t =np.asmatrix(f['ddq_t'])
    res.tau_t =np.asmatrix(f['tau_t'])
    res.c_t =np.asmatrix(f['c_t'])
    res.dc_t =np.asmatrix(f['dc_t'])
    res.ddc_t =np.asmatrix(f['ddc_t'])
    res.L_t =np.asmatrix(f['L_t'])
    res.dL_t =np.asmatrix(f['dL_t'])
    res.c_reference =np.asmatrix(f['c_reference'])
    res.dc_reference =np.asmatrix(f['dc_reference'])
    res.ddc_reference =np.asmatrix(f['ddc_reference'])
    res.L_reference =np.asmatrix(f['L_t'])
    res.dL_reference =np.asmatrix(f['dL_t'])
    res.wrench_t =np.asmatrix(f['wrench_t'])
    res.zmp_reference =np.asmatrix(f['zmp_t'])
    res.wrench_reference =np.asmatrix(f['wrench_t'])
    res.zmp_t =np.asmatrix(f['zmp_t'])        
    res.contact_forces =f['contact_forces'].tolist()
    res.contact_normal_force = f['contact_normal_force'].tolist()            
    res.effector_trajectories =f['effector_trajectories'].tolist()
    res.effector_references =f['effector_references'].tolist()
    res.contact_activity =f['contact_activity'].tolist()
    res.phases_intervals = f['phases_intervals'].tolist()
    f.close()
    for ee in res.eeNames : 
        res.contact_forces[ee] = np.asmatrix(res.contact_forces[ee])
        res.contact_normal_force[ee] = np.asmatrix(res.contact_normal_force[ee])                           
        res.effector_trajectories[ee] = np.asmatrix(res.effector_trajectories[ee])
        res.effector_references[ee] = np.asmatrix(res.effector_references[ee])
        res.contact_activity[ee] = np.asmatrix(res.contact_activity[ee])   
    return res