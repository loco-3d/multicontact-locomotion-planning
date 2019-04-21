import mlp.config as cfg 

#end_effector_method = ["smoothedFoot", "bezierPredef", "bezierConstrained", "limbRRT", "limbRRToptimized"]
method = cfg.end_effector_method
# prototype must be : in : (time_interval,placement_init,placement_end,numTry,q_t=None,phase_previous=None,phase=None,phase_next=None,fullBody=None,eeName=None,viewer=None) out : Trajectorie
if method == "smoothedFoot":
    from mlp.utils.trajectories import SmoothedFootTrajectory
    def generateEndEffectorTraj(time_interval,placement_init,placement_end,numTry=None,q_t=None,phase_previous=None,phase=None,phase_next=None,fullBody=None,eeName=None,viewer=None):
        if numTry > 0 :
            raise ValueError("SmoothedFootTrajectory will always produce the same trajectory, cannot be called with numTry > 0 ")        
        return SmoothedFootTrajectory(time_interval, [placement_init,placement_end]) 
elif method == "bezierPredef":
    from .bezier_predef import generateSmoothBezierTraj as generateEndEffectorTraj
elif method == "bezierConstrained":
    from .bezier_constrained import generateConstrainedBezierTraj as generateEndEffectorTraj
elif method == "limbRRT":
    from .limb_rrt import generateLimbRRTTraj as generateEndEffectorTraj
elif method == "limbRRToptimized":
    from .limb_rrt import generateLimbRRTOptimizedTraj as generateEndEffectorTraj
else : 
    raise ValueError("method type "+str(method)+" doesn't exist for end-effector trajectory generation")
