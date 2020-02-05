import mlp.config as cfg

#end_effector_method = ["smoothedFoot", "bezierPredef", "bezierConstrained", "limbRRT", "limbRRToptimized"]
method = cfg.end_effector_method
# prototype must be : in : (time_interval,placement_init,placement_end,numTry,q_t=None,phase_previous=None,phase=None,phase_next=None,fullBody=None,eeName=None,viewer=None) out : Trajectorie
if method == "limbRRT":
    from .limb_rrt import generateLimbRRTTraj as generateEndEffectorTraj
    from .limb_rrt import effectorCanRetry, Inputs, Outputs
elif method == "limbRRToptimized":
    from .limb_rrt import generateLimbRRTOptimizedTraj as generateEndEffectorTraj
    from .limb_rrt import effectorCanRetry, Inputs, Outputs
else:
    raise ValueError("method type " + str(method) + " doesn't exist for end-effector trajectory generation")
