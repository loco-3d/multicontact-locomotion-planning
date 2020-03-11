from mlp.utils.trajectories import SmoothedFootTrajectory
from  mlp.utils.requirements import Requirements

class EffectorInputsSmoothedfoot(Requirements):
    consistentContacts = True
    timings = True
class EffectorOutputsSmoothedfoot(EffectorInputsSmoothedfoot):
    effectorTrajectories = True

def generateEndEffectorTraj(cfg,
                            time_interval,
                            placement_init,
                            placement_end,
                            numTry=None,
                            q_t=None,
                            phase_previous=None,
                            phase=None,
                            phase_next=None,
                            fullBody=None,
                            eeName=None,
                            viewer=None):
    if numTry > 0:
        raise ValueError(
            "SmoothedFootTrajectory will always produce the same trajectory, cannot be called with numTry > 0 ")
    return SmoothedFootTrajectory(time_interval, [placement_init, placement_end])

def effectorCanRetry():
    return False