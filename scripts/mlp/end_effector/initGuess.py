import mlp.config as cfg
from mlp.utils.requirements import Requirements
from multicontact_api import ContactSequence
from mlp.utils.util import SE3FromConfig

#end_effector_initGuess_method_available = ["none","load","smoothedFoot", "bezierPredef"]

class Inputs(Requirements):
    consistentContacts = True
    timings = True

class Outputs(Inputs):
    effectorTrajectories = True

method_initGuess = cfg.end_effector_initGuess_method

if method_initGuess == "smoothedFoot":
    from .smoothedFoot import generateEndEffectorTraj, effectorCanRetry
elif method_initGuess == "bezierPredef":
    from .bezier_predef import generateSmoothBezierTraj as generateEndEffectorTraj
    from .bezier_predef import effectorCanRetry
else:
    raise ValueError("method type " + str(method_initGuess) + " doesn't exist for end-effector initial guess trajectory generation")

def effectorPlacementFromPhaseConfig(phase, eeName, fullBody):
    if fullBody is None :
        raise RuntimeError("Cannot compute the effector placement from the configuration without initialized fullBody object.")
    if not phase.q_init.any():
        raise RuntimeError("Cannot compute the effector placement as the initial configuration is not initialized in the ContactPhase.")

    fullBody.setCurrentConfig(phase.q_init.tolist())
    return SE3FromConfig(fullBody.getJointPosition(eeName))


def generateEffectorTrajectoriesForSequence(cs, fullBody = None):
    """
    Generate an effector trajectory for each effectors which are going to be in contact in the next phase
    :param cs:
    :return:
    """
    cs_res = ContactSequence(cs)
    effectors = cs_res.getAllEffectorsInContact()
    previous_phase = None
    for pid in range(cs_res.size()-1): # -1 as last phase never have effector trajectories
        phase = cs_res.contactPhases[pid]
        next_phase = cs_res.contactPhases[pid+1]
        if pid > 0 :
            previous_phase = cs_res.contactPhases[pid-1]

        for eeName in effectors:
            if not phase.isEffectorInContact(eeName) and next_phase.isEffectorInContact(eeName):
                # eeName will be in compute in the next phase, a trajectory should be added in the current phase
                placement_end = next_phase.contactPatch(eeName).placement
                time_interval = [phase.timeInitial, phase.timeFinal]
                if previous_phase is not None and previous_phase.isEffectorInContact(eeName):
                    placement_init = previous_phase.contactPatch(eeName).placement
                else:
                    placement_init = effectorPlacementFromPhaseConfig(phase,eeName,fullBody)
                # build the trajectory :
                traj = generateEndEffectorTraj(time_interval,placement_init,placement_end)
                phase.addEffectorTrajectory(eeName,traj)


    return cs_res


