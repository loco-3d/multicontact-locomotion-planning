from hpp_centroidal_dynamics import Equilibrium, EquilibriumAlgorithm, SolverLP
import hpp_bezier_com_traj as com_traj
from mlp.utils.util import buildRectangularContactPoints, computeContactNormal
import numpy as np
from ndcurves import bezier
from numpy import array
MARGIN = 0.02 # reduction of the size of the feets in each direction
USE_AM = True

def zeroStepCapturability(phase, cfg):
    """
    Try to compute a centroidal trajectory that bring the given phase to a complete stop without contact changes.
    I take the initial centroidal state and the duration from the state. As long as the contacts.
    If successfull, it fill the final centroidal state in the phase and the centroidal trajectories
    :param phase: the current ContactPhase, must have contacts and initial centroidal values defined
    :return: [success, phase]
    """
    # initialize the equilibrium lib
    eq = Equilibrium("equilibrium", cfg.MASS, 4, SolverLP.SOLVER_LP_QPOASES, True, 10, True)
    # set the contacts from the phase:
    for eeName in phase.effectorsInContact():
        placement = phase.contactPatch(eeName).placement
        size_feet = cfg.IK_eff_size[eeName][::]
        size_feet[0] -= 2. * MARGIN
        size_feet[1] -= 2. * MARGIN
        points = buildRectangularContactPoints(size_feet,cfg.Robot.dict_offset[eeName])
        positions = np.zeros([4,3])
        for i in range(4):
            positions[i,:] = (placement.rotation @ points[:,i]).T + placement.translation
        normals = array([computeContactNormal(placement)] * 4)
        # one position or normal per line of the matrix ! (opposite convention to the mlp package)
        eq.setNewContacts(positions, normals, cfg.MU, EquilibriumAlgorithm.EQUILIBRIUM_ALGORITHM_PP)
    # try to solve the 0 step capturability :
    # The boolean set if we use the angular moment or not, -1 is used to set the continuous mode
    if USE_AM:
        L0 = phase.L_init
    else:
        L0 = np.zeros(3)
    print("Initial com : ", phase.c_init)
    print("Initial AM : ", phase.L_init)
    #print("phase duration used :", phase.duration )
    res = com_traj.zeroStepCapturability(eq, phase.c_init, phase.dc_init, L0, USE_AM, phase.duration, -1)
    if not res.success:
        return False, phase
    #print("zero step, res.x : ", res.x)
    print("zero step, com final : ", res.c_of_t(res.c_of_t.max()))
    # save the results inside the phase
    # update the definition interval of the bezier curve :
    wp = res.c_of_t.waypoints()
    phase.c_t = bezier(wp, phase.timeInitial, phase.timeFinal)
    phase.dc_t = phase.c_t.compute_derivate(1)
    phase.ddc_t = phase.c_t.compute_derivate(2)
    wp = res.dL_of_t.waypoints()
    # see https://github.com/humanoid-path-planner/hpp-bezier-com-traj/blob/v4.8.0/src/solve_0_step.cpp#L208
    # the wp must be devided by the duration to get the same results
    wp /= phase.duration
    phase.dL_t = bezier(wp, phase.timeInitial, phase.timeFinal)
    wp = phase.dL_t.compute_primitive(1).waypoints()
    wp *= phase.duration
    for i in range(wp.shape[1]):
        wp[:,i] += phase.L_init
    phase.L_t =  bezier(wp, phase.timeInitial, phase.timeFinal)

    # set the final centroidal values from the trajectory:
    tmax = phase.timeFinal
    phase.c_final = phase.c_t(tmax)
    phase.dc_final = phase.dc_t(tmax)
    phase.ddc_final = phase.ddc_t(tmax)
    phase.L_final = phase.L_t(tmax)
    phase.dL_final = phase.dL_t(tmax)
    return True, phase

