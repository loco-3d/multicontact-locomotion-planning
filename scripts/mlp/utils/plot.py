import numpy as np
import os
import pinocchio as pin
from mlp.utils.util import discretizeCurve, discretizeSE3CurveTranslation, discretizeDerivateCurve
import matplotlib
matplotlib.use("Qt4agg")
import matplotlib.pyplot as plt
from multiprocessing import Process
from mlp.utils.requirements import Requirements
pin.switchToNumpyArray()
plt.ioff()


def show(plt):
    plt.show()


def show_non_blocking(plt):
    p = Process(target=show, args=([plt]))
    p.start()


def addVerticalLineContactSwitch(cs, plt, linestyle="-.", color='k'):
    for phase in cs.contactPhases:
        plt.axvline(phase.timeFinal, linestyle=linestyle, color=color)



def plotEffectorTrajectoryWithReference(cs_ref, cs, dt):
    labels = [
        "x (m)", "y (m)", "z (m)", "dx (m/s)", "dy (m/s)", "dz (m/s)", "ddx (m/s^2)", "ddy (m/s^2)", "ddz (m/s^2)"
    ]
    colors = ['r', 'g', 'b']

    for eeName in cs.getAllEffectorsInContact():
        traj = cs.concatenateEffectorTrajectories(eeName)
        pos, timeline = discretizeSE3CurveTranslation(traj, dt)
        vel = discretizeDerivateCurve(traj, dt, 1)[0][:3, :]
        acc = discretizeDerivateCurve(traj, dt, 2)[0][:3, :]
        values = np.vstack([pos, vel, acc])
        traj_ref = cs_ref.concatenateEffectorTrajectories(eeName)
        pos_ref = discretizeSE3CurveTranslation(traj_ref, dt)[0]
        vel_ref = discretizeDerivateCurve(traj_ref, dt, 1)[0][:3, :]
        acc_ref = discretizeDerivateCurve(traj_ref, dt, 2)[0][:3, :]
        values_ref = np.vstack([pos_ref, vel_ref, acc_ref])
        fig, ax = plt.subplots(3, 3)
        fig.canvas.set_window_title("Trajectory for effector " + eeName + " (dashed = reference)")
        fig.suptitle("Trajectory for effector " + eeName + " (dashed = reference)", fontsize=20)
        for i in range(3):  # line = pos,vel,acc
            for j in range(3):  # col = x,y,z
                ax_sub = ax[i, j]
                ax_sub.plot(timeline.T, values[i * 3 + j, :].T, color=colors[j])
                ax_sub.plot(timeline.T, values_ref[i * 3 + j, :].T, color=colors[j], linestyle=":")
                ax_sub.set_xlabel('time (s)')
                ax_sub.set_ylabel(labels[i * 3 + j])
                addVerticalLineContactSwitch(cs, ax_sub)
                ax_sub.grid(False)
    plt.show(block=False)



def plotEffectorTrajectory(cs, dt, name_prefix = ""):
    labels = [
        "x (m)", "y (m)", "z (m)", "dx (m/s)", "dy (m/s)", "dz (m/s)", "ddx (m/s^2)", "ddy (m/s^2)", "ddz (m/s^2)"
    ]
    colors = ['r', 'g', 'b']

    for eeName in cs.getAllEffectorsInContact():
        traj = cs.concatenateEffectorTrajectories(eeName)
        pos, timeline = discretizeSE3CurveTranslation(traj, dt)
        vel = discretizeDerivateCurve(traj, dt, 1)[0][:3, :]
        acc = discretizeDerivateCurve(traj, dt, 2)[0][:3, :]
        values = np.vstack([pos, vel, acc])
        fig, ax = plt.subplots(3, 3)
        fig.canvas.set_window_title(name_prefix+" trajectory for effector " + eeName)
        fig.suptitle(name_prefix+" trajectory for effector " + eeName, fontsize=20)
        for i in range(3):  # line = pos,vel,acc
            for j in range(3):  # col = x,y,z
                ax_sub = ax[i, j]
                ax_sub.plot(timeline.T, values[i * 3 + j, :].T, color=colors[j])
                ax_sub.set_xlabel('time (s)')
                ax_sub.set_ylabel(labels[i * 3 + j])
                addVerticalLineContactSwitch(cs, ax_sub)
                ax_sub.grid(False)
    plt.show(block=False)




def plotEffectorError(cs_ref, cs, dt):
    labels = ["x (m)", "y (m)", "z (m)"]
    colors = ['r', 'g', 'b']
    for eeName in cs.getAllEffectorsInContact():
        traj = cs.concatenateEffectorTrajectories(eeName)
        traj_ref = cs_ref.concatenateEffectorTrajectories(eeName)
        assert traj.min() == traj_ref.min(), "In plotEffectorError : reference and real one do not have the same timeline"
        assert traj.max() == traj_ref.max(), "In plotEffectorError : reference and real one do not have the same timeline"

        pos_ref, timeline = discretizeSE3CurveTranslation(traj_ref, dt)
        pos = discretizeSE3CurveTranslation(traj, dt)[0]
        error = pos - pos_ref
        fig, ax = plt.subplots(3, 1)
        fig.canvas.set_window_title("Effector tracking error : " + eeName)
        fig.suptitle("Effector tracking error : " + eeName, fontsize=20)
        for i in range(3):  # line = x,y,z
            ax_sub = ax[i]
            ax_sub.plot(timeline.T, error[i, :].T, color=colors[i])
            ax_sub.set_xlabel('time (s)')
            ax_sub.set_ylabel(labels[i])
            ax_sub.yaxis.grid(False)
            addVerticalLineContactSwitch(cs, ax_sub)


def plotCOMError(cs_ref, cs, dt):
    labels = ["x (m)", "y (m)", "z (m)"]
    colors = ['r', 'g', 'b']
    fig, ax = plt.subplots(3, 1)
    fig.canvas.set_window_title("COM tracking error")
    fig.suptitle("COM tracking error", fontsize=20)
    traj = cs.concatenateCtrajectories()
    traj_ref = cs_ref.concatenateCtrajectories()
    pos, timeline = discretizeCurve(traj, dt)
    pos_ref = discretizeCurve(traj_ref, dt)[0]
    error = pos - pos_ref
    for i in range(3):  # line = x,y,z
        ax_sub = ax[i]
        ax_sub.plot(timeline.T, error[i, :].T, color=colors[i])
        ax_sub.set_xlabel('time (s)')
        ax_sub.set_ylabel(labels[i])
        ax_sub.yaxis.grid()
        addVerticalLineContactSwitch(cs, ax_sub)


def plotAMError(cs_ref, cs, dt):
    labels = ["x", "y", "z"]
    colors = ['r', 'g', 'b']
    fig, ax = plt.subplots(3, 1)
    fig.canvas.set_window_title("AM tracking error")
    fig.suptitle("AM tracking error", fontsize=20)
    traj = cs.concatenateLtrajectories()
    traj_ref = cs_ref.concatenateLtrajectories()
    pos, timeline = discretizeCurve(traj, dt)
    pos_ref = discretizeCurve(traj_ref, dt)[0]
    error = pos - pos_ref
    for i in range(3):  # line = x,y,z
        ax_sub = ax[i]
        ax_sub.plot(timeline.T, error[i, :].T, color=colors[i])
        ax_sub.set_xlabel('time (s)')
        ax_sub.set_ylabel(labels[i])
        ax_sub.yaxis.grid()
        addVerticalLineContactSwitch(cs, ax_sub)


def plotZMP(cs_ref, cs, dt):
    fig = plt.figure("ZMP-CoM (xy)")
    ax = fig.gca()
    plt.suptitle("ZMP-CoM (xy) red = CoM ; black = ZMP (dashed = reference); green = feet placements")
    ZMP_t = discretizeCurve(cs.concatenateZMPtrajectories(), dt)[0]
    ZMP_ref = discretizeCurve(cs_ref.concatenateZMPtrajectories(), dt)[0]
    pcom_t = discretizeCurve(cs.concatenateCtrajectories(), dt)[0]
    plt.plot(ZMP_t[0, :].T, ZMP_t[1, :].T, color='k')
    plt.plot(ZMP_ref[0, :].T, ZMP_ref[1, :].T, color='k', linestyle=':')
    plt.plot(pcom_t[0, :].T, pcom_t[1, :].T, color='r')
    plt.xlabel("x position (m)")
    plt.ylabel("y position (m)")
    plt.axis('equal')
    plt.grid(True)
    colors_circles = ['g', 'r', 'b', 'y']
    effector_list = cs.getAllEffectorsInContact()
    for p in cs.contactPhases:
        # plot x for the center of the feets contact,
        # and a circle of 1cm of radius around it (size of the flexibility) :
        for eeName in p.effectorsInContact():
            pos = p.contactPatch(eeName).placement.translation
            color = colors_circles[effector_list.index(eeName)]
            plt.plot(pos[0], pos[1], marker="x", markersize=20, color=color)
            circle_r = plt.Circle((pos[0], pos[1]), 0.01, color='g', fill=False)
            ax.add_artist(circle_r)



def plotCOMTrajWithReferences(cs_ref, cs, dt):
    labels = [
        "x (m)", "y (m)", "z (m)", "dx (m/s)", "dy (m/s)", "dz (m/s)", "ddx (m/s^2)", "ddy (m/s^2)", "ddz (m/s^2)"
    ]
    colors = ['r', 'g', 'b']
    fig, ax = plt.subplots(3, 3)
    fig.canvas.set_window_title("COM trajectory (dashed = reference)")
    fig.suptitle("COM trajectory (dashed = reference)", fontsize=20)
    c_t, timeline = discretizeCurve(cs.concatenateCtrajectories(), dt)
    dc_t = discretizeCurve(cs.concatenateDCtrajectories(), dt)[0]
    ddc_t = discretizeCurve(cs.concatenateDDCtrajectories(), dt)[0]
    c_ref = discretizeCurve(cs_ref.concatenateCtrajectories(), dt)[0]
    dc_ref = discretizeCurve(cs_ref.concatenateDCtrajectories(), dt)[0]
    ddc_ref = discretizeCurve(cs_ref.concatenateDDCtrajectories(), dt)[0]
    values = np.vstack([c_t, dc_t, ddc_t])
    values_ref = np.vstack([c_ref, dc_ref, ddc_ref])
    for i in range(3):  # line = pos,vel,acc
        for j in range(3):  # col = x,y,z
            ax_sub = ax[i, j]
            ax_sub.plot(timeline.T, values[i * 3 + j, :].T, color=colors[j])
            ax_sub.plot(timeline.T, values_ref[i * 3 + j, :].T, color=colors[j], linestyle=":")
            ax_sub.set_xlabel('time (s)')
            ax_sub.set_ylabel(labels[i * 3 + j])
            ax_sub.yaxis.grid()
            addVerticalLineContactSwitch(cs, ax_sub)


def plotCOMTraj(cs, dt):
    labels = [
        "x (m)", "y (m)", "z (m)", "dx (m/s)", "dy (m/s)", "dz (m/s)", "ddx (m/s^2)", "ddy (m/s^2)", "ddz (m/s^2)"
    ]
    colors = ['r', 'g', 'b']
    fig, ax = plt.subplots(3, 3)
    fig.canvas.set_window_title("COM trajectory")
    fig.suptitle("COM trajectory", fontsize=20)
    c_t, timeline = discretizeCurve(cs.concatenateCtrajectories(), dt)
    dc_t = discretizeCurve(cs.concatenateDCtrajectories(), dt)[0]
    ddc_t = discretizeCurve(cs.concatenateDDCtrajectories(), dt)[0]
    values = np.vstack([c_t, dc_t, ddc_t])

    for i in range(3):  # line = pos,vel,acc
        for j in range(3):  # col = x,y,z
            ax_sub = ax[i, j]
            ax_sub.plot(timeline.T, values[i * 3 + j, :].T, color=colors[j])
            ax_sub.set_xlabel('time (s)')
            ax_sub.set_ylabel(labels[i * 3 + j])
            ax_sub.yaxis.grid()
            addVerticalLineContactSwitch(cs, ax_sub)


def plotAMTrajWithReferences(cs_ref, cs, dt):
    labels = ["x", "y", "z", "dx", "dy", "dz"]
    colors = ['r', 'g', 'b']
    fig, ax = plt.subplots(2, 3)
    fig.canvas.set_window_title("AM trajectory (dashed = reference)")
    fig.suptitle("AM trajectory (dashed = reference)", fontsize=20)
    L_t, timeline = discretizeCurve(cs.concatenateLtrajectories(), dt)
    dL_t = discretizeCurve(cs.concatenateDLtrajectories(), dt)[0]
    L_ref = discretizeCurve(cs_ref.concatenateLtrajectories(), dt)[0]
    dL_ref = discretizeCurve(cs_ref.concatenateDLtrajectories(), dt)[0]
    values = np.vstack([L_t, dL_t])
    values_ref = np.vstack([L_ref, dL_ref])

    for i in range(2):  # line = L,dL
        for j in range(3):  # col = x,y,z
            ax_sub = ax[i, j]
            ax_sub.plot(timeline.T, values[i * 3 + j, :].T, color=colors[j])
            ax_sub.plot(timeline.T, values_ref[i * 3 + j, :].T, color=colors[j], linestyle=":")
            ax_sub.set_xlabel('time (s)')
            ax_sub.set_ylabel(labels[i * 3 + j])
            ax_sub.yaxis.grid()
            addVerticalLineContactSwitch(cs, ax_sub)


def plotAMTraj(cs, dt):
    labels = ["x", "y", "z", "dx", "dy", "dz"]
    colors = ['r', 'g', 'b']
    fig, ax = plt.subplots(2, 3)
    fig.canvas.set_window_title("AM trajectory")
    fig.suptitle("AM trajectory", fontsize=20)
    L_t, timeline = discretizeCurve(cs.concatenateLtrajectories(), dt)
    dL_t = discretizeCurve(cs.concatenateDLtrajectories(), dt)[0]
    values = np.vstack([L_t, dL_t])

    for i in range(2):  # line = L,dL
        for j in range(3):  # col = x,y,z
            ax_sub = ax[i, j]
            ax_sub.plot(timeline.T, values[i * 3 + j, :].T, color=colors[j])
            ax_sub.set_xlabel('time (s)')
            ax_sub.set_ylabel(labels[i * 3 + j])
            ax_sub.yaxis.grid()
            addVerticalLineContactSwitch(cs, ax_sub)


def plotContactForces(cs, dt):
    colors = ['r', 'g', 'b', 'y']
    fig = plt.figure("Contact normal force")
    plt.suptitle("Contact normal force")
    ax = fig.gca()
    ax.set_xlabel('time (s)')
    ax.set_ylabel("Contact normal force (N)")
    ax.yaxis.grid()
    sum_f = None
    for i, eeName in enumerate(cs.getAllEffectorsInContact()):
        force, timeline = discretizeCurve(cs.concatenateNormalForceTrajectories(eeName), dt)
        ax.plot(timeline.T, force.T, color=colors[i], label=eeName)
        if sum_f is None:
            sum_f = force
        else:
            sum_f += force
    ax.plot(timeline.T, sum_f.T, color="k", label="sum")
    addVerticalLineContactSwitch(cs, ax)
    ax.legend()


def plotKneeTorque(cs, dt, kneeIds, offset):
    colors = ['r', 'g', 'b', 'y']
    fig = plt.figure("Knee torque")
    plt.suptitle("Knee torque")
    ax = fig.gca()
    ax.set_xlabel('time (s)')
    ax.set_ylabel("Torque")
    ax.yaxis.grid()
    tau, timeline = discretizeCurve(cs.concatenateTauTrajectories(), dt)
    for k, name in enumerate(kneeIds.keys()):
        ax.plot(timeline.T, tau[kneeIds[name] - offset, :].T, color=colors[k], label=name)
        # - 1 because it's the index in the configuration space, and here we are in the velocity space
    addVerticalLineContactSwitch(cs, ax)
    ax.legend()


def saveAllFigures(path_dir):
    if not os.path.exists(path_dir):
        os.makedirs(path_dir)
    for i in plt.get_fignums():
        fig = plt.figure(i)
        fig.savefig(path_dir + "/" + str(fig._suptitle.get_text()) + ".eps", dpi=600)


def plotALLFromWB(cs_ref, cs ,cfg):
    print("Plotting ...")
    plt.rcParams['axes.linewidth'] = plt.rcParams['font.size'] / 30.
    plt.rcParams['lines.linewidth'] = plt.rcParams['font.size'] / 30.
    display = cfg.DISPLAY_PLOT
    save = cfg.SAVE_PLOT
    path = cfg.OUTPUT_DIR + "/plot/" + cfg.DEMO_NAME
    dt = cfg.IK_dt
    if cs_ref.haveCOMtrajectories():
        if cs.haveCOMtrajectories():
            plotCOMTrajWithReferences(cs_ref, cs, dt)
            plotCOMError(cs_ref, cs, dt)
        elif not cfg.PLOT_CENTROIDAL:
            plotCOMTraj(cs_ref, dt)
        # else : it's already plotted
    if cs_ref.haveAMtrajectories():
        if cs.haveAMtrajectories():
            plotAMTrajWithReferences(cs_ref, cs, dt)
            plotAMError(cs_ref, cs, dt)
        elif not cfg.PLOT_CENTROIDAL:
            plotAMTraj(cs_ref, dt)
        # else : it's already plotted
    if cs.haveZMPtrajectories() and Requirements.requireZMPtrajectories(cs_ref, cfg):
        plotZMP(cs_ref, cs, dt)
    if cs.haveTorquesTrajectories():
        offset = cs.contactPhases[0].q_t.dim() - cs.contactPhases[0].tau_t.dim()
        plotKneeTorque(cs, dt, cfg.Robot.kneeIds, offset)
    if cs.haveContactForcesTrajectories():
        plotContactForces(cs, dt)
    if cs.haveEffectorsTrajectories(1e-1):
        plotEffectorTrajectoryWithReference(cs_ref, cs, dt)
        plotEffectorError(cs_ref, cs, dt)
    else:
        plotEffectorTrajectory(cs, dt, "Reference")

    if display:
        plt.show(block=False)
    if save and path:
        saveAllFigures(path)
    print("Plotting Done.")
