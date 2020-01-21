
import numpy as np
import os
import pinocchio as pin
from pinocchio import SE3
from mlp.utils.util import *
import matplotlib
matplotlib.use("Qt4agg")
import matplotlib.pyplot as plt
from multiprocessing import Process
from mlp.utils.util import stdVecToMatrix
from mlp.utils.computation_tools import computeZMP,computeZMPRef
plt.ioff()


def show(plt):
    plt.show()

def show_non_blocking(plt):
    p = Process(target=show,args= ([plt]))
    p.start()   

def addVerticalLineContactSwitch(timeline,intervals,plt,linestyle="-.",color='k'):
    plt.axvline(timeline[intervals[0][0]],linestyle=linestyle,color=color)    
    for interval in intervals:
        if len(interval) > 0 and (interval[-1] < len(timeline)): # may happen when the motion generation did not succeed completely
            plt.axvline(timeline[interval[-1]],linestyle=linestyle,color=color)

def plotEffectorRef(dict_refs,dt):
    labels=["x (m)" , "y (m)" ,"z (m)", "dx (m/s)" , "dy (m/s)" ,"dz (m/s)","ddx (m/s^2)" , "ddy (m/s^2)" ,"ddz (m/s^2)"]
    colors = ['r','g','b']    
    # look for the maximal time in the trajectories :
    t_max = 0.
    for trajs in dict_refs.values():
        if len(trajs) > 0:
            if trajs[-1].time_interval[-1] > t_max:
                t_max = trajs[-1].time_interval[-1]
    N = int(t_max/dt) +1
    timeline =  np.matrix([i*dt for i in range(N)])
    #print "plot : tmax = ",t_max
    #print " N = ",N
    #print "last timeline = ",timeline[-1]
    #print "len timeline : ",timeline.shape
    # One plot for each effector present in the dict :
    for eeName,trajs in dict_refs.items():
        if len(trajs) > 0 :
            # first build matrices of position, velocity and acceleration (each is 3D)  :
            values = np.matrix(np.zeros([9,N]))
            id_traj = 0
            traj = trajs[id_traj]
            # save current pos (or next) and use it when timing not inside bounds
            pos =  traj.placement_init.translation
            for i,t in np.ndenumerate(timeline) : 
                i=i[1]
                #print "t =",t
                if t > traj.time_interval[-1] and id_traj < (len(trajs)-1):  # take next traj in list
                    pos =  traj.placement_end.translation
                    id_traj += 1
                    traj = trajs[id_traj]
                    #print "new traj, t0 = ",traj.time_interval[0]
                if t >= traj.time_interval[0] and t < traj.time_interval[-1]:
                    values[0:3,i] = traj.curves(t-traj.time_interval[0])
                    values[3:6,i] = traj.curves.d(t-traj.time_interval[0])
                    values[6:9,i] = traj.curves.dd(t-traj.time_interval[0])
                    #print "add column ",i
                    #print " values : ",values[:,i]
                else :
                    values[0:3,i] =pos
            #print "values shape : ",values.shape
            # then plot each line of values in a graph  :
            fig, ax = plt.subplots(3,3)
            fig.canvas.set_window_title("Reference trajectory effector "+eeName)
            fig.suptitle("Reference trajectory effector "+eeName, fontsize=20)
            for i in range(3): # line = pos,vel,acc
                for j in range(3): # col = x,y,z
                    ax_sub = ax[i,j]
                    ax_sub.plot(timeline.T, values[i*3 + j,:].T, color=colors[j])
                    ax_sub.set_xlabel('time (s)')
                    ax_sub.set_ylabel(labels[i*3 + j])
                    ax_sub.grid(True)
    plt.show(block = False)
    
def plotEffectorTraj(timeline,p_intervals,ref_dicts,traj_dicts):
    labels=["x (m)" , "y (m)" ,"z (m)", "dx (m/s)" , "dy (m/s)" ,"dz (m/s)","ddx (m/s^2)" , "ddy (m/s^2)" ,"ddz (m/s^2)"]
    colors = ['r','g','b']
    for eeName in traj_dicts[0].keys():
        fig, ax = plt.subplots(3,3)
        fig.canvas.set_window_title("Effector trajectory (dashed = reference) : "+eeName)
        fig.suptitle("Effector trajectory (dashed = reference) : "+eeName, fontsize=20)    
        for i in range(3): # line = pos,vel,acc
                for j in range(3): # col = x,y,z
                    ax_sub = ax[i,j]
                    ax_sub.plot(timeline.T, traj_dicts[i][eeName][j,:].T, color=colors[j])
                    ax_sub.plot(timeline.T, ref_dicts[i][eeName][j,:].T, color=colors[j],linestyle=":")
                    ax_sub.set_xlabel('time (s)')
                    ax_sub.set_ylabel(labels[i*3+j])
                    ax_sub.yaxis.grid()
                    addVerticalLineContactSwitch(timeline.T,p_intervals,ax_sub)

    

def plotEffectorError(timeline,p_intervals,ref_dict,traj_dict):
    labels=["x (m)" , "y (m)" ,"z (m)"]
    colors = ['r','g','b']    
    for eeName,traj in traj_dict.items():
        ref = ref_dict[eeName]
        fig, ax = plt.subplots(3,1)
        fig.canvas.set_window_title("Effector tracking error : "+eeName)
        fig.suptitle("Effector tracking error : "+eeName, fontsize=20)    
        for i in range(3): # line = x,y,z
                ax_sub = ax[i]
                ax_sub.plot(timeline.T, traj[i,:].T - ref[i,:].T, color=colors[i])
                ax_sub.set_xlabel('time (s)')
                ax_sub.set_ylabel(labels[i])
                ax_sub.yaxis.grid()    
                addVerticalLineContactSwitch(timeline.T,p_intervals,ax_sub)
    
    
def plotCOMError(timeline,p_intervals,error):
    labels=["x (m)" , "y (m)" ,"z (m)"]
    colors = ['r','g','b']
    fig, ax = plt.subplots(3,1)
    fig.canvas.set_window_title("COM tracking error")
    fig.suptitle("COM tracking error", fontsize=20)    
    for i in range(3): # line = x,y,z
            ax_sub = ax[i]
            ax_sub.plot(timeline.T, error[i,:].T, color=colors[i])
            ax_sub.set_xlabel('time (s)')
            ax_sub.set_ylabel(labels[i])
            ax_sub.yaxis.grid()    
            addVerticalLineContactSwitch(timeline.T,p_intervals,ax_sub)

def plotAMError(timeline,p_intervals,error):
    labels=["x" , "y" ,"z"]
    colors = ['r','g','b']
    fig, ax = plt.subplots(3,1)
    fig.canvas.set_window_title("AM tracking error")
    fig.suptitle("AM tracking error", fontsize=20)    
    for i in range(3): # line = x,y,z
            ax_sub = ax[i]
            ax_sub.plot(timeline.T, error[i,:].T, color=colors[i])
            ax_sub.set_xlabel('time (s)')
            ax_sub.set_ylabel(labels[i])
            ax_sub.yaxis.grid()    
            addVerticalLineContactSwitch(timeline.T,p_intervals,ax_sub)


def plotZMP(cs,ZMP_t,ZMP_ref,pcom_t):
    fig = plt.figure("ZMP-CoM (xy)")
    ax=fig.gca()
    plt.suptitle("ZMP-CoM (xy) red = CoM ; black = ZMP (dashed = reference); green = feet placements")    
    plt.plot(ZMP_t[0,:].T,ZMP_t[1,:].T,color='k')
    plt.plot(ZMP_ref[0,:].T,ZMP_ref[1,:].T,color='k',linestyle=':')    
    plt.plot(pcom_t[0,:].T,pcom_t[1,:].T,color='r')
    plt.xlabel("x position (m)")
    plt.ylabel("y position (m)")
    plt.axis('equal')
    plt.grid(True)
    for p in cs.contact_phases:
        # plot x for the center of the feets contact, 
        # and a circle of 1cm of radius around it (size of the flexibility) :
        if p.RF_patch.active:
            pos = p.RF_patch.placement.translation
            plt.plot(pos[0], pos[1], marker="x", markersize=20, color='g')
            circle_r = plt.Circle((pos[0], pos[1]), 0.01, color='g', fill=False)
            ax.add_artist(circle_r)     
        if p.LF_patch.active:   
            pos = p.LF_patch.placement.translation
            plt.plot(pos[0], pos[1], marker="x", markersize=20, color='r')
            circle = plt.Circle((pos[0], pos[1]), 0.01, color='r', fill=False)      
            ax.add_artist(circle)
        if p.RH_patch.active:   
            pos = p.RH_patch.placement.translation
            plt.plot(pos[0], pos[1], marker="x", markersize=20, color='b')
            circle = plt.Circle((pos[0], pos[1]), 0.01, color='b', fill=False)      
            ax.add_artist(circle)   
        if p.LH_patch.active:   
            pos = p.LH_patch.placement.translation
            plt.plot(pos[0], pos[1], marker="x", markersize=20, color='y')
            circle = plt.Circle((pos[0], pos[1]), 0.01, color='y', fill=False)      
            ax.add_artist(circle)                    
      
def plotCOMTraj(timeline,p_intervals,ref_c,ref_dc,ref_ddc,c_t,dc_t,ddc_t):
    labels=["x (m)" , "y (m)" ,"z (m)", "dx (m/s)" , "dy (m/s)" ,"dz (m/s)","ddx (m/s^2)" , "ddy (m/s^2)" ,"ddz (m/s^2)"]
    colors = ['r','g','b']    
    fig, ax = plt.subplots(3,3)
    fig.canvas.set_window_title("COM trajectory (dashed = reference)")
    fig.suptitle("COM trajectory (dashed = reference)", fontsize=20)
    for i in range(3): # line = pos,vel,acc
        for j in range(3): # col = x,y,z
            ax_sub = ax[i,j]
            if i == 0 :
                ax_sub.plot(timeline.T, c_t[j,:].T, color=colors[j])
                ax_sub.plot(timeline.T, ref_c[j,:].T, color=colors[j],linestyle=":")
            elif i == 1 :
                ax_sub.plot(timeline.T, dc_t[j,:].T, color=colors[j])
                ax_sub.plot(timeline.T, ref_dc[j,:].T, color=colors[j],linestyle=":")                
            elif i == 2 :
                ax_sub.plot(timeline.T, ddc_t[j,:].T, color=colors[j])
                ax_sub.plot(timeline.T, ref_ddc[j,:].T, color=colors[j],linestyle=":")                                
            ax_sub.set_xlabel('time (s)')
            ax_sub.set_ylabel(labels[i*3 + j])
            ax_sub.yaxis.grid()    
            addVerticalLineContactSwitch(timeline.T,p_intervals,ax_sub)

def plotCOMTrajFromCS(cs):
    labels=["x (m)" , "y (m)" ,"z (m)", "dx (m/s)" , "dy (m/s)" ,"dz (m/s)","ddx (m/s^2)" , "ddy (m/s^2)" ,"ddz (m/s^2)"]
    colors = ['r','g','b']
    fig, ax = plt.subplots(3,3)
    fig.canvas.set_window_title("COM trajectory reference")
    fig.suptitle("COM trajectory reference", fontsize=20)
    for p in cs.contact_phases:
        states = stdVecToMatrix(p.state_trajectory)
        controls = stdVecToMatrix(p.control_trajectory)
        timeline = stdVecToMatrix(p.time_trajectory)
        for i in range(3): # line = pos,vel,acc
            for j in range(3): # col = x,y,z
                ax_sub = ax[i,j]
                if i == 0 :
                    ax_sub.plot(timeline.T, states[j,:].T, color=colors[j])
                elif i == 1 :
                    ax_sub.plot(timeline.T, states[j+3,:].T, color=colors[j])
                elif i == 2 :
                    ax_sub.plot(timeline.T, controls[j,:].T, color=colors[j])
                ax_sub.axvline(p.time_trajectory[-1], linestyle="-.", color='k')

    for i in range(3):  # line = pos,vel,acc
        for j in range(3):  # col = x,y,z
            ax_sub = ax[i, j]
            ax_sub.set_xlabel('time (s)')
            ax_sub.set_ylabel(labels[i*3 + j])
            ax_sub.yaxis.grid()
    plt.show(block=False)

def plotAMTraj(timeline,p_intervals,L_t,dL_t,L_reference,dL_reference):
    labels=["x" , "y" ,"z", "dx" , "dy" ,"dz"]
    colors = ['r','g','b']    
    fig, ax = plt.subplots(2,3)
    fig.canvas.set_window_title("AM trajectory (dashed = reference)")
    fig.suptitle("AM trajectory (dashed = reference)", fontsize=20)
    for i in range(2): # line = L,dL
        for j in range(3): # col = x,y,z
            ax_sub = ax[i,j]
            if i == 0 :
                ax_sub.plot(timeline.T, L_t[j,:].T, color=colors[j])
                ax_sub.plot(timeline.T, L_reference[j,:].T, color=colors[j],linestyle=":")
            elif i == 1 :
                ax_sub.plot(timeline.T, dL_t[j,:].T, color=colors[j])
                ax_sub.plot(timeline.T, dL_reference[j,:].T, color=colors[j],linestyle=":")                                                
            ax_sub.set_xlabel('time (s)')
            ax_sub.set_ylabel(labels[i*3 + j])
            ax_sub.yaxis.grid()    
            addVerticalLineContactSwitch(timeline.T,p_intervals,ax_sub)

def plotContactForces(timeline,p_intervals,forces_dict,N):
    colors = ['r','g','b','y']  
    fig = plt.figure("Contact normal force")
    plt.suptitle("Contact normal force")
    ax=fig.gca() 
    ax.set_xlabel('time (s)')
    ax.set_ylabel("Contact normal force (N)")  
    ax.yaxis.grid() 
    addVerticalLineContactSwitch(timeline.T,p_intervals,ax)
    i = 0
    sum_f = np.matrix(np.zeros([1,N]))
    
    for eeName,force in forces_dict.items():    
        ax.plot(timeline.T, force[0,:].T, color=colors[i],label = eeName)
        sum_f += force[0,:]
        i += 1
    ax.plot(timeline.T, sum_f[0,:].T, color="k",label = "sum")
    ax.legend()

def plotKneeTorque(timeline,p_intervals,tau,offset,kneeIds):
    colors = ['r','g','b','y']  
    fig = plt.figure("Knee torque")
    plt.suptitle("Knee torque")
    ax=fig.gca() 
    ax.set_xlabel('time (s)')
    ax.set_ylabel("Torque")  
    ax.yaxis.grid()    
    addVerticalLineContactSwitch(timeline.T,p_intervals,ax)    
    for k,name in enumerate(kneeIds.keys()) : 
        ax.plot(timeline.T, tau[kneeIds[name]-offset,:].T, color=colors[k],label = name)
    ax.legend()
    
def saveAllFigures(path_dir): 
    if not os.path.exists(path_dir):
        os.makedirs(path_dir)
    for i in plt.get_fignums():
        fig = plt.figure(i)
        fig.savefig(path_dir+"/"+str(fig._suptitle.get_text())+".eps",dpi=600)

def plotALLFromWB(cs,res,display=True,save=False,path=None):
    print("Plotting ...")
    plt.rcParams['axes.linewidth'] = plt.rcParams['font.size'] / 30.
    plt.rcParams['lines.linewidth'] = plt.rcParams['font.size'] / 30.    
    if res.c_t.any():
        plotCOMTraj(res.t_t,res.phases_intervals,res.c_reference,res.dc_reference,res.ddc_reference,res.c_t,res.dc_t,res.ddc_t)
        plotCOMError(res.t_t,res.phases_intervals,res.c_t - res.c_reference)        
    if res.dL_t.any():
        plotAMTraj(res.t_t,res.phases_intervals,res.L_t,res.dL_t,res.L_reference,res.dL_reference) 
        plotAMError(res.t_t,res.phases_intervals,res.L_t - res.L_reference)                
    if list(res.effector_trajectories.values())[0].any():
        plotEffectorTraj(res.t_t,res.phases_intervals,[res.effector_references,res.d_effector_references,res.d_effector_references],[res.effector_trajectories,res.d_effector_trajectories,res.dd_effector_trajectories])
        plotEffectorError(res.t_t,res.phases_intervals,res.effector_references,res.effector_trajectories)        
    plotContactForces(res.t_t,res.phases_intervals,res.contact_normal_force,res.N)
    # compute zmp from whole body or centroidal (only if it hasn't been computed already)
    plotZMP(cs,res.zmp_t,res.zmp_reference,res.c_t)
    if display:
        plt.show(block = False)
    if save and path:
        saveAllFigures(path)
    print("Plotting Done.")