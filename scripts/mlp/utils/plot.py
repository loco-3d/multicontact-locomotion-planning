
import numpy as np
import os
import pinocchio as pin
from pinocchio import SE3
from mlp.utils.util import *
import matplotlib
matplotlib.use("Qt4agg")
import matplotlib.pyplot as plt
from multiprocessing import Process
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
        plt.axvline(timeline[interval[-1]],linestyle=linestyle,color=color)

def plotEffectorRef(dict_refs,dt):
    labels=["x (m)" , "y (m)" ,"z (m)", "dx (m/s)" , "dy (m/s)" ,"dz (m/s)","ddx (m/s^2)" , "ddy (m/s^2)" ,"ddz (m/s^2)"]
    colors = ['r','g','b']    
    # look for the maximal time in the trajectories :
    t_max = 0.
    for trajs in dict_refs.itervalues():
        if trajs[-1].time_interval[-1] > t_max:
            t_max = trajs[-1].time_interval[-1]
    N = int(t_max/dt) +1
    timeline =  np.matrix([i*dt for i in range(N)])
    #print "plot : tmax = ",t_max
    #print " N = ",N
    #print "last timeline = ",timeline[-1]
    #print "len timeline : ",timeline.shape
    # One plot for each effector present in the dict :
    for eeName,trajs in dict_refs.iteritems():
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
            if t >= traj.time_interval[0] and t <= traj.time_interval[-1]:
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
    #show_non_blocking(plt)
    

def plotEffectorError(timeline,p_intervals,eff_error_dict):
    labels=["x (m)" , "y (m)" ,"z (m)"]
    colors = ['r','g','b']    
    for eeName,error in eff_error_dict.iteritems():
        fig, ax = plt.subplots(3,1)
        fig.canvas.set_window_title("Effector tracking error : "+eeName)
        fig.suptitle("Effector tracking error : "+eeName, fontsize=20)    
        for i in range(3): # line = x,y,z
                ax_sub = ax[i]
                ax_sub.plot(timeline.T, error[i,:].T, color=colors[i])
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
            plt.plot(pos[0], pos[1], marker="x", markersize=20, color='g')
            circle_l = plt.Circle((pos[0], pos[1]), 0.01, color='g', fill=False)      
            ax.add_artist(circle_l)
      
def plotEffectorTraj(timeline,p_intervals,ref_dict,traj_dict):
    labels=["x (m)" , "y (m)" ,"z (m)"]
    colors = ['r','g','b']    
    for eeName,traj in traj_dict.iteritems():
        fig, ax = plt.subplots(3,1)
        fig.canvas.set_window_title("Effector trajectory (dashed = reference) : "+eeName)
        fig.suptitle("Effector trajectory (dashed = reference) : "+eeName, fontsize=20)    
        for i in range(3): # line = x,y,z
                ax_sub = ax[i]
                ax_sub.plot(timeline.T, traj[i,:].T, color=colors[i])
                ax_sub.plot(timeline.T, ref_dict[eeName][i,:].T, color=colors[i],linestyle=":")                
                ax_sub.set_xlabel('time (s)')
                ax_sub.set_ylabel(labels[i])
                ax_sub.yaxis.grid()    
                addVerticalLineContactSwitch(timeline.T,p_intervals,ax_sub)

    
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
            
    return

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
    
    for eeName,force in forces_dict.iteritems():    
        ax.plot(timeline.T, force[0,:].T, color=colors[i],label = eeName)
        sum_f += force[0,:]
        i += 1
    sum_f /= float(i)
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
    print "Plotting ..."
    plt.rcParams['axes.linewidth'] = plt.rcParams['font.size'] / 30.
    plt.rcParams['lines.linewidth'] = plt.rcParams['font.size'] / 30.    
    if res.effector_trajectories.values()[0].any():
        plotEffectorTraj(res.t_t,res.phases_intervals,res.effector_references,res.effector_trajectories)
    if res.c_t.any():
        plotCOMTraj(res.t_t,res.phases_intervals,res.c_reference,res.dc_reference,res.ddc_reference,res.c_t,res.dc_t,res.ddc_t)
    if res.c_tracking_error.any() : 
        plotCOMError(res.t_t,res.phases_intervals,res.c_tracking_error)
        plotEffectorError(res.t_t,res.phases_intervals,res.effector_tracking_error)        
    plotContactForces(res.t_t,res.phases_intervals,res.contact_normal_force,res.N)
    # compute zmp from whole body or centroidal (only if it hasn't been computed already)
    plotZMP(cs,res.zmp_t,res.zmp_reference,res.c_t)
    if display:
        plt.show(block = False)
    if save and path:
        saveAllFigures(path)
    print "Plotting Done."