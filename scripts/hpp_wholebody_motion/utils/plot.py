
import numpy as np
import pinocchio as se3
from pinocchio import SE3
from hpp_wholebody_motion.utils.util import *
import hpp_wholebody_motion.config as cfg
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from multiprocessing import Process

plt.ion()


def show(plt):
    plt.show()

def show_non_blocking(plt):
    p = Process(target=show,args= ([plt]))
    p.start()   



def plotEffectorRef(dict_refs):
    labels=["x (m)" , "y (m)" ,"z (m)", "dx (m/s)" , "dy (m/s)" ,"dz (m/s)","ddx (m/s^2)" , "ddy (m/s^2)" ,"ddz (m/s^2)"]
    colors = ['r','g','b']    
    dt = cfg.IK_dt
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
    plt.draw()
    #show_non_blocking(plt)
    plt.show()
    

def plotEffectorError():
    return
    
    
def plotZMP(cs,ZMP_t,pcom_t):
    fig = plt.figure("ZMP-CoM (xy)")
    ax=fig.gca()
    plt.title("red = CoM ; black = ZMP ; green = feet placements")    
    plt.plot(ZMP_t[0,:].T,ZMP_t[1,:].T,color='k')
    plt.plot(pcom_t[0,:].T,pcom_t[1,:].T,color='r')
    plt.xlabel("x position (m)")
    plt.ylabel("y position (m)")
    plt.axis('equal')
  
    for p in cs.contact_phases:
        # plot x for the center of the feets contact, 
        # and a circle of 1cm of radius around it (size of the flexibility) :
        pos = JointPlacementForEffector(p,cfg.Robot.rfoot).translation
        plt.plot(pos[0], pos[1], marker="x", markersize=20, color='g')
        circle_r = plt.Circle((pos[0], pos[1]), 0.01, color='g', fill=False)      
        pos = JointPlacementForEffector(p,cfg.Robot.lfoot).translation
        plt.plot(pos[0], pos[1], marker="x", markersize=20, color='g')
        circle_l = plt.Circle((pos[0], pos[1]), 0.01, color='g', fill=False)      
        ax.add_artist(circle_r)
        ax.add_artist(circle_l)
      
    plt.draw()
    plt.show()  
