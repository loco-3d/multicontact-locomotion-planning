import hpp_wholebody_motion.config as cfg
import pinocchio as se3
from pinocchio import SE3, Quaternion
import locomote
from locomote import WrenchCone,SOC6,ContactPatch, ContactPhaseHumanoid, ContactSequenceHumanoid
import numpy as np
import time

STONE_HEIGHT = 0.005
STONE_GROUP = "stepping_stones"        
TRAJ_GROUP = "com_traj"

def displaySphere(viewer,pos,size=0.01,color=[0,0,0,1]):
  rootName = "s"
  # add indices until the name is free
  list = viewer.client.gui.getNodeList()
  i=0
  name = rootName
  while list.count(name) > 0:
      name=rootName+"_"+str(i)
      i+=1     
  viewer.client.gui.addSphere(name,size,color)
  viewer.client.gui.addToGroup(name,viewer.sceneName)
  #viewer.client.gui.setVisibility(name,'ALWAYS_ON_TOP')
  q=pos+[0,0,0,1]
  viewer.client.gui.applyConfiguration(name,q)
  viewer.client.gui.refresh()    

def SE3ToViewerConfig(placement):
    q = [0]*7
    q[0:3] = placement.translation.T.tolist()[0]
    r = Quaternion(placement.rotation)
    q[6]=  r.w
    q[3:6] =  r.coeffs().transpose().tolist()[0][0:3]    
    return q

def addContactLandmark(M,color,v):
    global i_sphere
    gui = v.client.gui
    name = 's'+str(i_sphere)
    i_sphere += 1    
    gui.addSphere(name,0.01,color)
    #gui.setVisibility(name,"ALWAYS_ON_TOP")
    gui.addToGroup(name,viewer.sceneName)
    gui.applyConfiguration(name,SE3ToViewerConfig(M))
    gui.addLandmark(name,0.03) 
    #print "contact altitude : "+str(p[2])
    
def displayContactsLandmarkFromPhase(phase,viewer):
    if phase.LF_patch.active:
        addContactLandmark(phase.LF_patch.placement*cfg.Robot.MLsole_display,cfg.Robot.dict_limb_color_traj[cfg.Robot.lfoot] ,viewer)
    if phase.RF_patch.active:
        addContactLandmark(phase.RF_patch.placement*cfg.Robot.MRsole_display,cfg.Robot.dict_limb_color_traj[cfg.Robot.rfoot] ,viewer)  
    if phase.LH_patch.active:
        addContactLandmark(phase.LH_patch.placement*cfg.Robot.MLhand_display,cfg.Robot.dict_limb_color_traj[cfg.Robot.lhand] ,viewer)
    if phase.RH_patch.active:
        addContactLandmark(phase.RH_patch.placement*cfg.Robot.MRhand_display,cfg.Robot.dict_limb_color_traj[cfg.Robot.rhand] ,viewer)                 
    viewer.client.gui.refresh() 

def addSteppingStone(viewer,placement,name,size,color):
    viewer.client.gui.addBox(name,size[0]/2.,size[1]/2.,STONE_HEIGHT,color)
    viewer.client.gui.addToGroup(name,STONE_GROUP)
    viewer.client.gui.applyConfiguration(name,SE3ToViewerConfig(placement))
    
    
def displaySteppingStones(cs,viewer):
    viewer.client.gui.createGroup(STONE_GROUP)
    name_RF = STONE_GROUP+"/stone_RF_"
    name_LF = STONE_GROUP+"/stone_LF_"
    name_RH = STONE_GROUP+"/stone_RH_"
    name_LH = STONE_GROUP+"/stone_LH_"
    id_RF = 0
    id_LF = 0
    id_RH = 0
    id_LH = 0
    
    for phase in cs.contact_phases:
        if phase.LF_patch.active:
            addSteppingStone(viewer,phase.LF_patch.placement*cfg.Robot.dict_display_offset[cfg.Robot.lfoot],name_LF+str(id_LF),cfg.Robot.dict_size[cfg.Robot.lfoot],cfg.Robot.dict_limb_color_traj[cfg.Robot.lfoot])
            id_LF += 1
        if phase.RF_patch.active:
            addSteppingStone(viewer,phase.RF_patch.placement*cfg.Robot.dict_display_offset[cfg.Robot.rfoot],name_RF+str(id_RF),cfg.Robot.dict_size[cfg.Robot.rfoot],cfg.Robot.dict_limb_color_traj[cfg.Robot.rfoot])
            id_RF += 1            
        if phase.LH_patch.active:
            addSteppingStone(viewer,phase.LH_patch.placement*cfg.Robot.dict_display_offset[cfg.Robot.lhand],name_LH+str(id_LH),cfg.Robot.dict_size[cfg.Robot.lhand],cfg.Robot.dict_limb_color_traj[cfg.Robot.lhand])
            id_LH += 1
        if phase.RH_patch.active:
            addSteppingStone(viewer,phase.RH_patch.placement*cfg.Robot.dict_display_offset[cfg.Robot.rhand],name_RH+str(id_RH),cfg.Robot.dict_size[cfg.Robot.rhand],cfg.Robot.dict_limb_color_traj[cfg.Robot.rhand])
            id_RH += 1
    
    viewer.client.gui.addToGroup(STONE_GROUP,viewer.sceneName)
    viewer.client.gui.refresh()

def stdVecToMatrix(std_vector):
    if len(std_vector) == 0:
        raise Exception("std_vector is Empty")
    vec_l = []
    for vec in std_vector:
        vec_l.append(vec)

    res = np.hstack(tuple(vec_l))
    return res

def comPosListFromState(state_traj):
    state = stdVecToMatrix(state_traj)
    c =state[:3,:]
    c_l = []
    for i in range(c.shape[1]):
        c_l += [c[:,i].T.tolist()[0]]
    return c_l

def displayCOMTrajForPhase(p,viewer,name,name_group,color):
    c = comPosListFromState(p.state_trajectory)
    viewer.client.gui.addCurve(name,c,color)
    viewer.client.gui.addToGroup(name,name_group)
    
    

def displayCOMTrajectory(cs,viewer,colors=[0,0,0,1],nameGroup=""):
    name_group = TRAJ_GROUP+nameGroup
    viewer.client.gui.createGroup(name_group)
    for pid in range(len(cs.contact_phases)):
        phase = cs.contact_phases[pid]
        if pid < len(cs.contact_phases)-1 :
            phase_next = cs.contact_phases[pid+1]
        else :
            phase_next = None
        name = name_group+"/"+'%.2f' % phase.time_trajectory[0]+"-"+'%.2f' % phase.time_trajectory[-1]
        color = colors[pid%len(colors)]
        displayCOMTrajForPhase(phase,viewer,name,name_group,color)
    viewer.client.gui.addToGroup(name_group,viewer.sceneName)
    viewer.client.gui.refresh()        
    
def displaySE3Traj(traj,viewer,name,color,time_interval,offset=SE3.Identity()):
    if name==None:
        name="SE3_traj"
    rootName = name
    # add indices until the name is free
    list = viewer.client.gui.getNodeList()
    i=0
    while list.count(name) > 0:
        name=rootName+"_"+str(i)
        i+=1    
    path = []    
    dt = 0.01
    t = time_interval[0]
    while t <= time_interval[1]:
        m = traj(t)[0]
        m = m.act(offset)
        path += m.translation.T.tolist()
        t += dt
    viewer.client.gui.addCurve(name,path,color)
    viewer.client.gui.addToGroup(name,viewer.sceneName)    
    viewer.client.gui.refresh
    
def displayWBconfig(viewer,q_matrix):
  q = q_matrix.T.tolist()[0]
  extraDof = viewer.robot.getConfigSize() - q_matrix.shape[0]
  assert extraDof >= 0 , "Robot model used by the IK is not the same as during the planning"
  if extraDof > 0:
    q += [0]*extraDof
  viewer(q)  
    
def displayWBmotion(viewer,q_t,dt,dt_display):
    id = 0
    step = dt_display / dt 
    assert step%1 == 0 ,"display dt shouldbe a multiple of ik dt"
    # check if robot have extradof : 
    step = int(step)
    while id < q_t.shape[1]:
        t_start = time.time()
        displayWBconfig(viewer,q_t[:,id])
        id += step
        elapsed = time.time() - t_start
        if elapsed > dt_display :
            print "Warning : display not real time ! choose a greater time step for the display."
        else : 
            time.sleep(dt_display - elapsed)
    # display last config if the total duration is not a multiple of the dt
    displayWBconfig(viewer,q_t[:,-1])
          

