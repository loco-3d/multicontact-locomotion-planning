import numpy as np
import mlp.utils.wholebody_result as wb_res
import mlp.utils.plot as plot
import multicontact_api
from multicontact_api import ContactSequenceHumanoid
import mlp.viewer.display_tools as display_tools
from hpp.gepetto import Viewer,ViewerFactory
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver import ProblemSolver
import time
import os
class Robot (FullBody):
    packageName = "talos_data"
    meshPackageName = "talos_data"
    rootJointType = "freeflyer"    
    urdfName = "talos"
    urdfSuffix = "_reduced"
    srdfSuffix = ""    
    
    def __init__ (self, name = None,load = True):
        FullBody.__init__ (self,load)
        if load:
            self.loadFullBodyModel(self.urdfName, self.rootJointType, self.meshPackageName, self.packageName, self.urdfSuffix, self.srdfSuffix)
        if name != None:
            self.name = name
            
def initScene(envName = "multicontact/ground"):
    fullBody = Robot ()
    ps = ProblemSolver(fullBody)
    vf = ViewerFactory (ps)
    vf.loadObstacleModel ("hpp_environments", envName, "planning")
    v = vf.createViewer( displayCoM = True)    
    return v,fullBody
            
def loadMotionFromFiles(v,path,npzFilename,csFilename):
    # load cs from file : 
    cs = ContactSequenceHumanoid(0)
    cs.loadFromXML(path+csFilename, "ContactSequence")  
    display_tools.displaySteppingStones(cs,v)
    colors = [v.color.blue, v.color.green]
    display_tools.displayCOMTrajectory(cs,v,colors) 
    #extract data from npz archive : 
    res = wb_res.loadFromNPZ(path+npzFilename)
    display_tools.displayFeetTrajFromResult(v,res)
    plot.plotALLFromWB(cs,res)
    return res,cs
    
    
    
    
    
# example code : 
## kill previous instance of the viewer and server : 
os.system('killall gepetto-gui')
os.system('killall hpp-rbprm-server')
## start new instances : 
os.system('gepetto-gui &>/dev/null 2>&1')
os.system('hpp-rbprm-server &>/dev/null 2>&1')
time.sleep(2)

v,fb = initScene()
path = "/local/dev_hpp/src/multicontact-locomotion-planning/res/"
npzFile = "export/npz/talos_flatGround.npz"
csFile = "contact_sequences/talos_flatGround_COM.xml"
res,cs = loadMotionFromFiles(v,path,npzFile,csFile)

def dispCS(step = 0.2): 
    for p in cs.contact_phases:
        display_tools.displayWBconfig(v,p.reference_configurations[0])
        time.sleep(step)
     
def dispWB():
    display_tools.displayWBmotion(v,res.q_t,res.dt,0.05)

dispWB()


