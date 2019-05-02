import numpy as np
import mlp.utils.wholebody_result as wb_res
import mlp.utils.plot as plot
import multicontact_api
from multicontact_api import ContactSequenceHumanoid
import mlp.viewer.display_tools as display_tools
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
import time
import os
from mlp.utils.status import Status
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

            
def loadMotionFromFiles(v,path,npzFilename,csFilename):
    # load cs from file : 
    cs = ContactSequenceHumanoid(0)
    cs.loadFromBinary(path+csFilename)  
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
os.system('gepetto-gui &> log-viewer.log &')
os.system('hpp-rbprm-server &> log-rbprm.log &')
time.sleep(2)

fb,v = display_tools.initScene(Robot)
path = "/local/dev_hpp/src/multicontact-locomotion-planning/res/"
npzFile = "export/npz/talos_circle_oriented.npz"
csFile = "contact_sequences/talos_circle_oriented_COM.cs"
res,cs = loadMotionFromFiles(v,path,npzFile,csFile)
status = Status(path+"/../scripts/infos.log")


def dispCS(step = 0.2): 
    display_tools.displayContactSequence(v,cs,step)
     
def dispWB():
    display_tools.displayWBmotion(v,res.q_t,res.dt,0.05)

dispWB()


