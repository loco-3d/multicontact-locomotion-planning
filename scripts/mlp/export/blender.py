import os
import mlp.config as cfg
import mlp.viewer.display_tools as display_tools

# export at 25 fps
dt_display = 0.04

def export(q_t,v,dt):
    path = cfg.EXPORT_PATH+"/blender"
    if not os.path.exists(path):
        os.makedirs(path)
    filename = path+"/"+cfg.DEMO_NAME+".yaml"
    nodes = [cfg.Robot.urdfName]
    v.client.gui.setCaptureTransform(filename,nodes)
    id = 0
    step = dt_display / dt
    assert step%1 == 0 ,"display dt should be a multiple of dt"
    step = int(step)
    while id < q_t.shape[1]:
        display_tools.displayWBconfig(v,q_t[:,id])
        v.client.gui.captureTransform()
        id += step
    # display last config if the total duration is not a multiple of the dt
    display_tools.displayWBconfig(v,q_t[:,-1])
    v.client.gui.captureTransform()    
    print "motion exported to ",filename

def exportSteppingStones(v):
    from mlp.viewer.display_tools import STONE_GROUP, STONE_LF, STONE_LH,STONE_RF,STONE_RH
    path = cfg.EXPORT_PATH + "/blender/stepping_stones/"+cfg.DEMO_NAME
    if not os.path.exists(path):
        os.makedirs(path)
    print "## export stl in : ",path
    v.client.gui.writeNodeFile(STONE_LF, path+"/LF_stones.stl")
    v.client.gui.writeNodeFile(STONE_LH, path+"/LH_stones.stl")
    v.client.gui.writeNodeFile(STONE_RF, path+"/RF_stones.stl")
    v.client.gui.writeNodeFile(STONE_RH, path+"/RH_stones.stl")

    