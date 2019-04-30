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
        displayWBconfig(viewer,q_t[:,id])
        v.client.gui.captureTransform()
        id += step
    # display last config if the total duration is not a multiple of the dt
    displayWBconfig(viewer,q_t[:,-1])
    v.client.gui.captureTransform()    
    print "motion exported to ",filename
    
    