import os
import mlp.config as cfg
import mlp.viewer.display_tools as display_tools

def export(q_t,v):
    path = cfg.EXPORT_PATH+"/blender"
    if not os.path.exists(path):
        os.makedirs(path)
    filename = path+"/"+cfg.DEMO_NAME+".yaml"
    nodes = [cfg.Robot.urdfName]
    v.client.gui.setCaptureTransform(filename,nodes)
    display_tools.displayWBconfig(v,q_t[:,0])
    v.client.gui.captureTransformOnRefresh(True)
    display_tools.displayWBmotion(v,q_t,cfg.IK_dt,cfg.DT_DISPLAY)
    v.client.gui.captureTransformOnRefresh(False)
    print "motion exported to ",filename
    
    