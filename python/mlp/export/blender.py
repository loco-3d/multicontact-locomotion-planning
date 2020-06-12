import os
import mlp.viewer.display_tools as display_tools

# export at 25 fps
dt_display = 0.04


def export(cfg, q_t, v):
    path = cfg.EXPORT_PATH + "/blender"
    if not os.path.exists(path):
        os.makedirs(path)
    filename = path + "/" + cfg.DEMO_NAME + ".yaml"
    print ("Export motion to "+filename+" ... ")
    nodes = [cfg.Robot.urdfName]
    v.client.gui.setCaptureTransform(filename, nodes)
    t = q_t.min()
    while t <= q_t.max():
        display_tools.displayWBconfig(v, q_t(t))
        v.client.gui.captureTransform()
        t += dt_display
    # display last config if the total duration is not a multiple of the dt
    display_tools.displayWBconfig(v, q_t(q_t.max()))
    v.client.gui.captureTransform()
    print("motion exported to ", filename)


def exportSteppingStones(v):
    from mlp.viewer.display_tools import STONE_GROUP, STONE_LF, STONE_LH, STONE_RF, STONE_RH
    path = cfg.EXPORT_PATH + "/blender/stepping_stones/" + cfg.DEMO_NAME
    if not os.path.exists(path):
        os.makedirs(path)
    print("## export stl in : ", path)
    v.client.gui.writeNodeFile(STONE_LF, path + "/LF_stones.stl")
    v.client.gui.writeNodeFile(STONE_LH, path + "/LH_stones.stl")
    v.client.gui.writeNodeFile(STONE_RF, path + "/RF_stones.stl")
    v.client.gui.writeNodeFile(STONE_RH, path + "/RH_stones.stl")
