import os


def export(cfg, q_t):
    path = cfg.EXPORT_PATH + "/gazebo"
    print ("Export motion to "+path+" ... ")
    if not os.path.exists(path):
        os.makedirs(path)
    filename = path + "/" + cfg.DEMO_NAME + ".posture"
    dt = cfg.IK_dt
    with open(filename, 'w') as f:
        t = q_t.min()
        while t <= q_t.max():
            q = q_t(t)
            line = str(t - q_t.min()) + " "
            for k in range(7, len(q)):
                line += str(q[k]) + " "
            f.write(line + "\n")
            t += dt
    print("Motion exported in : ", filename)
