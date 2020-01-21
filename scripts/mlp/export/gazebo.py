import os
import mlp.config as cfg

def export(q_t):
    path = cfg.EXPORT_PATH+"/gazebo"
    if not os.path.exists(path):
        os.makedirs(path)
    filename = path+"/"+cfg.DEMO_NAME+".posture"
    dt = cfg.IK_dt
    with open(filename,'w') as f:
        for i in range(len(q_t)):
            q = q_t[i]
            line = str(dt*i)+" "
            for k in range(7,len(q)):
                line +=str(q[k,0])+" "
            f.write(line+"\n")
    print("Motion exported in : ",filename)