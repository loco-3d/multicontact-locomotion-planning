import os
from mlp.utils.wholebody_result import Result, FromContactSequenceWB
from mlp.utils.requirements import Requirements

def export(cfg, cs_ref, cs):
    dt = cfg.IK_dt
    path = cfg.EXPORT_PATH
    name = cfg.DEMO_NAME + ".npz"
    Requirements.requireRootTrajectories(cs_ref, cfg)
    Requirements.requireZMPtrajectories(cs_ref, cfg)
    res = FromContactSequenceWB(cs_ref, cs, dt)
    res.exportNPZ(path, name)