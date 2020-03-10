from mlp.config import Config
cfg = Config()

#wholebody_method_available = ["load", "tsid", "croccodyl"]
method = cfg.wholebody_method

if method == "load":
    from .fromCSWBfile import generateWholeBodyMotion, Inputs, Outputs
elif method == "tsid":
    from .tsid_invdyn import generateWholeBodyMotion
    from mlp.utils.requirements import Requirements

    class Inputs(Requirements):
        consistentContacts = True
        friction = True
        timings = True
        centroidalTrajectories = True
        effectorTrajectories = True
        rootTrajectories = True


    class Outputs(Requirements):
        consistentContacts = True
        timings = True
        jointsTrajectories = True
        if cfg.IK_store_joints_derivatives:
            jointsDerivativesTrajectories = True
        if cfg.IK_store_joints_torque:
            torqueTrajectories = True
        if cfg.IK_store_centroidal:
            centroidalTrajectories = True
        if cfg.IK_store_effector:
            effectorTrajectories = True
        if cfg.IK_store_contact_forces:
            contactForcesTrajectories = False
        if cfg.IK_store_zmp:
            ZMPtrajectories = True


elif method == "croccodyl":
    from .croccodyl import generateWholeBodyMotion, Inputs, Outputs
elif method == "none":
    from  mlp.utils.requirements import Requirements as Inputs
    from  mlp.utils.requirements import Requirements as Outputs
    def generateWholeBodyMotion(cs_ref, cfg, fullBody=None, viewer=None):
        print("Whole body motion not computed !")
        return None
else:
    raise ValueError("method type " + str(method) + " doesn't exist for wholeBody motion generation")
