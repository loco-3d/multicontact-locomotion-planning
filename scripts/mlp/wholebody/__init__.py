import mlp.config as cfg

#wholebody_method_available = ["load", "tsid", "croccodyl"]
method = cfg.wholebody_method

if method == "load":
    from .fromNPZfile import generateWholeBodyMotion, Inputs, Outputs
elif method == "tsid":
    from .tsid_invdyn import generateWholeBodyMotion, Inputs, Outputs
elif method == "croccodyl":
    from .croccodyl import generateWholeBodyMotion, Inputs, Outputs
elif method == "none":
    from  mlp.utils.requirements import Requirements as Inputs
    from  mlp.utils.requirements import Requirements as Outputs
    def generateWholeBodyMotion(cs_ref, fullBody=None, viewer=None):
        print("Whole body motion not computed !")
        return None, None
else:
    raise ValueError("method type " + str(method) + " doesn't exist for wholeBody motion generation")
