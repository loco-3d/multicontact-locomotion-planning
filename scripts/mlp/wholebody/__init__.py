import mlp.config as cfg 

#wholebody_method_available = ["load", "tsid", "croccodyl"]
method = cfg.wholebody_method

if method == "load":
    from .fromNPZfile import generateWholeBodyMotion
elif method == "tsid":
    from .tsid_invdyn import generateWholeBodyMotion
elif method == "croccodyl":
    from .croccodyl import generateWholeBodyMotion
elif method == "none":
    def generateWholeBodyMotion(cs,fullBody=None,viewer=None):
        print("Whole body motion not computed !")
        return None,None
else : 
    raise ValueError("method type "+str(method)+" doesn't exist for wholeBody motion generation")
