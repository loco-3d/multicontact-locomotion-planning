import mlp.config as cfg

# contact_generation_method_available = ["none","load", "rbprm"]

method = cfg.contact_generation_method

if method == "none":
    from mlp.viewer.display_tools import initScene
    def generateContactSequence():
        fb,v = initScene(cfg.Robot,cfg.ENV_NAME)
        return None,fb,v
elif method == "load":
    from .fromCSfile import generateContactSequence
elif method == "rbprm":
    from .rbprm import generateContactSequence
else :
    raise ValueError("method type "+str(method)+" doesn't exist for contact generation")