from mlp.config import Config
cfg = Config()

# contact_generation_method_available = ["none","load", "rbprm"]

method = cfg.contact_generation_method

if method == "none":
    from mlp.viewer.display_tools import initScene
    from mlp.utils.requirements import Requirements as Outputs
    def generateContactSequence():
        fb, v = initScene(cfg.Robot, cfg.ENV_NAME)
        return None, fb, v

elif method == "load":
    from .fromCSfile import generateContactSequence, Outputs
elif method == "rbprm":
    from .rbprm import generateContactSequence, Outputs
elif method == "sl1m":
    from .sl1m import generateContactSequence, Outputs
else:
    raise ValueError("method type " + str(method) + " doesn't exist for contact generation")
