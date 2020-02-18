import mlp.config as cfg
#centroidal_initGuess_method_available = ["none", "geometric", "croc", "timeopt", "quasistatic"]

method_initGuess = cfg.centroidal_initGuess_method

if method_initGuess == "none":
    from  mlp.utils.requirements import Requirements as Inputs
    from  mlp.utils.requirements import Requirements as Outputs
    def generateCentroidalTrajectory(cs, cs_initGuess=None, fullBody=None, viewer=None):
        return None
elif method_initGuess == "geometric":
    from .geometric import generateCentroidalTrajectory, Inputs, Outputs
elif method_initGuess == "croc":
    from .croc import generateCentroidalTrajectory, Inputs, Outputs
elif method_initGuess == "timeopt":
    from .topt import generateCentroidalTrajectory, Inputs, Outputs
elif method_initGuess == "quasistatic":
    from .quasiStatic import generateCentroidalTrajectory, Inputs, Outputs
else:
    raise ValueError("method type " + str(method_initGuess) + " doesn't exist for centroidal initGuess")
