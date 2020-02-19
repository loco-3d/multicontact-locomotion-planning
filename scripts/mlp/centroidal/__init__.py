import mlp.config as cfg

#centroidal_method_available = ["load", "geometric", "croc", "timeopt", "quasistatic", "muscod"]

method = cfg.centroidal_method

if method == "load":
    from .fromCSCOMfile import generateCentroidalTrajectory, Inputs, Outputs
elif method == "geometric":
    from .geometric import generateCentroidalTrajectory, Inputs, Outputs
elif method == "croc":
    from .croc import generateCentroidalTrajectory, Inputs, Outputs
elif method == "timeopt":
    from .topt import generateCentroidalTrajectory, Inputs, Outputs
elif method == "quasistatic":
    from .quasiStatic import generateCentroidalTrajectory, Inputs, Outputs
elif method == "muscod":
    from .muscod import generateCentroidalTrajectory, Inputs, Outputs
elif method == "none":
    from  mlp.utils.requirements import Requirements as Inputs
    from  mlp.utils.requirements import Requirements as Outputs
    def generateCentroidalTrajectory(cs, cs_initGuess=None, fullBody=None, viewer=None):
        print("Centroidal trajectory not computed !")
        return None
else:
    raise ValueError("method type " + str(method) + " doesn't exist for centroidal trajectory generation")
