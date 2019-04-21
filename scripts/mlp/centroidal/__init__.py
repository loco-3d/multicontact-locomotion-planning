import mlp.config as cfg 

#centroidal_initGuess_method_available = ["none", "geometric", "croc", "timeopt", "quasistatic"]
#centroidal_method_available = ["load", "geometric", "croc", "timeopt", "quasistatic", "muscod"]

method_initGuess = cfg.centroidal_initGuess_method
method = cfg.centroidal_method

if method_initGuess == "none":
    def generateCentroidalInitGuess(cs,cs_initGuess = None,fullBody=None, viewer =None):
        return None
elif method_initGuess == "geometric":
    from .geometric import generateCentroidalTrajectory as generateCentroidalInitGuess
elif method_initGuess == "croc":
    from .croc import generateCentroidalTrajectory as generateCentroidalInitGuess
elif method_initGuess == "timeopt":
    from .topt import generateCentroidalTrajectory as generateCentroidalInitGuess
elif method_initGuess == "quasistatic":
    from .quasiStatic import generateCentroidalTrajectory as generateCentroidalInitGuess    
else : 
    raise ValueError("method type "+str(method)+" doesn't exist for centroidal initGuess")

if method == "load":
    from .fromCSCOMfile import generateCentroidalTrajectory
elif method == "geometric":
    from .geometric import generateCentroidalTrajectory
elif method == "croc":
    from .croc import generateCentroidalTrajectory 
elif method == "timeopt":
    from .topt import generateCentroidalTrajectory 
elif method == "quasistatic":
    from .quasiStatic import generateCentroidalTrajectory  
elif method == "muscod":
    from .muscod import generateCentroidalTrajectory    
else : 
    raise ValueError("method type "+str(method)+" doesn't exist for centroidal trajectory generation")