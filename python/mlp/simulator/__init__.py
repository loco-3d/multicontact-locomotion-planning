from mlp.config import Config
cfg = Config()

#["pinocchioIntegration"]
method = cfg.simulator_method

if method == "pinocchioIntegration":
    from .pinocchio_integration import PinocchioIntegration as Simulator