import mlp.config as cfg

#["pinocchioIntegration"]
method = cfg.simulator_method

if method == "pinocchioIntegration":
    from .pinocchio_integration import PinocchioIntegration as Simulator