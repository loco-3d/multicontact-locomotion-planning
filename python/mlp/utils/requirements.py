from  mlp.utils import cs_tools



class Requirements():
    """
    This class is used to defined the inputs / outputs expected for each algorithms

    It provides methods to check if this requirements are satisfied by a given contact sequence,
    and may use methods to set them at default values when possibles
    """
    timings = False
    consistentContacts = False
    friction = False
    contactModel = False
    rootTrajectories = False
    COMvalues = False
    AMvalues = False
    centroidalValues = False
    configurationValues = False
    COMtrajectories = False
    AMtrajectories = False
    ZMPtrajectories = False
    centroidalTrajectories = False
    jointsTrajectories = False
    jointsDerivativesTrajectories = False
    torqueTrajectories = False
    effectorTrajectories = False
    contactForcesTrajectories = False

    @classmethod
    def requireTimings(cls, cs, cfg):
        if not cs.haveTimings():
            print("- Contact sequence do not have consistent timings.")
            if cfg:
                print("Compute timings from predefined values in configuration file ...")
                cs = cs_tools.computePhasesTimings(cs, cfg)
                if not cs.haveTimings():
                    print("An error occurred in cs_tools.computePhasesTimings")
                    return False
            else:
                print("Cannot compute timings without a cfg pointer.")
                return False
        return True

    @classmethod
    def requireConsistentContacts(cls, cs):
        if not cs.haveConsistentContacts():
            print("- Contact sequence do not have consistent contacts.")
            return False
        else:
            return True

    @classmethod
    def requireFriction(cls, cs, mu):
        if not cs.haveFriction():
            print("- Contact sequence do not have consistent contacts.")
            print("Set all the uninitialized patch with the defaut value : ",mu )
            cs_tools.setAllUninitializedFrictionCoef(cs, mu)
            if not cs.haveFriction():
                print("An error occurred in setAllUninitializedFrictionCoef.")
                return False
        return True

    @classmethod
    def requireContactModel(cls, cs, Robot):
        if not cs.haveContactModelDefined():
            print("- Contact sequence do not have ContactModel defined.")
            print("Set all the uninitialized model from the Robot class")
            cs_tools.setAllUninitializedContactModel(cs, Robot)
            if not cs.haveContactModelDefined():
                print("An error occurred in setAllUninitializedContactModel.")
                return False
        return True


    @classmethod
    def requireRootTrajectories(cls, cs, cfg):
        if not cs.haveRootTrajectories():
            print("- Contact sequence do not have consistent root trajectories.")
            cls.requireTimings(cs, cfg)
            if cs.haveConfigurationsValues():
                print("Compute it from the configurations ...")
                cs_tools.computeRootTrajFromConfigurations(cs)
            else:
                print("Compute it from the contact placements ...")
                cs_tools.computeRootTrajFromContacts(cfg.Robot, cs)
            if not cs.haveRootTrajectories():
                print("An error occurred in during root trajectory computation.")
                return False
        return True

    @classmethod
    def requireCOMvalues(cls, cs, default_height):
        if not cs.haveCOMvalues():
            print("- Contact sequence do not have consistent CoM values.")
            print("Compute CoM initial and final positions by projecting on the support polygon for each phase ...")
            cs = cs_tools.computePhasesCOMValues(cs, default_height)
            if not cs.haveCOMvalues():
                print("An error occurred in cs_tools.computePhasesCOMValues")
                return False
        return True

    @classmethod
    def requireAMvalues(cls, cs):
        if not cs.haveAMvalues():
            print("- Contact sequence do not have consistent AM values .")
            return False
        return True

    @classmethod
    def requireCentroidalValues(cls, cs):
        if not cs.haveCentroidalValues():
            print("- Contact sequence do not have consistent centroidal values.")
            return False
        return True

    @classmethod
    def requireConfigurationValues(cls, cs, fullBody = None, cfg = None):
        if not cs.haveConfigurationsValues():
            print("- Contact sequence do not have consistent configurations values.")
            if fullBody:
                print("Try to compute configuration from inverse kinematics from the contacts ... ")
                cls.requireRootTrajectories(cs, cfg)
                cs = cs_tools.computePhasesConfigurations(cs, fullBody)
                if not cs.haveConfigurationsValues():
                    print("An error occurred in cs_tools.computePhasesConfigurations")
                    return False
            else:
                print("Cannot compute this values without a fullBody object.")
                return False
        return True

    @classmethod
    def requireCOMtrajectories(cls, cs):
        if not cs.haveCOMtrajectories():
            print("- Contact sequence do not have consistent CoM trajectories.")
            return False
        return True

    @classmethod
    def requireAMtrajectories(cls, cs):
        if not cs.haveAMtrajectories():
            print("- Contact sequence do not have consistent AM trajectories.")
            return False # TODO gen 0 splines
        return True

    @classmethod
    def requireZMPtrajectories(cls, cs, cfg):
        if not cs.haveZMPtrajectories():
            print("- Contact sequence do not have consistent ZMP trajectories.")
            if cs.haveCOMtrajectories() and cs.haveAMtrajectories():
                print(" compute it from the centroidal data")
                from mlp.utils.computation_tools import computeZMPRef
                computeZMPRef(cs, cfg)
                if not cs.haveZMPtrajectories():
                    print("An error occurred in computation_tools.computeZMPRef")
                    return False
            else:
                return False
        return True

    @classmethod
    def requireCentroidalTrajectories(cls, cs):
        return cls.requireAMtrajectories(cs) and cls.requireCOMtrajectories(cs)

    @classmethod
    def requireJointsTrajectories(cls, cs):
        if not cs.haveJointsTrajectories():
            print("- Contact sequence do not have consistent joints trajectories.")
            return False
        return True

    @classmethod
    def requireJointsDerivativesTrajectories(cls, cs):
        if not cs.haveJointsDerivativesTrajectories():
            print("- Contact sequence do not have consistent joints derivatives trajectories.")
            return False
        return True

    @classmethod
    def requireTorqueTrajectories(cls, cs):
        if not cs.haveTorquesTrajectories():
            print("- Contact sequence do not have consistent torques trajectories")
            return False
        return True

    @classmethod
    def requireEffectorTrajectories(cls, cs):
        if not cs.haveEffectorsTrajectories(1e-2):
            print("- Contact sequence do not have consistent effector trajectories.")
            return False
        return True

    @classmethod
    def requireContactForcesTrajectories(cls, cs):
        if not cs.haveContactForcesTrajectories():
            print("- Contact sequence do not have consistent contact forces trajectories.")
            return False
        return True


    @classmethod
    def print(cls):
        print("The following data are required : ")
        if cls.timings:
            print("- Consistent timings")
        if cls.consistentContacts:
            print("- Consistent contacts")
        if cls.friction:
            print("- friction coefficient for all contacts")
        if cls.contactModel:
            print("- ContactModel defined for all ContactPhases")
        if cls.COMvalues:
            print("- c_init / final for each phases")
        if cls.AMvalues:
            print("- L_init / final for each phases")
        if cls.centroidalValues:
            print("- c_init and L_init / final for each phases")
        if cls.configurationValues:
            print("- q_init / final for each phases")
        if cls.COMtrajectories:
            print("- c, dc, ddc trajectories for each phases")
        if cls.AMtrajectories:
            print("- L, dL trajectories for each phases")
        if cls.ZMPtrajectories:
            print("- ZMP trajectories for each phases")
        if cls.centroidalTrajectories:
            print("- c, dc, ddc, L, dL trajectories for each phases")
        if cls.jointsTrajectories:
            print("- q trajectories for each phases")
        if cls.jointsDerivativesTrajectories:
            print("- dq, ddq trajectories for each phases")
        if cls.torqueTrajectories:
            print("- torque trajectories for each phases")
        if cls.effectorTrajectories:
            print("- effector trajectories for each phases before a contact creation")
        if cls.contactForcesTrajectories:
            print("- contact forces trajectories for each effector in contact")

    @classmethod
    def assertRequirements(cls, cs):
        #print("# Assert requirements : ")
        if cls.timings:
            assert cs.haveTimings(), "Contact sequence do not have consistent timings."
        if cls.consistentContacts:
            assert cs.haveConsistentContacts(), "Contact sequence do not have consistent contacts."
        if cls.friction:
            assert cs.haveFriction(), "Contact sequence do not have friction coefficient defined for all contacts."
        if cls.contactModel:
            assert cs.haveContactModelDefined(), "Contact Sequence do not have ContactModel defined for all phases."
        if cls.rootTrajectories:
            assert cs.haveRootTrajectories(), "Contact sequence do not have consistent Root trajectories."
        if cls.COMvalues:
            assert cs.haveCOMvalues(), "Contact sequence do not have consistent CoM values."
        if cls.AMvalues:
            assert cs.haveAMvalues(), "Contact sequence do not have consistent AM values ."
        if cls.centroidalValues:
            assert cs.haveCentroidalValues(), "Contact sequence do not have consistent centroidal values."
        if cls.configurationValues:
            assert cs.haveConfigurationsValues(), "Contact sequence do not have consistent configurations values."
        if cls.COMtrajectories:
            assert cs.haveCOMtrajectories(), "Contact sequence do not have consistent CoM trajectories."
        if cls.AMtrajectories:
            assert cs.haveAMtrajectories(), "Contact sequence do not have consistent AM trajectories."
        if cls.ZMPtrajectories:
            assert cs.haveZMPtrajectories(), "Contact sequence do not have consistent ZMP trajectories."
        if cls.centroidalTrajectories:
            assert cs.haveCentroidalTrajectories (), "Contact sequence do not have consistent centroidal trajectories."
        if cls.jointsTrajectories:
            assert cs.haveJointsTrajectories(), "Contact sequence do not have consistent joints trajectories."
        if cls.jointsDerivativesTrajectories:
            assert cs.haveJointsDerivativesTrajectories(), "Contact sequence do not have consistent joints trajectories."
        if cls.torqueTrajectories:
            assert cs.haveTorquesTrajectories(), "Contact sequence do not have consistent torques trajectories"
        if cls.effectorTrajectories:
            assert cs.haveEffectorsTrajectories(1e-2), "Contact sequence do not have consistent effector trajectories."
        if cls.contactForcesTrajectories:
            assert cs.haveContactForcesTrajectories(), "Contact sequence do not have consistent contact forces trajectories."
        #print("# Assert requirements done.")

    @classmethod
    def assertWholebodyData(cls, cs, cfg):
        """
        Assert that the selected options in cfg.IK_store_* are correctly filled
        :param cs: the ContactSequence
        :param cfg: an instance of the configuration class
        :return:
        """
        if cfg.IK_store_joints_derivatives:
            assert cs.haveJointsDerivativesTrajectories(), "Contact sequence do not have consistent joints trajectories."
        if cfg.IK_store_joints_torque:
            assert cs.haveTorquesTrajectories(), "Contact sequence do not have consistent torques trajectories"
        if cfg.IK_store_centroidal:
            assert cs.haveCentroidalTrajectories (), "Contact sequence do not have consistent centroidal trajectories."
        if cfg.IK_store_effector:
            assert cs.haveEffectorsTrajectories(1e-2), "Contact sequence do not have consistent effector trajectories."
        if cfg.IK_store_contact_forces:
            assert cs.haveContactForcesTrajectories(), "Contact sequence do not have consistent contact forces trajectories."
        if cfg.IK_store_zmp:
            assert cs.haveZMPtrajectories(), "Contact sequence do not have consistent ZMP trajectories."

    @classmethod
    def checkAndFillRequirements(cls, cs, cfg, fullBody = None):
        """
        Check if the given ContactSequence satisfy the requirement,
        for some requirement it can try to set default values from the configuration file
        :param cs: ContactSequence to test
        :param cfg: configuration file
        :param fullBody: rbprm.FullBody instance with a robot loaded
        :return: True if either it satisfy the requirement or it managed to set them to default values, False otherwise
        """
        #print("# check requirements : ")

        if cls.timings:
            if not cls.requireTimings(cs, cfg):
                return False
        if cls.consistentContacts:
            if not cls.requireConsistentContacts(cs):
                return False
        if cls.friction:
            if not cls.requireFriction(cs, cfg.MU):
                return False
        if cls.contactModel:
            if not cls.requireContactModel(cs, cfg.Robot):
                return False
        if cls.rootTrajectories:
            if not cls.requireRootTrajectories(cs, cfg):
                return False
        if cls.COMvalues:
            if not cls.requireCOMvalues(cs, cfg.Robot.DEFAULT_COM_HEIGHT):
                return False
        if cls.AMvalues:
            if not cls.requireAMvalues(cs):
                return False
        if cls.centroidalValues:
            if not cls.requireCentroidalValues(cs):
                return False
        if cls.configurationValues:
            if not cls.requireConfigurationValues(cs, fullBody, cfg):
                return False
        if cls.COMtrajectories:
            if not cls.requireCOMtrajectories(cs):
                return False
        if cls.AMtrajectories:
            if not cls.requireAMtrajectories(cs):
                return False
        if cls.ZMPtrajectories:
            if not cls.requireZMPtrajectories(cs, cfg):
                return False
        if cls.centroidalTrajectories:
            if not cls.requireCentroidalTrajectories(cs):
                return False
        if cls.jointsTrajectories:
            if not cls.requireJointsTrajectories(cs):
                return False
        if cls.jointsDerivativesTrajectories:
            if not cls.requireJointsDerivativesTrajectories(cs):
                return False
        if cls.torqueTrajectories:
            if not cls.requireTorqueTrajectories(cs):
                return False
        if cls.effectorTrajectories:
            if not cls.requireEffectorTrajectories(cs):
                return False
        if cls.contactForcesTrajectories:
            if not cls.requireContactForcesTrajectories(cs):
                return False
        #print("# check requirements done. ")

        return True