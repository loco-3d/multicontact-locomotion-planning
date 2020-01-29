from  mlp.utils import cs_tools



class Requirements():
    """
    This class is used to defined the inputs / outputs expected for each algorithms

    It provides methods to check if this requirements are satisfied by a given contact sequence,
    and may use methods to set them at default values when possibles
    """
    timings = False
    consistentContacts = False
    COMvalues = False
    AMvalues = False
    centroidalValues = False
    configurationValues = False
    COMtrajectories = False
    AMtrajectories = False
    centroidalTrajectories = False
    jointsTrajectories = False
    torqueTrajectories = False
    effectorTrajectories = False
    contactForcesTrajectories = False

    @classmethod
    def print(cls):
        print("The following data are required : ")
        if cls.timings:
            print("- Consistent timings")
        if cls.consistentContacts:
            print("- Consistent contacts")
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
        if cls.centroidalTrajectories:
            print("- c, dc, ddc, L, dL trajectories for each phases")
        if cls.jointsTrajectories:
            print("- q, dq, ddq trajectories for each phases")
        if cls.torqueTrajectories:
            print("- torque trajectories for each phases")
        if cls.effectorTrajectories:
            print("- effector trajectories for each phases before a contact creation")
        if cls.contactForcesTrajectories:
            print("- contact forces trajectories for each effector in contact")

    @classmethod
    def assertRequirements(cls, cs):
        print("# Assert requirements : ")
        if cls.timings:
            assert cs.haveTimings(), "Contact sequence do not have consistent timings."
        if cls.consistentContacts:
            assert cs.haveConsistentContacts(), "Contact sequence do not have consistent contacts."
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
        if cls.centroidalTrajectories:
            assert cs.haveCentroidalTrajectories (), "Contact sequence do not have consistent centroidal trajectories."
        if cls.jointsTrajectories:
            assert cs.haveJointsTrajectories(), "Contact sequence do not have consistent joints trajectories."
        if cls.torqueTrajectories:
            assert cs.haveTorquesTrajectories(), "Contact sequence do not have consistent torques trajectories"
        if cls.effectorTrajectories:
            assert cs.haveEffectorsTrajectories(), "Contact sequence do not have consistent effector trajectories."
        if cls.contactForcesTrajectories:
            assert cs.haveContactForcesTrajectories(), "Contact sequence do not have consistent contact forces trajectories."
        print("# Assert requirements done.")

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
        print("# check requirements : ")

        if cls.timings:
            if not cs.haveTimings():
                print("- Contact sequence do not have consistent timings.")
                print("Compute timings from predefined values in configuration file ...")
                cs = cs_tools.computePhasesTimings(cs, cfg)
                if not cs.haveTimings():
                    print("An error occurred in cs_tools.computePhasesTimings")
                    return False
        if cls.consistentContacts:
            if not cs.haveConsistentContacts():
                print("- Contact sequence do not have consistent contacts.")
                return False
        if cls.COMvalues:
            if not cs.haveCOMvalues():
                print("- Contact sequence do not have consistent CoM values.")
                print("Compute CoM initial and final positions by projecting on the support polygon for each phase ...")
                cs = cs_tools.computePhasesCOMValues(cs, cfg.Robot.DEFAULT_COM_HEIGHT)
                if not cs.haveCOMvalues():
                    print("An error occurred in cs_tools.computePhasesCOMValues")
                    return False
        if cls.AMvalues:
            if not cs.haveAMvalues():
                print("- Contact sequence do not have consistent AM values .")
                return False
        if cls.centroidalValues:
            if not cs.haveCentroidalValues():
                print("- Contact sequence do not have consistent centroidal values.")
                return False
        if cls.configurationValues:
            if not cs.haveConfigurationsValues():
                print("- Contact sequence do not have consistent configurations values.")
                if fullBody is not None:
                    print("Try to compute configuration from inverse kinematics from the contacts ... ")
                    cs = cs_tools.computePhasesConfigurations(cs,fullBody)
                    if not cs.haveConfigurationsValues():
                        print("An error occurred in cs_tools.computePhasesConfigurations")
                        return False
                else:
                    print("Cannot compute this values without a fullBody object.")
                    return False
        if cls.COMtrajectories:
            if not cs.haveCOMtrajectories():
                print("- Contact sequence do not have consistent CoM trajectories.")
                return False
        if cls.AMtrajectories:
            if not cs.haveAMtrajectories():
                print("- Contact sequence do not have consistent AM trajectories.")
                return False
        if cls.centroidalTrajectories:
            if not cs.haveCentroidalTrajectories ():
                print("- Contact sequence do not have consistent centroidal trajectories.")
                return False
        if cls.jointsTrajectories:
            if not cs.haveJointsTrajectories():
                print("- Contact sequence do not have consistent joints trajectories.")
                return False
        if cls.torqueTrajectories:
            if not cs.haveTorquesTrajectories():
                print("- Contact sequence do not have consistent torques trajectories")
                return False
        if cls.effectorTrajectories:
            if not cs.haveEffectorsTrajectories():
                print("- Contact sequence do not have consistent effector trajectories.")
                return False
        if cls.contactForcesTrajectories:
            if not cs.haveContactForcesTrajectories():
                print("- Contact sequence do not have consistent contact forces trajectories.")
                return False
        print("# check requirements done. ")

        return True