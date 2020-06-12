from mlp.utils import check_path

def check_motion(testCase, planner, contains_all_data=True):

    # check that the size of the sequences are correct:
    testCase.assertTrue(planner.cs.size() > 2)
    testCase.assertTrue(planner.cs.size() < 50)
    testCase.assertEqual(planner.cs.size(), planner.cs_com.size())
    testCase.assertEqual(planner.cs.size(), planner.cs_ref.size())
    testCase.assertEqual(planner.cs.size(), planner.cs_wb.size())

    # check that the sequence contains the expected data:
    testCase.assertTrue(planner.cs.haveConsistentContacts())

    testCase.assertTrue(planner.cs_ref.haveConsistentContacts())
    testCase.assertTrue(planner.cs_ref.haveTimings())
    testCase.assertTrue(planner.cs_ref.haveRootTrajectories())
    testCase.assertTrue(planner.cs_ref.haveCentroidalTrajectories())
    testCase.assertTrue(planner.cs_ref.haveZMPtrajectories())
    testCase.assertTrue(planner.cs_ref.haveEffectorsTrajectories(1e-2))

    testCase.assertTrue(planner.cs_wb.haveConsistentContacts())
    testCase.assertTrue(planner.cs_wb.haveTimings())
    testCase.assertTrue(planner.cs_wb.haveJointsTrajectories())
    if contains_all_data:
        testCase.assertTrue(planner.cs_wb.haveJointsTrajectories())
        testCase.assertTrue(planner.cs_wb.haveJointsDerivativesTrajectories())
        testCase.assertTrue(planner.cs_wb.haveCentroidalTrajectories())
        testCase.assertTrue(planner.cs_wb.haveContactForcesTrajectories())
        testCase.assertTrue(planner.cs_wb.haveTorquesTrajectories())
        testCase.assertTrue(planner.cs_wb.haveZMPtrajectories())
        testCase.assertTrue(planner.cs_wb.haveEffectorsTrajectories(1e-2))

    # check that the motion is valid:
    validator = check_path.PathChecker(planner.fullBody, planner.cfg.CHECK_DT, True)
    motion_valid, _ = validator.check_motion(planner.cs_wb.concatenateQtrajectories())
    testCase.assertTrue(motion_valid)
