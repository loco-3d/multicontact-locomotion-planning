SET(${PROJECT_NAME}_PYTHON_TESTS
  test_talos_walk_sl1m_quasistatic
  test_talos_walk_sl1m_quasistatic_no_store
  test_talos_walk_rbprm_mopt
  test_talos_walk_sl1m_mopt_dyn_filter
  test_talos_walk_sl1m_topt
  )

FOREACH(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
  ADD_PYTHON_UNIT_TEST("py-${TEST}" "tests/python/${TEST}.py")
  SET_TESTS_PROPERTIES("py-${TEST}" PROPERTIES RUN_SERIAL "ON" TIMEOUT 600)
ENDFOREACH(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
