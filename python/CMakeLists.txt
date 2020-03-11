
SET(${PROJECT_NAME}_PYTHON_FILES
    __init__.py
    config.py
    loco_planner.py
    )

SET(${PROJECT_NAME}_CENTROIDAL_FILES
    croc.py
    geometric.py
    __init__.py
    load.py
    momentumopt.py
    muscod.py
    none.py
    quasistatic.py
    )

SET(${PROJECT_NAME}_CONTACT_FILES
    __init__.py
    load.py
    rbprm.py
    sl1m.py
    )

SET(${PROJECT_NAME}_EFFECTOR_FILES
    bezier_constrained.py
    bezier_predef.py
    __init__.py
    limb_rrt_optimized.py
    limb_rrt.py
    load.py
    smoothed_foot.py
    )

SET(${PROJECT_NAME}_WHOLEBODY_FILES
    croccodyl.py
    __init__.py
    load.py
    none.py
    tsid.py
    )

SET(${PROJECT_NAME}_SIMULATOR_FILES
    __init__.py
    pinocchio_integration.py
    )

SET(${PROJECT_NAME}_EXPORT_FILES
    blender.py
    gazebo.py
    __init__.py
    npz.py
    openHRP.py
    sotTalosBalance.py
    )

SET(${PROJECT_NAME}_UTILS_FILES
    check_path.py
    computation_tools.py
    cs_tools.py
    derivative_filters.py
    __init__.py
    plot.py
    requirements.py
    status.py
    trajectories.py
    util.py
    wholebody_result.py
    )

SET(${PROJECT_NAME}_VIEWER_FILES
    __init__.py
    display_tools.py
    )

SET(${PROJECT_NAME}_DEMOS_FILES
    anymal_circle_oriented.py
    anymal_circle.py
    anymal_flatGround.py
    anymal_modular_palet.py
    anymal_platform_random.py
    anymal_slalom_debris.py
    anymal_slalom.py
    common_anymal.py
    common_hrp2.py
    common_hyq.py
    common_talos_fixedUpper.py
    common_talos.py
    darpa_hyq.py
    hrp2_flatGround.py
    hrp2_lp_complex.py
    hrp2_pushRecovery.py
    hyq_slalom_debris.py
    __init__.py
    talos_bauzil_with_stairs.py
    talos_circle_oriented.py
    talos_circle.py
    talos_flatGround.py
    talos_flatGround_quasiStatic.py
    talos_lp_complex.py
    talos_lp_maze.py
    talos_lp_slalom.py
    talos_mazeEas_oriented.py
    talos_mazeEas.py
    talos_moveEffector_flat.py
    talos_moveEffector_stairs_m10.py
    talos_moveEffector_stairs_m15.py
    talos_moveEffector_stairs_p10.py
    talos_moveEffector_stairs_p15.py
    talos_navBauzil.py
    talos_obstaclesFeet.py
    talos_plateformes_large.py
    talos_plateformes.py
    talos_plateformes_quasiStatic.py
    talos_platform_random.py
    talos_pushRecovery.py
    talos_randomMove_flat.py
    talos_stairs10.py
    talos_stairs10_random.py
    talos_table.py
    )

# Install python files : 
FOREACH(file ${${PROJECT_NAME}_PYTHON_FILES})
  PYTHON_INSTALL_ON_SITE("mlp" ${file})
ENDFOREACH(file ${${PROJECT_NAME}_PYTHON_FILES})

FOREACH(file ${${PROJECT_NAME}_CENTROIDAL_FILES})
  PYTHON_INSTALL_ON_SITE("mlp/centroidal" ${file})
ENDFOREACH(file ${${PROJECT_NAME}_CENTROIDAL_FILES})

FOREACH(file ${${PROJECT_NAME}_CONTACT_FILES})
  PYTHON_INSTALL_ON_SITE("mlp/contact_sequence" ${file})
ENDFOREACH(file ${${PROJECT_NAME}_CONTACT_FILES})

FOREACH(file ${${PROJECT_NAME}_EFFECTOR_FILES})
  PYTHON_INSTALL_ON_SITE("mlp/end_effector" ${file})
ENDFOREACH(file ${${PROJECT_NAME}_EFFECTOR_FILES})

FOREACH(file ${${PROJECT_NAME}_WHOLEBODY_FILES})
  PYTHON_INSTALL_ON_SITE("mlp/wholebody" ${file})
ENDFOREACH(file ${${PROJECT_NAME}_WHOLEBODY_FILES})

FOREACH(file ${${PROJECT_NAME}_SIMULATOR_FILES})
  PYTHON_INSTALL_ON_SITE("mlp/simulator" ${file})
ENDFOREACH(file ${${PROJECT_NAME}_SIMULATOR_FILES})

FOREACH(file ${${PROJECT_NAME}_EXPORT_FILES})
  PYTHON_INSTALL_ON_SITE("mlp/export" ${file})
ENDFOREACH(file ${${PROJECT_NAME}_EXPORT_FILES})

FOREACH(file ${${PROJECT_NAME}_UTILS_FILES})
  PYTHON_INSTALL_ON_SITE("mlp/utils" ${file})
ENDFOREACH(file ${${PROJECT_NAME}_UTILS_FILES})

FOREACH(file ${${PROJECT_NAME}_VIEWER_FILES})
  PYTHON_INSTALL_ON_SITE("mlp/viewer" ${file})
ENDFOREACH(file ${${PROJECT_NAME}_VIEWER_FILES})

FOREACH(file ${${PROJECT_NAME}_DEMOS_FILES})
  PYTHON_INSTALL_ON_SITE("mlp/demo_configs" ${file})
ENDFOREACH(file ${${PROJECT_NAME}_DEMOS_FILES})


