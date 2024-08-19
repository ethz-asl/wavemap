# A small helper method to append tests defined in a pure CMake project to
# catkin's run_tests target, s.t. they're included when running:
# catkin test <package_name>
function(append_to_catkin_tests target)
  if (TARGET ${target})
    # Make sure the target is built before running tests
    add_dependencies(tests ${target})

    # Add it to catkin's run_tests target
    get_target_property(_target_path ${target} RUNTIME_OUTPUT_DIRECTORY)
    # cmake-lint: disable=C0301
    set(cmd "${_target_path}/${target} --gtest_output=xml:${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME}/gtest-${target}.xml")
    catkin_run_tests_target("gtest" ${target} "gtest-${target}.xml"
        COMMAND ${cmd}
        DEPENDENCIES ${target})
  endif ()
endfunction()
