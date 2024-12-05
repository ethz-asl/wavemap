add_executable(test_wavemap_io)

target_include_directories(test_wavemap_io PRIVATE
    ${PROJECT_SOURCE_DIR}/test/include)
target_sources(test_wavemap_io PRIVATE
    test_config_file_conversions.cc
    test_map_file_conversions.cc)
target_compile_definitions(test_wavemap_io PRIVATE
    DATADIR="${CMAKE_CURRENT_SOURCE_DIR}/../../data")

set_wavemap_target_properties(test_wavemap_io)
target_link_libraries(test_wavemap_io wavemap_core wavemap_io GTest::gtest_main)

gtest_discover_tests(test_wavemap_io)
