add_executable(test_wavemap_io)

target_include_directories(test_wavemap_io PRIVATE
    ${PROJECT_SOURCE_DIR}/test/include)
target_sources(test_wavemap_io PRIVATE test_file_conversions.cc)

set_wavemap_target_properties(test_wavemap_io)
target_link_libraries(test_wavemap_io wavemap_core wavemap_io GTest::gtest_main)

gtest_discover_tests(test_wavemap_io)
