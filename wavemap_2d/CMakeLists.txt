cmake_minimum_required(VERSION 3.0.2)
project(wavemap_2d)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Configure
if (ENABLE_BENCHMARKING)
  find_package(benchmark REQUIRED)
endif ()
if (CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  append_coverage_compiler_flags()
endif ()

set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -std=c++17 -march=native \
    -Wall -Wextra -Wno-deprecated-copy -Wno-class-memaccess \
    -fPIC -DEIGEN_INITIALIZE_MATRICES_BY_NAN")

if (USE_UBSAN)
  set(CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} -fsanitize=undefined -fno-omit-frame-pointer -g \
      -fsanitize=float-divide-by-zero -fsanitize=float-cast-overflow")
endif ()
if (USE_ASAN)
  set(CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer -g \
      -fsanitize-address-use-after-scope")
endif ()
if (USE_TSAN)
  set(CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} -fsanitize=thread -fno-omit-frame-pointer -g")
endif ()

if (USE_CLANG_TIDY)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif ()

# Libraries
cs_add_library(
  ${PROJECT_NAME}
  src/integrator/pointcloud_integrator.cc
  src/integrator/beam_model.cc
  src/integrator/fixed_logodds_model.cc
  src/datastructure/datastructure_base.cc
  src/utils/eigen_format.cc
  src/utils/evaluation_utils.cc)

# Binaries
cs_add_executable(carmen_processor app/carmen_processor.cc)
target_link_libraries(carmen_processor ${PROJECT_NAME})

# Tests
if (CATKIN_ENABLE_TESTING)
  cs_add_library(testing_main test/gtest_main.cc)

  catkin_add_gtest(test_cell_traits test/src/datastructure/test_cell_traits.cc)
  target_include_directories(test_cell_traits PRIVATE test/include)
  target_link_libraries(test_cell_traits ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_dense_grid test/src/datastructure/test_dense_grid.cc)
  target_include_directories(test_dense_grid PRIVATE test/include)
  target_link_libraries(test_dense_grid ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_hashed_blocks
                   test/src/datastructure/test_hashed_blocks.cc)
  target_include_directories(test_hashed_blocks PRIVATE test/include)
  target_link_libraries(test_hashed_blocks ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_pointcloud test/src/datastructure/test_pointcloud.cc)
  target_include_directories(test_pointcloud PRIVATE test/include)
  target_link_libraries(test_pointcloud ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_quadtree test/src/datastructure/test_quadtree.cc)
  target_include_directories(test_quadtree PRIVATE test/include)
  target_link_libraries(test_quadtree ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_sparse_vector
                   test/src/datastructure/test_sparse_vector.cc)
  target_include_directories(test_sparse_vector PRIVATE test/include)
  target_link_libraries(test_sparse_vector ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_iterators test/src/integrator/test_iterators.cc)
  target_include_directories(test_iterators PRIVATE test/include)
  target_link_libraries(test_iterators ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_measurement_models
                   test/src/integrator/test_measurement_models.cc)
  target_include_directories(test_measurement_models PRIVATE test/include)
  target_link_libraries(test_measurement_models ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_constexpr_functions
                   test/src/utils/test_constexpr_functions.cc)
  target_include_directories(test_constexpr_functions PRIVATE test/include)
  target_link_libraries(test_constexpr_functions ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_discrete_wavelet_transform
                   test/src/transform/test_discrete_wavelet_transform.cc)
  target_include_directories(test_discrete_wavelet_transform
                             PRIVATE test/include)
  target_link_libraries(test_discrete_wavelet_transform ${PROJECT_NAME}
                        testing_main)

  catkin_add_gtest(test_naive_haar test/src/transform/test_naive_haar.cc)
  target_include_directories(test_naive_haar PRIVATE test/include)
  target_link_libraries(test_naive_haar ${PROJECT_NAME} testing_main)

  if (ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test/*" "*/${PROJECT_NAME}/app/*"
                          "*/${PROJECT_NAME}/benchmark/*")
    add_code_coverage(NAME ${PROJECT_NAME}_coverage_report DEPENDENCIES tests)
  endif ()
endif ()

# Benchmarks
if (ENABLE_BENCHMARKING)
  cs_add_library(benchmarking_main benchmark/google_benchmark_main.cc)
  target_link_libraries(benchmarking_main benchmark::benchmark)

  add_executable(benchmark_measurement_integrator
                 benchmark/benchmark_measurement_integrator.cc)
  target_link_libraries(benchmark_measurement_integrator ${PROJECT_NAME}
                        benchmarking_main)

  add_executable(benchmark_sparse_array benchmark/benchmark_sparse_array.cc)
  target_link_libraries(benchmark_sparse_array ${PROJECT_NAME}
                        benchmarking_main)
endif ()

# Export
cs_install()
cs_export()
