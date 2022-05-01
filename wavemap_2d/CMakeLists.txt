cmake_minimum_required(VERSION 3.0.2)
project(wavemap_2d)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Configure
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_compile_definitions(EIGEN_INITIALIZE_MATRICES_BY_NAN)
add_compile_options(
  -march=native -Wall -Wextra -Wpedantic -Wsuggest-attribute=const
  -Wno-deprecated-copy -Wno-class-memaccess)

if (USE_UBSAN)
  add_compile_options(
    -fsanitize=undefined
    -fsanitize=shift
    -fsanitize=integer-divide-by-zero
    -fsanitize=null
    -fsanitize=return
    -fsanitize=float-divide-by-zero
    -fsanitize=float-cast-overflow
    -fsanitize=builtin
    -fno-omit-frame-pointer
    -g)
  add_link_options(-fsanitize=undefined)
endif ()
if (USE_ASAN)
  add_compile_options(-fsanitize=address -fsanitize-address-use-after-scope
                      -fno-omit-frame-pointer -g)
  add_link_options(-fsanitize=address)
endif ()
if (USE_TSAN)
  add_compile_options(-fsanitize=thread -fno-omit-frame-pointer -g)
  add_link_options(-fsanitize=thread)
endif ()

if (USE_CLANG_TIDY)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif ()

if (ENABLE_BENCHMARKING)
  find_package(benchmark REQUIRED)
endif ()

if (CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  add_compile_options(-fprofile-arcs -ftest-coverage -O0 -g)
  add_link_options(-fprofile-arcs)
endif ()

# Libraries
cs_add_library(
  ${PROJECT_NAME}
  src/data_structure/volumetric_data_structure.cc
  src/integrator/pointcloud_integrator.cc
  src/integrator/measurement_model/beam_model.cc
  src/integrator/measurement_model/fixed_logodds_model.cc
  src/integrator/measurement_model/measurement_model.cc
  src/integrator/scan_integrator/coarse_to_fine/coarse_to_fine_integrator.cc
  src/integrator/scan_integrator/fixed_resolution/fixed_resolution_integrator.cc
  src/utils/eigen_format.cc
  src/utils/evaluation_utils.cc)

# Binaries
cs_add_executable(carmen_processor app/carmen_processor.cc)
target_link_libraries(carmen_processor ${PROJECT_NAME})

cs_add_executable(offline_compression_evaluation
                  app/offline_compression_evaluation.cc)
target_link_libraries(offline_compression_evaluation ${PROJECT_NAME})

# Tests
if (CATKIN_ENABLE_TESTING)
  cs_add_library(testing_main test/gtest_main.cc NO_AUTO_EXPORT)

  catkin_add_gtest(
    test_cell_traits
    test/src/data_structure/volumetric/cell_types/test_cell_types.cc)
  target_include_directories(test_cell_traits PRIVATE test/include)
  target_link_libraries(test_cell_traits ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_dense_grid
                   test/src/data_structure/volumetric/test_dense_grid.cc)
  target_include_directories(test_dense_grid PRIVATE test/include)
  target_link_libraries(test_dense_grid ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_hashed_blocks
                   test/src/data_structure/volumetric/test_hashed_blocks.cc)
  target_include_directories(test_hashed_blocks PRIVATE test/include)
  target_link_libraries(test_hashed_blocks ${PROJECT_NAME} testing_main)

  catkin_add_gtest(
    test_volumetric_data_structure
    test/src/data_structure/volumetric/test_volumetric_data_structure.cc)
  target_include_directories(test_volumetric_data_structure
                             PRIVATE test/include)
  target_link_libraries(test_volumetric_data_structure ${PROJECT_NAME}
                        testing_main)

  catkin_add_gtest(test_pointcloud
                   test/src/data_structure/generic/test_pointcloud.cc)
  target_include_directories(test_pointcloud PRIVATE test/include)
  target_link_libraries(test_pointcloud ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_range_image test/src/integrator/test_range_image.cc)
  target_include_directories(test_range_image PRIVATE test/include)
  target_link_libraries(test_range_image ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_scalar_quadtree
                   test/src/data_structure/volumetric/test_scalar_quadtree.cc)
  target_include_directories(test_scalar_quadtree PRIVATE test/include)
  target_link_libraries(test_scalar_quadtree ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_sparse_vector
                   test/src/data_structure/generic/test_sparse_vector.cc)
  target_include_directories(test_sparse_vector PRIVATE test/include)
  target_link_libraries(test_sparse_vector ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_index_conversions
                   test/src/indexing/test_index_conversions.cc)
  target_include_directories(test_index_conversions PRIVATE test/include)
  target_link_libraries(test_index_conversions ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_ndtree_index test/src/indexing/test_ndtree_index.cc)
  target_include_directories(test_ndtree_index PRIVATE test/include)
  target_link_libraries(test_ndtree_index ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_grid_iterator test/src/iterator/test_grid_iterator.cc)
  target_include_directories(test_grid_iterator PRIVATE test/include)
  target_link_libraries(test_grid_iterator ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_ray_iterator test/src/iterator/test_ray_iterator.cc)
  target_include_directories(test_ray_iterator PRIVATE test/include)
  target_link_libraries(test_ray_iterator ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_subtree_iterator
                   test/src/iterator/test_subtree_iterator.cc)
  target_include_directories(test_subtree_iterator PRIVATE test/include)
  target_link_libraries(test_subtree_iterator ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_pointcloud_integrators
                   test/src/integrator/test_pointcloud_integrators.cc)
  target_include_directories(test_pointcloud_integrators PRIVATE test/include)
  target_link_libraries(test_pointcloud_integrators ${PROJECT_NAME}
                        testing_main)

  catkin_add_gtest(test_coarse_to_fine_integrator
                   test/src/integrator/test_coarse_to_fine_integrator.cc)
  target_include_directories(test_coarse_to_fine_integrator
                             PRIVATE test/include)
  target_link_libraries(test_coarse_to_fine_integrator ${PROJECT_NAME}
                        testing_main)

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
endif ()

# Benchmarks
if (ENABLE_BENCHMARKING)
  cs_add_library(benchmarking_main benchmark/google_benchmark_main.cc)
  target_link_libraries(benchmarking_main benchmark::benchmark)

  add_executable(benchmark_measurement_integrator
                 benchmark/benchmark_measurement_integrator.cc)
  target_link_libraries(benchmark_measurement_integrator ${PROJECT_NAME}
                        benchmarking_main)

  add_executable(benchmark_sparse_vector benchmark/benchmark_sparse_vector.cc)
  target_link_libraries(benchmark_sparse_vector ${PROJECT_NAME}
                        benchmarking_main)
endif ()

# Export
cs_install()
cs_export()
