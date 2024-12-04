add_executable(test_wavemap_core)

target_include_directories(test_wavemap_core PRIVATE
    ${PROJECT_SOURCE_DIR}/test/include)
target_sources(test_wavemap_core PRIVATE
    data_structure/test_aabb.cc
    data_structure/test_image.cc
    data_structure/test_ndtree_conversions.cc
    data_structure/test_ndtree.cc
    data_structure/test_pointcloud.cc
    data_structure/test_sparse_vector.cc
    indexing/test_index_conversions.cc
    indexing/test_ndtree_index.cc
    integrator/projection_model/test_circular_projector.cc
    integrator/projection_model/test_image_projectors.cc
    integrator/projection_model/test_spherical_projector.cc
    integrator/test_hierarchical_range_image.cc
    integrator/test_measurement_models.cc
    integrator/test_pointcloud_integrators.cc
    integrator/test_range_image_intersector.cc
    map/test_haar_cell.cc
    map/test_hashed_blocks.cc
    map/test_map.cc
    map/test_volumetric_octree.cc
    utils/bits/test_bit_operations.cc
    utils/data/test_comparisons.cc
    utils/data/test_fill.cc
    utils/iterate/test_grid_iterator.cc
    utils/iterate/test_ray_iterator.cc
    utils/iterate/test_subtree_iterator.cc
    utils/math/test_approximate_trigonometry.cc
    utils/math/test_int_math.cc
    utils/math/test_tree_math.cc
    utils/neighbors/test_adjacency.cc
    utils/neighbors/test_grid_adjacency.cc
    utils/neighbors/test_grid_neighborhood.cc
    utils/neighbors/test_ndtree_adjacency.cc
    utils/profile/test_resource_monitor.cc
    utils/query/test_classified_map.cc
    utils/query/test_map_interpolator.cpp
    utils/query/test_occupancy_classifier.cc
    utils/query/test_probability_conversions.cc
    utils/query/test_query_accelerator.cc
    utils/sdf/test_sdf_generators.cc
    utils/time/test_stopwatch.cc
    utils/test_thread_pool.cc)

set_wavemap_target_properties(test_wavemap_core)
target_link_libraries(test_wavemap_core wavemap_core GTest::gtest_main)

gtest_discover_tests(test_wavemap_core)
