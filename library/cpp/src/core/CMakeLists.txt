# Declare the wavemap core library
add_library(wavemap_core)
add_library(wavemap::wavemap_core ALIAS wavemap_core)

# Configure target properties, include directories and linking
set_wavemap_target_properties(wavemap_core)
add_wavemap_include_directories(wavemap_core)
target_link_libraries(wavemap_core
    PUBLIC Eigen3::Eigen glog Boost::preprocessor)
if (tracy_FOUND)
  target_link_libraries(wavemap_core PUBLIC TracyClient)
endif ()

# Set sources
target_sources(wavemap_core PRIVATE
    config/string_list.cc
    config/value_with_unit.cc
    integrator/measurement_model/continuous_beam.cc
    integrator/measurement_model/constant_ray.cc
    integrator/measurement_model/continuous_ray.cc
    integrator/measurement_model/measurement_model_factory.cc
    integrator/projection_model/circular_projector.cc
    integrator/projection_model/ouster_projector.cc
    integrator/projection_model/pinhole_camera_projector.cc
    integrator/projection_model/spherical_projector.cc
    integrator/projection_model/projector_factory.cc
    integrator/projective/coarse_to_fine/coarse_to_fine_integrator.cc
    integrator/projective/coarse_to_fine/hashed_chunked_wavelet_integrator.cc
    integrator/projective/coarse_to_fine/hashed_wavelet_integrator.cc
    integrator/projective/coarse_to_fine/wavelet_integrator.cc
    integrator/projective/fixed_resolution/fixed_resolution_integrator.cc
    integrator/projective/projective_integrator.cc
    integrator/ray_tracing/ray_tracing_integrator.cc
    integrator/integrator_base.cc
    integrator/integrator_factory.cc
    map/hashed_blocks.cc
    map/hashed_chunked_wavelet_octree.cc
    map/hashed_chunked_wavelet_octree_block.cc
    map/hashed_wavelet_octree.cc
    map/hashed_wavelet_octree_block.cc
    map/volumetric_octree.cc
    map/wavelet_octree.cc
    map/map_base.cc
    map/map_factory.cc
    utils/profile/resource_usage_monitor.cc
    utils/query/classified_map.cc
    utils/query/query_accelerator.cc
    utils/query/point_sampler.cc
    utils/sdf/full_euclidean_sdf_generator.cc
    utils/sdf/quasi_euclidean_sdf_generator.cc
    utils/time/stopwatch.cc
    utils/undistortion/pointcloud_undistortion.cc
    utils/undistortion/stamped_pointcloud.cc
    utils/logging_level.cc
    utils/thread_pool.cc)

# Support installs
if (GENERATE_WAVEMAP_INSTALL_RULES)
  # Mark headers for installation
  install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/wavemap/core/
      DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/wavemap/core
      FILES_MATCHING PATTERN "*.h")

  # Mark target for installation and add it to the CMake exports file
  install(TARGETS wavemap_core
      EXPORT wavemap-targets
      LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
      ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
      RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif ()
