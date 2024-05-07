add_library(wavemap_core)
add_library(wavemap::wavemap_core ALIAS wavemap_core)

add_wavemap_include_directories(wavemap_core)
target_sources(wavemap_core PRIVATE
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
    utils/query/classified_map.cc
    utils/query/query_accelerator.cc
    utils/query/point_sampler.cc
    utils/sdf/full_euclidean_sdf_generator.cc
    utils/sdf/quasi_euclidean_sdf_generator.cc
    utils/time/stopwatch.cc
    utils/thread_pool.cc)

set_wavemap_target_properties(wavemap_core)
target_link_libraries(wavemap_core PUBLIC Eigen3::Eigen glog)
if (tracy_FOUND)
  target_link_libraries(wavemap_core PUBLIC TracyClient)
endif ()
