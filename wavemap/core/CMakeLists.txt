add_library(core STATIC)
add_library(wavemap::core ALIAS core)
target_sources(core PRIVATE
    src/tmp_check_3rdparty.cc
    src/config/value_with_unit.cc
    src/data_structure/volumetric/hashed_blocks.cc
    src/data_structure/volumetric/hashed_chunked_wavelet_octree.cc
    src/data_structure/volumetric/hashed_chunked_wavelet_octree_block.cc
    src/data_structure/volumetric/hashed_wavelet_octree.cc
    src/data_structure/volumetric/hashed_wavelet_octree_block.cc
    src/data_structure/volumetric/volumetric_octree.cc
    src/data_structure/volumetric/wavelet_octree.cc
    src/data_structure/volumetric/volumetric_data_structure_base.cc
    src/data_structure/volumetric/volumetric_data_structure_factory.cc
    src/integrator/measurement_model/continuous_beam.cc
    src/integrator/measurement_model/constant_ray.cc
    src/integrator/measurement_model/continuous_ray.cc
    src/integrator/measurement_model/measurement_model_factory.cc
    src/integrator/projection_model/circular_projector.cc
    src/integrator/projection_model/ouster_projector.cc
    src/integrator/projection_model/pinhole_camera_projector.cc
    src/integrator/projection_model/spherical_projector.cc
    src/integrator/projection_model/projector_factory.cc
    src/integrator/projective/coarse_to_fine/coarse_to_fine_integrator.cc
    src/integrator/projective/coarse_to_fine/hashed_chunked_wavelet_integrator.cc
    src/integrator/projective/coarse_to_fine/hashed_wavelet_integrator.cc
    src/integrator/projective/coarse_to_fine/wavelet_integrator.cc
    src/integrator/projective/fixed_resolution/fixed_resolution_integrator.cc
    src/integrator/projective/projective_integrator.cc
    src/integrator/ray_tracing/ray_tracing_integrator.cc
    src/integrator/integrator_base.cc
    src/integrator/integrator_factory.cc
    src/utils/stopwatch.cc
    src/utils/thread_pool.cc
)
target_include_directories(core PRIVATE include)
target_link_libraries(core PUBLIC Eigen3::Eigen glog::glog TracyClient)
set_global_target_properties(core)