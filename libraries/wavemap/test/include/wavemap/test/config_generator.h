#ifndef WAVEMAP_TEST_CONFIG_GENERATOR_H_
#define WAVEMAP_TEST_CONFIG_GENERATOR_H_

#include <utility>

#include "wavemap/common.h"
#include "wavemap/data_structure/volumetric/hashed_chunked_wavelet_octree.h"
#include "wavemap/data_structure/volumetric/hashed_wavelet_octree.h"
#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"
#include "wavemap/data_structure/volumetric/volumetric_octree.h"
#include "wavemap/data_structure/volumetric/wavelet_octree.h"
#include "wavemap/integrator/measurement_model/continuous_beam.h"
#include "wavemap/integrator/projection_model/spherical_projector.h"
#include "wavemap/integrator/projective/projective_integrator.h"
#include "wavemap/integrator/ray_tracing/ray_tracing_integrator.h"
#include "wavemap/utils/random_number_generator.h"

namespace wavemap {
namespace detail {
// Define the generic template that we will specialize for each config type
template <typename ConfigT, typename... Args>
ConfigT getRandomConfig(RandomNumberGenerator&, Args...);

// Volumetric data structure configs
template <>
inline VolumetricDataStructureConfig
getRandomConfig<VolumetricDataStructureConfig>(RandomNumberGenerator& rng) {
  VolumetricDataStructureConfig config;
  config.min_cell_width = rng.getRandomRealNumber(0.05f, 1.f);
  config.min_log_odds = rng.getRandomRealNumber(-10.f, 0.f);
  config.max_log_odds = rng.getRandomRealNumber(0.f, 10.f);
  return config;
}

template <>
inline VolumetricOctreeConfig getRandomConfig<VolumetricOctreeConfig>(
    RandomNumberGenerator& rng) {
  VolumetricOctreeConfig config;
  config.min_cell_width = rng.getRandomRealNumber(0.05f, 1.f);
  config.min_log_odds = rng.getRandomRealNumber(-10.f, 0.f);
  config.max_log_odds = rng.getRandomRealNumber(0.f, 10.f);
  config.tree_height = rng.getRandomInteger(13, 16);
  return config;
}

template <>
inline WaveletOctreeConfig getRandomConfig<WaveletOctreeConfig>(
    RandomNumberGenerator& rng) {
  WaveletOctreeConfig config;
  config.min_cell_width = rng.getRandomRealNumber(0.05f, 1.f);
  config.min_log_odds = rng.getRandomRealNumber(-10.f, 0.f);
  config.max_log_odds = rng.getRandomRealNumber(0.f, 10.f);
  config.tree_height = rng.getRandomInteger(13, 16);
  return config;
}

template <>
inline HashedWaveletOctreeConfig getRandomConfig<HashedWaveletOctreeConfig>(
    RandomNumberGenerator& rng) {
  HashedWaveletOctreeConfig config;
  config.min_cell_width = rng.getRandomRealNumber(0.05f, 1.f);
  config.min_log_odds = rng.getRandomRealNumber(-10.f, 0.f);
  config.max_log_odds = rng.getRandomRealNumber(0.f, 10.f);
  config.tree_height = rng.getRandomInteger(2, 14);
  return config;
}

template <>
inline HashedChunkedWaveletOctreeConfig
getRandomConfig<HashedChunkedWaveletOctreeConfig>(RandomNumberGenerator& rng) {
  HashedChunkedWaveletOctreeConfig config;
  config.min_cell_width = rng.getRandomRealNumber(0.05f, 1.f);
  config.min_log_odds = rng.getRandomRealNumber(-10.f, 0.f);
  config.max_log_odds = rng.getRandomRealNumber(0.f, 10.f);
  config.tree_height = rng.getRandomInteger(1, 3) *
                       HashedChunkedWaveletOctreeBlock::kChunkHeight;
  return config;
}

// Projection model configs
template <>
inline SphericalProjectorConfig getRandomConfig<SphericalProjectorConfig>(
    RandomNumberGenerator& rng) {
  SphericalProjectorConfig config;
  config.elevation.min_angle = rng.getRandomRealNumber(-kQuarterPi, 0.f);
  config.elevation.max_angle = rng.getRandomRealNumber(
      config.elevation.min_angle + kPi / 8.f, kQuarterPi);
  config.azimuth.min_angle = -kPi;
  config.azimuth.max_angle = kPi;
  config.elevation.num_cells = int_math::exp2(rng.getRandomInteger(4, 6));
  config.azimuth.num_cells = int_math::exp2(rng.getRandomInteger(7, 10));
  return config;
}

// Measurement model configs
template <>
inline ContinuousBeamConfig getRandomConfig<ContinuousBeamConfig>(
    RandomNumberGenerator& rng, SphericalProjector projection_model) {
  ContinuousBeamConfig config;
  const FloatingPoint max_angle_sigma_without_overlap =
      (projection_model.getMaxImageCoordinates() -
       projection_model.getMinImageCoordinates())
          .cwiseQuotient(projection_model.getDimensions().cast<FloatingPoint>())
          .minCoeff() /
      (2.f * 6.f);
  config.angle_sigma = rng.getRandomRealNumber(
      max_angle_sigma_without_overlap / 10.f, max_angle_sigma_without_overlap);
  config.range_sigma = rng.getRandomRealNumber(5e-3f, 1e-1f);
  return config;
}

// Integrator configs
template <>
inline RayTracingIntegratorConfig getRandomConfig<RayTracingIntegratorConfig>(
    RandomNumberGenerator& rng) {
  RayTracingIntegratorConfig config;
  config.min_range = rng.getRandomRealNumber(0.2f, 3.f);
  config.max_range = rng.getRandomRealNumber(
      static_cast<FloatingPoint>(config.min_range), 20.f);
  return config;
}

template <>
inline ProjectiveIntegratorConfig getRandomConfig<ProjectiveIntegratorConfig>(
    RandomNumberGenerator& rng) {
  ProjectiveIntegratorConfig config;
  config.min_range = rng.getRandomRealNumber(0.2f, 3.f);
  config.max_range = rng.getRandomRealNumber(
      static_cast<FloatingPoint>(config.min_range), 20.f);
  return config;
}
}  // namespace detail

class ConfigGenerator {
 public:
  explicit ConfigGenerator(size_t random_seed = 0u)
      : random_number_generator_(random_seed) {}

  // Forward the config requests to the specialized templates defined above
  // NOTE: The methods cannot be defined directly inside this class since
  //       explicit template specialization is not allowed in non-namespace
  //       scopes (e.g. inside a class).
  template <typename T, typename... Args>
  T getRandomConfig(Args... args) {
    return detail::getRandomConfig<T>(random_number_generator_,
                                      std::forward<Args>(args)...);
  }

 private:
  RandomNumberGenerator random_number_generator_;
};
}  // namespace wavemap

#endif  // WAVEMAP_TEST_CONFIG_GENERATOR_H_
