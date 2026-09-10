#ifndef WAVEMAP_ROS_LAYERED_HASHED_WAVELET_INTEGRATOR_FACTORY_H_
#define WAVEMAP_ROS_LAYERED_HASHED_WAVELET_INTEGRATOR_FACTORY_H_

#include <memory>
#include <utility>

#include <glog/logging.h>
#include <wavemap/core/integrator/integrator_base.h>
#include <wavemap/core/integrator/measurement_model/measurement_model_factory.h>
#include <wavemap/core/integrator/projective/coarse_to_fine/hashed_chunked_wavelet_integrator.h>
#include <wavemap/core/integrator/projective/coarse_to_fine/hashed_wavelet_integrator.h>
#include <wavemap/core/integrator/projective/projective_integrator.h>
#include <wavemap/core/integrator/projection_model/projector_factory.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/utils/thread_pool.h>

namespace wavemap {

// Typed counterpart of the original runtime IntegratorFactory boundary. The
// original factory remains unchanged for occupancy-only maps; layered servers
// use this path because their voxel type is known at compile time.
namespace detail {
template <typename IntegratorT, typename MapT>
std::unique_ptr<IntegratorBase> createLayeredWaveletIntegratorImpl(
    const param::Value& params,
    typename MapT::Ptr occupancy_map, IntegratorType expected_type,
    const char* expected_type_name,
    std::shared_ptr<ThreadPool> thread_pool) {
  const auto type = IntegratorType::from(params, "integration_method");
  if (!type || type.value() != expected_type) {
    LOG(ERROR) << "This layered continuous-map backend requires "
                  "integration_method type "
               << expected_type_name << ".";
    return nullptr;
  }

  const auto integrator_config =
      ProjectiveIntegratorConfig::from(params, "integration_method");
  if (!integrator_config) {
    LOG(ERROR) << "Layered integrator config could not be loaded.";
    return nullptr;
  }

  std::shared_ptr<ProjectorBase> projection_model =
      ProjectorFactory::create(params);
  if (!projection_model) {
    LOG(ERROR) << "Layered integrator projection model could not be created.";
    return nullptr;
  }

  auto posed_range_image =
      std::make_shared<PosedImage<>>(projection_model->getDimensions());
  auto beam_offset_image =
      std::make_shared<Image<Vector2D>>(projection_model->getDimensions());
  auto measurement_model = MeasurementModelFactory::create(
      params, projection_model, posed_range_image, beam_offset_image);
  if (!measurement_model) {
    LOG(ERROR) << "Layered integrator measurement model could not be created.";
    return nullptr;
  }

  return std::make_unique<IntegratorT>(
      integrator_config.value(), std::move(projection_model),
      std::move(posed_range_image), std::move(beam_offset_image),
      std::move(measurement_model), std::move(occupancy_map),
      std::move(thread_pool));
}
}  // namespace detail

template <typename CellDataT>
std::unique_ptr<IntegratorBase> createLayeredWaveletIntegrator(
    const param::Value& params,
    typename HashedWaveletOctreeT<CellDataT>::Ptr occupancy_map,
    std::shared_ptr<ThreadPool> thread_pool) {
  return detail::createLayeredWaveletIntegratorImpl<
      HashedWaveletIntegratorT<CellDataT>,
      HashedWaveletOctreeT<CellDataT>>(
      params, std::move(occupancy_map),
      IntegratorType::kHashedWaveletIntegrator,
      "hashed_wavelet_integrator", std::move(thread_pool));
}

template <typename CellDataT>
std::unique_ptr<IntegratorBase> createLayeredWaveletIntegrator(
    const param::Value& params,
    typename HashedChunkedWaveletOctreeT<CellDataT>::Ptr occupancy_map,
    std::shared_ptr<ThreadPool> thread_pool) {
  return detail::createLayeredWaveletIntegratorImpl<
      HashedChunkedWaveletIntegratorT<CellDataT>,
      HashedChunkedWaveletOctreeT<CellDataT>>(
      params, std::move(occupancy_map),
      IntegratorType::kHashedChunkedWaveletIntegrator,
      "hashed_chunked_wavelet_integrator", std::move(thread_pool));
}

}  // namespace wavemap

#endif  // WAVEMAP_ROS_LAYERED_HASHED_WAVELET_INTEGRATOR_FACTORY_H_
