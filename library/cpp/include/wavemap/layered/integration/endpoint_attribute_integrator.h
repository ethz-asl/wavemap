#ifndef WAVEMAP_LAYERED_ENDPOINT_ATTRIBUTE_INTEGRATOR_H_
#define WAVEMAP_LAYERED_ENDPOINT_ATTRIBUTE_INTEGRATOR_H_

#include <cstddef>
#include <utility>

#include <wavemap/core/indexing/index_conversions.h>
#include <wavemap/core/integrator/integrator_base.h>
#include <wavemap/layered/integration/continuous_layer_updater.h>
#include <wavemap/layered/integration/endpoint_attribute_pointcloud.h>
#include <wavemap/layered/integration/layer_integrator.h>
#include <wavemap/layered/integration/layer_observation.h>
#include <wavemap/layered/schema/layer_traits.h>

namespace wavemap::layered {

struct EndpointAttributeIntegrationResult {
  size_t received = 0u;
  size_t integrated = 0u;
  size_t rejected = 0u;
};

// Adds endpoint-only continuous fields around an existing occupancy integrator.
// Occupancy remains entirely owned by the wrapped Wavemap integrator.
template <typename LayeredMapT, typename... EndpointLayerTags>
class EndpointAttributeIntegrator {
 public:
  using Measurement =
      PosedEndpointAttributePointcloud<EndpointLayerTags...>;

  EndpointAttributeIntegrator(LayeredMapT& map,
                              IntegratorBase& occupancy_integrator)
      : map_(map), occupancy_integrator_(occupancy_integrator) {
    static_assert((kIsWaveletCompatibleLayer<EndpointLayerTags> && ...),
                  "Endpoint sensor fields must be continuous layers.");
  }

  EndpointAttributeIntegrationResult integrate(const Measurement& measurement) {
    EndpointAttributeIntegrationResult result;
    result.received = measurement.size();
    if (!measurement.hasConsistentSizes()) {
      result.rejected = result.received;
      return result;
    }

    // Preserve the original occupancy path and its measurement model exactly.
    const PosedPointcloud<> occupancy_measurement(measurement.getPose(),
                                                   measurement.points().data());
    occupancy_integrator_.integrate(occupancy_measurement);

    const FloatingPoint cell_width_inv =
        1.f / map_.continuousMap().getMinCellWidth();
    for (size_t point_idx = 0u; point_idx < measurement.size(); ++point_idx) {
      const Point3D C_endpoint = measurement.point(point_idx);
      if (!C_endpoint.array().isFinite().all()) {
        ++result.rejected;
        continue;
      }
      const Point3D W_endpoint = measurement.getPose() * C_endpoint;
      const Index3D endpoint_index =
          convert::pointToNearestIndex(W_endpoint, cell_width_inv);
      updateContinuousLayersAtVoxel(
          map_.continuousMap(), endpoint_index, [&](auto& layers) {
            (updateLayer<EndpointLayerTags>(layers, measurement, point_idx,
                                            W_endpoint),
             ...);
          });
      ++result.integrated;
    }
    return result;
  }

 private:
  template <typename LayerTagT, typename ContinuousLayersT>
  static void updateLayer(ContinuousLayersT& layers,
                          const Measurement& measurement, size_t point_idx,
                          const Point3D& W_endpoint) {
    constexpr auto descriptor =
        detail::descriptorForLayer<LayerTagT, ContinuousLayersT>();
    auto& current_value = descriptor.value(layers);
    const LayerObservation<LayerValueT<LayerTagT>> observation(
        W_endpoint, measurement.template attribute<LayerTagT>(point_idx));
    current_value = LayerUpdatePolicyT<LayerTagT>{}(current_value, observation);
  }

  LayeredMapT& map_;
  IntegratorBase& occupancy_integrator_;
};

}  // namespace wavemap::layered

#endif  // WAVEMAP_LAYERED_ENDPOINT_ATTRIBUTE_INTEGRATOR_H_
