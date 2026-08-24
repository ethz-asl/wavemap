#ifndef WAVEMAP_LAYERED_LAYERED_PIPELINE_H_
#define WAVEMAP_LAYERED_LAYERED_PIPELINE_H_

#include <vector>

#include <wavemap/layered/layer_integrator.h>
#include <wavemap/layered/layer_observation_batch.h>

namespace wavemap::layered {

// A typed, sensor-independent input path for additional map layers. It is
// intentionally independent of wavemap::Pipeline so the original occupancy
// pipeline remains unchanged.
template <typename LayeredMapT>
class LayeredPipeline {
 public:
  explicit LayeredPipeline(LayeredMapT& map) : map_(map) {}

  template <typename LayerTagT>
  LayerIntegrationResult integrate(
      const std::vector<LayerObservation<LayerValueT<LayerTagT>>>&
          observations) {
    return integrateLayerObservations<LayerTagT>(map_, observations);
  }

  template <typename LayerTagT>
  LayerIntegrationResult integrate(
      const std::vector<LayerObservation<LayerValueT<LayerTagT>>>& observations,
      const LayerUpdatePolicyT<LayerTagT>& policy) {
    return integrateLayerObservations<LayerTagT>(map_, observations, policy);
  }

  template <typename ObservationBatchT>
  LayerIntegrationResult integrateBatch(const ObservationBatchT& batch) {
    LayerIntegrationResult total;
    batch.forEachLayer([&](auto layer_tag, const auto& observations) {
      using LayerTag = typename decltype(layer_tag)::Type;
      if (observations.empty()) {
        return;
      }
      const auto result = integrate<LayerTag>(observations);
      total.received += result.received;
      total.integrated += result.integrated;
      total.rejected += result.rejected;
      total.updated_voxels += result.updated_voxels;
    });
    return total;
  }

 private:
  LayeredMapT& map_;
};

}  // namespace wavemap::layered

#endif  // WAVEMAP_LAYERED_LAYERED_PIPELINE_H_
