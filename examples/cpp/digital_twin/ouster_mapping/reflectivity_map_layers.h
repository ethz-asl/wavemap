#ifndef WAVEMAP_EXAMPLES_DIGITAL_TWIN_OUSTER_REFLECTIVITY_MAP_LAYERS_H_
#define WAVEMAP_EXAMPLES_DIGITAL_TWIN_OUSTER_REFLECTIVITY_MAP_LAYERS_H_

#include <array>
#include <string_view>

#include <wavemap/layered/layer_schema.h>
#include <wavemap/layered/layer_update_policy.h>
#include <wavemap/layered/layered_map_config_builder.h>
#include <wavemap/layered/layered_map_definition.h>

namespace wavemap::examples::reflectivity_map {

using ReflectivityReplacePolicy = layered::ReplaceLayerUpdatePolicy<float>;
using ReflectivityExponentialPolicy =
    layered::ExponentialScalarLayerUpdatePolicy<1, 5>;

template <typename UpdatePolicyT>
struct ReflectivityLayerTemplate
    : layered::schema::ContinuousLayer<float, UpdatePolicyT> {
  static constexpr layered::ScalarLayerVisualization visualization{
      {0.05f, 0.05f, 0.05f}, {1.f, 1.f, 1.f}};
  static constexpr std::string_view name = "reflectivity";
};

// Both policies are user-selectable. This example uses alpha=0.2 exponential
// smoothing; changing this alias to ReflectivityReplacePolicy selects latest
// observation replacement without changing the adapter or pipeline.
using ReflectivityLayer =
    ReflectivityLayerTemplate<ReflectivityExponentialPolicy>;

struct ClassLayer : layered::schema::DiscreteLayer<int, layered::ReplaceLayerUpdatePolicy<int>> {
  static constexpr std::string_view name = "class";
  static constexpr std::array categories{
      layered::IntegerCategoryLabel{1, "Ground", {0.2f, 0.8f, 0.2f}},
      layered::IntegerCategoryLabel{2, "Obstacle", {0.9f, 0.2f, 0.15f}}};
};

// Occupancy is built into the shared continuous voxel and is not repeated in
// the user schema.
using Schema = layered::schema::LayerSchema<ReflectivityLayer, ClassLayer>;
using Definition = layered::LayeredMapDefinition<Schema>;

inline Definition::Config makeMapConfig() {
  layered::LayeredMapConfigBuilder<Definition> config;
  config.map()
      .minCellWidth(0.25f)
      .occupancyLogOdds(-2.f, 4.f)
      .treeHeight(7)
      .onlyPruneBlocksIfUnusedFor(5.f);
  config.occupancy().pruningScale(1e-3f).pruningWeight(1.f);
  config.layer<ReflectivityLayer>()
      .storageBounds(0.f, 1.f)
      .pruningScale(1e-3f)
      .pruningWeight(1.f);
  config.discreteBlockHeight(0);
  config.combinedPruningThreshold(1.f);
  return config.build();
}

}  // namespace wavemap::examples::reflectivity_map

#endif  // WAVEMAP_EXAMPLES_DIGITAL_TWIN_OUSTER_REFLECTIVITY_MAP_LAYERS_H_
