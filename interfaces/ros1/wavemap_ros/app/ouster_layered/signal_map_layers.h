#ifndef WAVEMAP_EXAMPLES_OUSTER_SIGNAL_MAP_LAYERS_H_
#define WAVEMAP_EXAMPLES_OUSTER_SIGNAL_MAP_LAYERS_H_

#include <string_view>

#include "reflectivity_map_layers.h"

namespace wavemap::examples::signal_map {

using ReflectivityLayer = reflectivity_map::ReflectivityLayer;
using ClassLayer = reflectivity_map::ClassLayer;
using SignalPolicy = layered::ExponentialScalarLayerUpdatePolicy<1, 5>;

struct SignalLayer
    : layered::schema::ContinuousLayer<float, SignalPolicy> {
  static constexpr layered::ScalarLayerVisualization visualization{
      {0.02f, 0.04f, 0.20f}, {0.00f, 1.00f, 1.00f}};
  static constexpr std::string_view name = "signal";
};

struct NearIrLayer
    : layered::schema::ContinuousLayer<float, SignalPolicy> {
  static constexpr layered::ScalarLayerVisualization visualization{
      {0.08f, 0.00f, 0.15f}, {1.00f, 0.85f, 0.00f}};
  static constexpr std::string_view name = "near_ir";
};

// Occupancy remains built into every shared continuous voxel.
using Schema =
    layered::schema::LayerSchema<ReflectivityLayer, SignalLayer>;
using NearIrSchema = layered::schema::LayerSchema<
    ReflectivityLayer, SignalLayer, NearIrLayer>;
using ClassSchema = layered::schema::LayerSchema<
    ReflectivityLayer, SignalLayer, ClassLayer>;
using NearIrClassSchema = layered::schema::LayerSchema<
    ReflectivityLayer, SignalLayer, NearIrLayer, ClassLayer>;

template <typename SchemaT>
using DefinitionFor =
    layered::LayeredMapDefinition<SchemaT,
                                  layered::HashedWaveletOctreeBackend>;
using Definition = DefinitionFor<Schema>;
using NearIrDefinition = DefinitionFor<NearIrSchema>;
using ClassDefinition = DefinitionFor<ClassSchema>;
using NearIrClassDefinition = DefinitionFor<NearIrClassSchema>;

template <typename DefinitionT>
inline typename DefinitionT::Config makeMapConfigFor(
    [[maybe_unused]] int discrete_block_height = 0) {
  layered::LayeredMapConfigBuilder<DefinitionT> config;
  config.map()
      .minCellWidth(0.25f)
      .occupancyLogOdds(-2.f, 4.f)
      .treeHeight(7)
      .onlyPruneBlocksIfUnusedFor(5.f);
  config.occupancy().pruningScale(1e-3f).pruningWeight(1.f);
  config.template layer<ReflectivityLayer>()
      .storageBounds(0.f, 1.f)
      .pruningScale(1e-3f)
      .pruningWeight(1.f);
  config.template layer<SignalLayer>()
      .storageBounds(0.f, 1.f)
      .pruningScale(1e-3f)
      .pruningWeight(1.f);
  if constexpr (
      DefinitionT::Schema::template containsContinuous<NearIrLayer>) {
    config.template layer<NearIrLayer>()
        .storageBounds(0.f, 1.f)
        .pruningScale(1e-3f)
        .pruningWeight(1.f);
  }
  if constexpr (
      DefinitionT::Schema::template containsDiscrete<ClassLayer>) {
    config.discreteBlockHeight(discrete_block_height);
  }
  config.combinedPruningThreshold(1.f);
  return config.build();
}

inline Definition::Config makeMapConfig() {
  return makeMapConfigFor<Definition>();
}

}  // namespace wavemap::examples::signal_map

#endif  // WAVEMAP_EXAMPLES_OUSTER_SIGNAL_MAP_LAYERS_H_
