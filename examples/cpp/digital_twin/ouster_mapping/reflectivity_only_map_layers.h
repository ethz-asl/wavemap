#ifndef WAVEMAP_EXAMPLES_DIGITAL_TWIN_OUSTER_REFLECTIVITY_ONLY_MAP_LAYERS_H_
#define WAVEMAP_EXAMPLES_DIGITAL_TWIN_OUSTER_REFLECTIVITY_ONLY_MAP_LAYERS_H_

#include "reflectivity_map_layers.h"

namespace wavemap::examples::reflectivity_only_map {

using ReflectivityLayer = reflectivity_map::ReflectivityLayer;
using Schema = layered::schema::LayerSchema<ReflectivityLayer>;
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
  config.combinedPruningThreshold(1.f);
  return config.build();
}

}  // namespace wavemap::examples::reflectivity_only_map

#endif  // WAVEMAP_EXAMPLES_DIGITAL_TWIN_OUSTER_REFLECTIVITY_ONLY_MAP_LAYERS_H_
