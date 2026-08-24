#ifndef WAVEMAP_EXAMPLES_DIGITAL_TWIN_OUSTER_OCCUPANCY_MAP_LAYERS_H_
#define WAVEMAP_EXAMPLES_DIGITAL_TWIN_OUSTER_OCCUPANCY_MAP_LAYERS_H_

#include <wavemap/layered/layer_schema.h>
#include <wavemap/layered/layered_map_config_builder.h>
#include <wavemap/layered/layered_map_definition.h>

namespace wavemap::examples::occupancy_map {

// Occupancy is built into every layered map, so this schema intentionally has
// no user-defined layers. It is benchmark variant B.
using Schema = layered::schema::LayerSchema<>;
using Definition = layered::LayeredMapDefinition<Schema>;

inline Definition::Config makeMapConfig() {
  layered::LayeredMapConfigBuilder<Definition> config;
  config.map()
      .minCellWidth(0.25f)
      .occupancyLogOdds(-2.f, 4.f)
      .treeHeight(7)
      .onlyPruneBlocksIfUnusedFor(5.f);
  config.occupancy().pruningScale(1e-3f).pruningWeight(1.f);
  config.combinedPruningThreshold(1.f);
  return config.build();
}

}  // namespace wavemap::examples::occupancy_map

#endif  // WAVEMAP_EXAMPLES_DIGITAL_TWIN_OUSTER_OCCUPANCY_MAP_LAYERS_H_
