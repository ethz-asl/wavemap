#ifndef WAVEMAP_ROS_APP_OUSTER_LAYERED_RGB_MAP_LAYERS_H_
#define WAVEMAP_ROS_APP_OUSTER_LAYERED_RGB_MAP_LAYERS_H_

#include <string_view>

#include <wavemap/layered/integration/layer_update_policy.h>
#include <wavemap/layered/map/layered_map_config_builder.h>
#include <wavemap/layered/map/layered_map_definition.h>
#include <wavemap/layered/schema/layer_schema.h>
#include <wavemap/layered/types/rgb.h>

namespace wavemap::examples::rgb_map {

struct RgbLayer
    : layered::schema::ContinuousLayer<
          layered::Rgb,
          layered::ReplaceLayerUpdatePolicy<layered::Rgb>> {
  static constexpr std::string_view name = "rgb";
};

using Schema = layered::schema::LayerSchema<RgbLayer>;
using Definition = layered::LayeredMapDefinition<Schema>;

inline Definition::Config makeMapConfig() {
  layered::LayeredMapConfigBuilder<Definition> config;
  config.map()
      .minCellWidth(0.25f)
      .occupancyLogOdds(-2.f, 4.f)
      .treeHeight(7)
      .onlyPruneBlocksIfUnusedFor(5.f);
  config.occupancy().pruningScale(1e-3f).pruningWeight(1.f);
  config.layer<RgbLayer>()
      .storageBounds(layered::Rgb{0.f, 0.f, 0.f},
                     layered::Rgb{1.f, 1.f, 1.f})
      .pruningScale(1e-3f)
      .pruningWeight(1.f);
  config.combinedPruningThreshold(1.f);
  return config.build();
}

}  // namespace wavemap::examples::rgb_map

#endif  // WAVEMAP_ROS_APP_OUSTER_LAYERED_RGB_MAP_LAYERS_H_
