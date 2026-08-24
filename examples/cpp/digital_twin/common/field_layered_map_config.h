#ifndef WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_FIELD_LAYERED_MAP_CONFIG_H_
#define WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_FIELD_LAYERED_MAP_CONFIG_H_

#include <wavemap/layered/descriptor_layered_map_io.h>
#include <wavemap/layered/layered_map.h>
#include <wavemap/layered/schema_continuous_traits.h>
#include <wavemap/layered/schema_layer_storage.h>

#include "field_layer_policies.h"

namespace wavemap::examples::field_map {

using ContinuousLayers = layered::schema::ContinuousLayerBundle<Schema>;
using ContinuousArithmetic =
    layered::schema::ContinuousBundleArithmetic<Schema>;
using DiscreteLayers = layered::schema::DiscreteLayerBundle<Schema>;

using Map =
    layered::LayeredMap<ContinuousLayers, ContinuousArithmetic, DiscreteLayers>;
using Config = Map::Config;
using Voxel = Map::ContinuousVoxel;
using ContinuousMap = Map::ContinuousMap;

using VoxelSerializer = layered::DescriptorVoxelSerializer<Voxel>;
using MapIo = layered::io::LayeredMapIo<Map, VoxelSerializer>;

inline ContinuousLayers makeContinuousLayers(float reflectivity) {
  ContinuousLayers layers;
  layers.get<ReflectivityLayer>() = reflectivity;
  return layers;
}

inline Config makeDefaultConfig() {
  Config config;
  config.continuous_map.min_cell_width = 0.25f;
  config.continuous_map.min_log_odds = -2.f;
  config.continuous_map.max_log_odds = 4.f;
  config.continuous_map.tree_height = 6;
  config.continuous_map.only_prune_blocks_if_unused_for = 5.f;

  auto& reflectivity_threshold =
      config.continuous_threshold.data.get<ReflectivityLayer>();
  reflectivity_threshold.min = 0.f;
  reflectivity_threshold.max = 0.4f;

  config.continuous_pruning.occupancy.scale = 1e-3f;
  config.continuous_pruning.occupancy.weight = 1.f;
  auto& reflectivity_pruning =
      config.continuous_pruning.data.get<ReflectivityLayer>();
  reflectivity_pruning.scale = 1e-3f;
  reflectivity_pruning.weight = 1.f;
  config.continuous_pruning.combined_threshold = 1.f;
  config.continuous_pruning.data.combined_threshold = 1.f;

  config.discrete_compression.block_height = 1;
  return config;
}

}  // namespace wavemap::examples::field_map

#endif  // WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_FIELD_LAYERED_MAP_CONFIG_H_
