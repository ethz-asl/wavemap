#ifndef WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_CUSTOM_LAYERED_MAP_CONFIG_H_
#define WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_CUSTOM_LAYERED_MAP_CONFIG_H_

#include <algorithm>
#include <cmath>
#include <istream>
#include <ostream>

#include <wavemap/core/map/cell_types/cell_data_traits.h>
#include <wavemap/io/stream_conversions.h>

#include <wavemap/layered/layered_map.h>
#include <wavemap/layered/layered_map_io.h>

using wavemap::layered::DiscreteCompressionConfig;
using wavemap::layered::DiscreteLayer;
using wavemap::layered::LayeredMap;

struct MyContinuousLayers {
  float temperature = 0.f;
  float intensity = 0.f;

  bool operator==(const MyContinuousLayers& other) const {
    return temperature == other.temperature && intensity == other.intensity;
  }
};

struct MyContinuousPolicy {
  static MyContinuousLayers add(const MyContinuousLayers& lhs,
                                const MyContinuousLayers& rhs) {
    return {lhs.temperature + rhs.temperature, lhs.intensity + rhs.intensity};
  }

  static MyContinuousLayers subtract(const MyContinuousLayers& lhs,
                                     const MyContinuousLayers& rhs) {
    return {lhs.temperature - rhs.temperature, lhs.intensity - rhs.intensity};
  }

  static MyContinuousLayers scale(const MyContinuousLayers& data,
                                  wavemap::FloatingPoint factor) {
    return {factor * data.temperature, factor * data.intensity};
  }
};

namespace wavemap {
template <>
struct CellDataTraits<::MyContinuousLayers> {
  struct ThresholdConfig {
    FloatingPoint min_temperature = -50.f;
    FloatingPoint max_temperature = 80.f;
    FloatingPoint min_intensity = 0.f;
    FloatingPoint max_intensity = 1.f;
  };

  struct PruningConfig {
    FieldPruningConfig temperature{};
    FieldPruningConfig intensity{};
    FloatingPoint combined_threshold = 1.f;
  };

  static void threshold(MyContinuousLayers& layers,
                        const ThresholdConfig& config) {
    layers.temperature = std::clamp(layers.temperature,
                                    config.min_temperature,
                                    config.max_temperature);
    layers.intensity = std::clamp(layers.intensity, config.min_intensity,
                                  config.max_intensity);
  }

  static FloatingPoint pruningScore(const MyContinuousLayers& layers,
                                    const PruningConfig& config) {
    return weightedPruningScore(std::abs(layers.temperature),
                                config.temperature) +
           weightedPruningScore(std::abs(layers.intensity), config.intensity);
  }

  static bool isNonzero(const MyContinuousLayers& layers,
                        const PruningConfig& config) {
    return config.combined_threshold < pruningScore(layers, config);
  }
};
}  // namespace wavemap

struct MyDiscreteLayers {
  explicit MyDiscreteLayers(
      DiscreteCompressionConfig config = DiscreteCompressionConfig())
      : semantic_label(config), manually_changed(config) {}

  DiscreteLayer<int> semantic_label;
  DiscreteLayer<bool> manually_changed;
};

using MyLayeredMap =
    LayeredMap<MyContinuousLayers, MyContinuousPolicy, MyDiscreteLayers>;
using MyLayeredMapConfig = MyLayeredMap::Config;
using MyVoxel = MyLayeredMap::ContinuousVoxel;

struct MyVoxelSerializer {
  static void write(std::ostream& ostream, const MyVoxel& voxel) {
    wavemap::io::StreamableCellData<wavemap::FloatingPoint>::write(
        ostream, voxel.occupancy);
    wavemap::io::StreamableCellData<wavemap::FloatingPoint>::write(
        ostream, voxel.data.temperature);
    wavemap::io::StreamableCellData<wavemap::FloatingPoint>::write(
        ostream, voxel.data.intensity);
  }

  static MyVoxel read(std::istream& istream) {
    MyVoxel voxel;
    voxel.occupancy =
        wavemap::io::StreamableCellData<wavemap::FloatingPoint>::read(istream);
    voxel.data.temperature =
        wavemap::io::StreamableCellData<wavemap::FloatingPoint>::read(istream);
    voxel.data.intensity =
        wavemap::io::StreamableCellData<wavemap::FloatingPoint>::read(istream);
    return voxel;
  }
};

namespace wavemap::layered {
template <>
struct ContinuousLayerSchemaTraits<::MyContinuousLayers> {
  static std::vector<LayerSchemaEntry> layers() {
    return {{"temperature", "float32"}, {"intensity", "float32"}};
  }
};
}  // namespace wavemap::layered

namespace wavemap::layered::io {
template <>
struct DiscreteLayerBundleTraits<::MyDiscreteLayers> {
  static auto layers(MyDiscreteLayers& layers) {
    return std::make_tuple(
        namedDiscreteLayer("semantic_label", layers.semantic_label),
        namedDiscreteLayer("manually_changed", layers.manually_changed));
  }

  static auto layers(const MyDiscreteLayers& layers) {
    return std::make_tuple(
        namedDiscreteLayer("semantic_label", layers.semantic_label),
        namedDiscreteLayer("manually_changed", layers.manually_changed));
  }
};
}  // namespace wavemap::layered::io

using MyLayeredMapIo = wavemap::layered::io::LayeredMapIo<MyLayeredMap,
                                                    MyVoxelSerializer>;

inline MyLayeredMapConfig makeMyLayeredMapConfig() {
  MyLayeredMapConfig config;
  config.continuous_map.min_cell_width = 0.1f;
  config.continuous_map.min_log_odds = -100.f;
  config.continuous_map.max_log_odds = 100.f;
  config.continuous_map.tree_height = 3;
  config.continuous_threshold.data.min_temperature = -20.f;
  config.continuous_threshold.data.max_temperature = 60.f;
  config.continuous_threshold.data.min_intensity = 0.f;
  config.continuous_threshold.data.max_intensity = 1.f;
  config.continuous_pruning.occupancy.weight = 1.f;
  config.continuous_pruning.occupancy.scale = 1e-3f;
  config.continuous_pruning.data.temperature.weight = 1.f;
  config.continuous_pruning.data.temperature.scale = 1e-3f;
  config.continuous_pruning.data.intensity.weight = 1.f;
  config.continuous_pruning.data.intensity.scale = 1e-3f;
  config.continuous_pruning.combined_threshold = 1.f;
  config.continuous_pruning.data.combined_threshold = 1.f;
  config.discrete_compression.block_height = 1;
  return config;
}

#endif  // WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_CUSTOM_LAYERED_MAP_CONFIG_H_
