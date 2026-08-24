#ifndef WAVEMAP_LAYERED_LAYERED_MAP_CONFIG_BUILDER_H_
#define WAVEMAP_LAYERED_LAYERED_MAP_CONFIG_BUILDER_H_

#include <utility>

#include <wavemap/core/common.h>
#include <wavemap/layered/layer_traits.h>

namespace wavemap::layered {

// User-facing facade over LayeredMap's internal threshold and pruning bundles.
// It intentionally owns no behavior: build() returns the existing Config type
// consumed by LayeredMap.
template <typename MapDefinitionT>
class LayeredMapConfigBuilder {
 public:
  using Definition = MapDefinitionT;
  using Schema = typename Definition::Schema;
  using Config = typename Definition::Config;

  class MapSettings {
   public:
    explicit MapSettings(Config& config) : config_(config) {}

    MapSettings& minCellWidth(FloatingPoint meters) {
      config_.continuous_map.min_cell_width = meters;
      return *this;
    }
    MapSettings& occupancyLogOdds(FloatingPoint min_log_odds,
                                  FloatingPoint max_log_odds) {
      config_.continuous_map.min_log_odds = min_log_odds;
      config_.continuous_map.max_log_odds = max_log_odds;
      return *this;
    }
    MapSettings& treeHeight(IndexElement tree_height) {
      config_.continuous_map.tree_height = tree_height;
      return *this;
    }
    MapSettings& onlyPruneBlocksIfUnusedFor(FloatingPoint seconds) {
      config_.continuous_map.only_prune_blocks_if_unused_for = seconds;
      return *this;
    }

   private:
    Config& config_;
  };

  class OccupancySettings {
   public:
    explicit OccupancySettings(Config& config) : config_(config) {}

    OccupancySettings& pruningScale(FloatingPoint scale) {
      config_.continuous_pruning.occupancy.scale = scale;
      return *this;
    }
    OccupancySettings& pruningWeight(FloatingPoint weight) {
      config_.continuous_pruning.occupancy.weight = weight;
      return *this;
    }

   private:
    Config& config_;
  };

  template <typename LayerTagT>
  class ContinuousLayerSettings {
   public:
    using Value = LayerValueT<LayerTagT>;

    explicit ContinuousLayerSettings(Config& config) : config_(config) {
      static_assert(
          Schema::template containsContinuous<LayerTagT>,
          "layer<LayerTagT>() requires a continuous layer in this schema.");
    }

    ContinuousLayerSettings& storageBounds(const Value& min_value,
                                           const Value& max_value) {
      auto& bounds =
          config_.continuous_threshold.data.template get<LayerTagT>();
      bounds.min = min_value;
      bounds.max = max_value;
      return *this;
    }
    ContinuousLayerSettings& pruningScale(FloatingPoint scale) {
      config_.continuous_pruning.data.template get<LayerTagT>().scale = scale;
      return *this;
    }
    ContinuousLayerSettings& pruningWeight(FloatingPoint weight) {
      config_.continuous_pruning.data.template get<LayerTagT>().weight =
          weight;
      return *this;
    }

   private:
    Config& config_;
  };

  LayeredMapConfigBuilder() = default;
  explicit LayeredMapConfigBuilder(Config config)
      : config_(std::move(config)) {}

  MapSettings map() { return MapSettings(config_); }
  OccupancySettings occupancy() { return OccupancySettings(config_); }

  template <typename LayerTagT>
  ContinuousLayerSettings<LayerTagT> layer() {
    return ContinuousLayerSettings<LayerTagT>(config_);
  }

  LayeredMapConfigBuilder& combinedPruningThreshold(FloatingPoint threshold) {
    config_.continuous_pruning.combined_threshold = threshold;
    // The additional-data bundle uses the same cutoff while scoring its own
    // fields. Keep this internal value synchronized for the user.
    config_.continuous_pruning.data.combined_threshold = threshold;
    return *this;
  }

  LayeredMapConfigBuilder& discreteBlockHeight(int block_height) {
    config_.discrete_compression.block_height = block_height;
    return *this;
  }

  const Config& config() const { return config_; }
  Config build() const { return config_; }
  Config&& moveConfig() { return std::move(config_); }

 private:
  Config config_{};
};

}  // namespace wavemap::layered

#endif  // WAVEMAP_LAYERED_LAYERED_MAP_CONFIG_BUILDER_H_
