#ifndef WAVEMAP_LAYERED_LAYERED_MAP_H_
#define WAVEMAP_LAYERED_LAYERED_MAP_H_

#include <memory>
#include <utility>

#include <wavemap/core/map/cell_types/voxel_data.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>

#include <wavemap/layered/discrete_layer.h>

namespace wavemap::layered {

template <typename ContinuousLayersT, typename ContinuousPolicyT>
using LayeredMapVoxel =
    wavemap::VoxelData<ContinuousLayersT, ContinuousPolicyT>;

template <typename ContinuousLayersT, typename ContinuousPolicyT>
using ContinuousWaveletMapT =
    wavemap::HashedWaveletOctreeT<
        LayeredMapVoxel<ContinuousLayersT, ContinuousPolicyT>>;

template <typename ContinuousMapT>
struct LayeredMapConfig {
  wavemap::HashedWaveletOctreeConfig continuous_map;
  typename ContinuousMapT::ThresholdConfig continuous_threshold;
  typename ContinuousMapT::PruningConfig continuous_pruning;
  DiscreteCompressionConfig discrete_compression;
};

template <typename ContinuousLayersT, typename ContinuousPolicyT,
          typename DiscreteLayersT>
class LayeredMap {
 public:
  using ContinuousLayers = ContinuousLayersT;
  using ContinuousPolicy = ContinuousPolicyT;
  using ContinuousVoxel = LayeredMapVoxel<ContinuousLayersT, ContinuousPolicyT>;
  using ContinuousMap = ContinuousWaveletMapT<ContinuousLayersT,
                                             ContinuousPolicyT>;
  using Config = LayeredMapConfig<ContinuousMap>;
  using DiscreteLayers = DiscreteLayersT;

  explicit LayeredMap(const Config& config)
      : continuous_map_(std::make_shared<ContinuousMap>(
            config.continuous_map, config.continuous_threshold,
            config.continuous_pruning)),
        discrete_layers_(config.discrete_compression) {}

  explicit LayeredMap(const wavemap::HashedWaveletOctreeConfig& config)
      : continuous_map_(std::make_shared<ContinuousMap>(config)),
        discrete_layers_() {}

  LayeredMap(const wavemap::HashedWaveletOctreeConfig& config,
             const typename ContinuousMap::ThresholdConfig& threshold_config,
             const typename ContinuousMap::PruningConfig& pruning_config)
      : continuous_map_(std::make_shared<ContinuousMap>(
            config, threshold_config, pruning_config)),
        discrete_layers_() {}

  LayeredMap(typename ContinuousMap::Ptr continuous_map,
             DiscreteLayersT discrete_layers)
      : continuous_map_(std::move(continuous_map)),
        discrete_layers_(std::move(discrete_layers)) {}

  ContinuousMap& continuousMap() { return *continuous_map_; }
  const ContinuousMap& continuousMap() const { return *continuous_map_; }

  DiscreteLayersT& discreteLayers() { return discrete_layers_; }
  const DiscreteLayersT& discreteLayers() const { return discrete_layers_; }

 private:
  typename ContinuousMap::Ptr continuous_map_;
  DiscreteLayersT discrete_layers_;
};

}  // namespace wavemap::layered

#endif  // WAVEMAP_LAYERED_LAYERED_MAP_H_
