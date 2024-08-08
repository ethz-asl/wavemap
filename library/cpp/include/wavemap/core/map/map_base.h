#ifndef WAVEMAP_CORE_MAP_MAP_BASE_H_
#define WAVEMAP_CORE_MAP_MAP_BASE_H_

#include <algorithm>
#include <memory>
#include <string>

#include "wavemap/core/common.h"
#include "wavemap/core/config/config_base.h"
#include "wavemap/core/config/type_selector.h"
#include "wavemap/core/indexing/ndtree_index.h"

namespace wavemap {
struct MapType : TypeSelector<MapType> {
  using TypeSelector<MapType>::TypeSelector;

  enum Id : TypeId {
    kHashedBlocks,
    kOctree,
    kWaveletOctree,
    kHashedWaveletOctree,
    kHashedChunkedWaveletOctree
  };

  static constexpr std::array names = {
      "hashed_blocks", "octree", "wavelet_octree", "hashed_wavelet_octree",
      "hashed_chunked_wavelet_octree"};
};

/**
 * Base config struct for maps.
 */
struct MapBaseConfig : ConfigBase<MapBaseConfig, 3> {
  //! Maximum resolution of the map, set as the width of the smallest cell that
  //! it can represent.
  Meters<FloatingPoint> min_cell_width = 0.1f;

  //! Lower threshold for the occupancy values stored in the map, in log-odds.
  FloatingPoint min_log_odds = -2.f;
  //! Upper threshold for the occupancy values stored in the map, in log-odds.
  FloatingPoint max_log_odds = 4.f;

  static MemberMap memberMap;

  // Constructors
  MapBaseConfig() = default;
  MapBaseConfig(FloatingPoint min_cell_width, FloatingPoint min_log_odds,
                FloatingPoint max_log_odds)
      : min_cell_width(min_cell_width),
        min_log_odds(min_log_odds),
        max_log_odds(max_log_odds) {}

  bool isValid(bool verbose) const override;
};

class MapBase {
 public:
  static constexpr int kDim = 3;
  using Ptr = std::shared_ptr<MapBase>;
  using ConstPtr = std::shared_ptr<const MapBase>;

  explicit MapBase(const MapBaseConfig& config)
      : config_(config.checkValid()) {}
  virtual ~MapBase() = default;

  virtual bool empty() const = 0;
  virtual size_t size() const = 0;
  virtual void threshold() = 0;
  virtual void prune() = 0;
  virtual void pruneSmart() {
    // NOTE: This method can be overriden by derived classes to provide more
    //       efficient selective pruning strategies. Otherwise, just prune all.
    prune();
  }
  virtual void clear() = 0;

  FloatingPoint getMinCellWidth() const { return config_.min_cell_width; }
  FloatingPoint getMinLogOdds() const { return config_.min_log_odds; }
  FloatingPoint getMaxLogOdds() const { return config_.max_log_odds; }
  virtual size_t getMemoryUsage() const = 0;
  virtual IndexElement getTreeHeight() const = 0;

  virtual Index3D getMinIndex() const = 0;
  virtual Index3D getMaxIndex() const = 0;

  virtual FloatingPoint getCellValue(const Index3D& index) const = 0;
  virtual void setCellValue(const Index3D& index, FloatingPoint new_value) = 0;
  virtual void addToCellValue(const Index3D& index, FloatingPoint update) = 0;

  using IndexedLeafVisitorFunction =
      std::function<void(const OctreeIndex& index, FloatingPoint value)>;
  virtual void forEachLeaf(IndexedLeafVisitorFunction visitor_fn) const = 0;

 protected:
  const MapBaseConfig config_;

  FloatingPoint clamp(FloatingPoint value) const {
    return std::clamp(value, config_.min_log_odds, config_.max_log_odds);
  }
  FloatingPoint clampedAdd(FloatingPoint value, FloatingPoint update) const {
    return clamp(value + update);
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_CORE_MAP_MAP_BASE_H_
