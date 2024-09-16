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
  //! it can represent
  Meters<FloatingPoint> min_cell_width = 0.1f;

  //! Lower threshold for the occupancy values stored in the map, in log-odds
  FloatingPoint min_log_odds = -2.f;
  //! Upper threshold for the occupancy values stored in the map, in log-odds
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

/**
 * Base class for wavemap maps
 */
class MapBase {
 public:
  static constexpr int kDim = 3;
  using Ptr = std::shared_ptr<MapBase>;
  using ConstPtr = std::shared_ptr<const MapBase>;

  explicit MapBase(const MapBaseConfig& config)
      : config_(config.checkValid()) {}
  virtual ~MapBase() = default;

  //! Whether the map is empty
  virtual bool empty() const = 0;
  //! The number of cells or nodes in the map
  virtual size_t size() const = 0;
  //! Threshold the occupancy values of all cells in the map to stay within the
  //! range specified by its min_log_odds and max_log_odds
  virtual void threshold() = 0;
  //! Free up memory by pruning nodes that are no longer needed
  //! @note Implementations of this pruning operation should be lossless and
  //!       does not alter the estimated occupancy posterior.
  virtual void prune() = 0;
  //! Similar to prune(), but avoids de-allocating nodes that will likely be
  //! used again in the near future
  virtual void pruneSmart() {
    // NOTE: This method can be overriden by derived classes to provide more
    //       efficient selective pruning strategies. Otherwise, just prune all.
    prune();
  }
  //! Erase all cells in the map
  virtual void clear() = 0;

  //! Maximum map resolution, set as width of smallest cell it can represent
  FloatingPoint getMinCellWidth() const { return config_.min_cell_width; }
  //! Lower threshold for the occupancy values stored in the map, in log-odds
  FloatingPoint getMinLogOdds() const { return config_.min_log_odds; }
  //! Upper threshold for the occupancy values stored in the map, in log-odds
  FloatingPoint getMaxLogOdds() const { return config_.max_log_odds; }
  //! The amount of memory used by the map, in bytes
  virtual size_t getMemoryUsage() const = 0;
  //! Height of the octree used to store the map
  //! @note This value is only defined for multi-resolution maps.
  virtual IndexElement getTreeHeight() const = 0;

  //! Index of the minimum corner of the map's Axis Aligned Bounding Box
  virtual Index3D getMinIndex() const = 0;
  //! Index of the maximum corner of the map's Axis Aligned Bounding Box
  virtual Index3D getMaxIndex() const = 0;

  //! Query the value of the map at a given index
  virtual FloatingPoint getCellValue(const Index3D& index) const = 0;
  //! Set the value of the map at a given index
  virtual void setCellValue(const Index3D& index, FloatingPoint new_value) = 0;
  //! Increment the value of the map at a given index
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
