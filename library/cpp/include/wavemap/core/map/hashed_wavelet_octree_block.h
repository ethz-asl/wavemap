#ifndef WAVEMAP_CORE_MAP_HASHED_WAVELET_OCTREE_BLOCK_H_
#define WAVEMAP_CORE_MAP_HASHED_WAVELET_OCTREE_BLOCK_H_

#include "wavemap/core/common.h"
#include "wavemap/core/data_structure/ndtree/ndtree.h"
#include "wavemap/core/map/cell_types/haar_coefficients.h"
#include "wavemap/core/map/cell_types/haar_transform.h"
#include "wavemap/core/map/map_base.h"
#include "wavemap/core/utils/time/time.h"

namespace wavemap {
class HashedWaveletOctreeBlock {
 public:
  static constexpr int kDim = 3;
  using BlockIndex = Index3D;
  using Coefficients = HaarCoefficients<FloatingPoint, kDim>;
  using Transform = HaarTransform<FloatingPoint, kDim>;
  using OctreeType = Octree<Coefficients::Details>;

  explicit HashedWaveletOctreeBlock(IndexElement tree_height,
                                    FloatingPoint min_log_odds,
                                    FloatingPoint max_log_odds)
      : tree_height_(tree_height),
        min_log_odds_(min_log_odds),
        max_log_odds_(max_log_odds) {}

  bool empty() const;
  size_t size() const { return ndtree_.size(); }
  void threshold();
  void prune();
  void clear();

  FloatingPoint getCellValue(const OctreeIndex& index) const;
  void setCellValue(const OctreeIndex& index, FloatingPoint new_value);
  void addToCellValue(const OctreeIndex& index, FloatingPoint update);

  void forEachLeaf(const BlockIndex& block_index,
                   typename MapBase::IndexedLeafVisitorFunction visitor_fn,
                   IndexElement termination_height = 0) const;

  Coefficients::Scale& getRootScale() { return root_scale_coefficient_; }
  const Coefficients::Scale& getRootScale() const {
    return root_scale_coefficient_;
  }
  OctreeType::NodeRefType getRootNode() { return ndtree_.getRootNode(); }
  OctreeType::NodeConstRefType getRootNode() const {
    return ndtree_.getRootNode();
  }

  void setNeedsPruning(bool value = true) { needs_pruning_ = value; }
  bool getNeedsPruning() const { return needs_pruning_; }
  void setNeedsThresholding(bool value = true) { needs_thresholding_ = value; }
  bool getNeedsThresholding() const { return needs_thresholding_; }
  void setLastUpdatedStamp(Timestamp stamp = Time::now()) {
    last_updated_stamp_ = stamp;
  }
  Timestamp getLastUpdatedStamp() const { return last_updated_stamp_; }
  FloatingPoint getTimeSinceLastUpdated() const;

  template <TraversalOrder traversal_order>
  auto getNodeIterator() {
    return ndtree_.getIterator<traversal_order>();
  }
  template <TraversalOrder traversal_order>
  auto getNodeIterator() const {
    return ndtree_.getIterator<traversal_order>();
  }

  size_t getMemoryUsage() const { return ndtree_.getMemoryUsage(); }

 private:
  const IndexElement tree_height_;
  const FloatingPoint min_log_odds_;
  const FloatingPoint max_log_odds_;

  OctreeType ndtree_{tree_height_ - 1};
  Coefficients::Scale root_scale_coefficient_{};

  bool needs_thresholding_ = false;
  bool needs_pruning_ = false;
  Timestamp last_updated_stamp_ = Time::now();

  Coefficients::Scale recursiveThreshold(OctreeType::NodeRefType node,
                                         Coefficients::Scale scale_coefficient);
  void recursivePrune(OctreeType::NodeRefType node);
};
}  // namespace wavemap

#include "wavemap/core/map/impl/hashed_wavelet_octree_block_inl.h"

#endif  // WAVEMAP_CORE_MAP_HASHED_WAVELET_OCTREE_BLOCK_H_
