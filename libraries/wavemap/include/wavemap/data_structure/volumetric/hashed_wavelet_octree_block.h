#ifndef WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_HASHED_WAVELET_OCTREE_BLOCK_H_
#define WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_HASHED_WAVELET_OCTREE_BLOCK_H_

#include <chrono>

#include "wavemap/common.h"
#include "wavemap/data_structure/ndtree/ndtree.h"
#include "wavemap/data_structure/volumetric/cell_types/haar_coefficients.h"
#include "wavemap/data_structure/volumetric/cell_types/haar_transform.h"
#include "wavemap/data_structure/volumetric/cell_types/occupancy_state.h"
#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"

namespace wavemap {
class HashedWaveletOctreeBlock {
 public:
  static constexpr int kDim = 3;
  using Clock = std::chrono::steady_clock;
  using Time = std::chrono::time_point<Clock>;
  using BlockIndex = Index3D;
  using Coefficients = HaarCoefficients<FloatingPoint, kDim>;
  using Transform = HaarTransform<FloatingPoint, kDim>;
  using NodeType = NdtreeNode<typename Coefficients::Details, kDim>;

  explicit HashedWaveletOctreeBlock(IndexElement tree_height,
                                    FloatingPoint min_log_odds,
                                    FloatingPoint max_log_odds)
      : tree_height_(tree_height),
        min_log_odds_(min_log_odds),
        max_log_odds_(max_log_odds) {}

  bool empty() const {
    return ndtree_.empty() &&
           !OccupancyState::isObserved(root_scale_coefficient_);
  }
  size_t size() const { return ndtree_.size(); }
  void threshold();
  void prune();
  void clear();

  FloatingPoint getCellValue(const OctreeIndex& index) const;
  void setCellValue(const OctreeIndex& index, FloatingPoint new_value);
  void addToCellValue(const OctreeIndex& index, FloatingPoint update);

  void forEachLeaf(
      const BlockIndex& block_index,
      typename VolumetricDataStructureBase::IndexedLeafVisitorFunction
          visitor_fn,
      IndexElement termination_height = 0) const;

  Coefficients::Scale& getRootScale() { return root_scale_coefficient_; }
  const Coefficients::Scale& getRootScale() const {
    return root_scale_coefficient_;
  }
  NodeType& getRootNode() { return ndtree_.getRootNode(); }
  const NodeType& getRootNode() const { return ndtree_.getRootNode(); }

  void setNeedsPruning(bool value = true) { needs_pruning_ = value; }
  bool getNeedsPruning() const { return needs_pruning_; }
  void setNeedsThresholding(bool value = true) { needs_thresholding_ = value; }
  bool getNeedsThresholding() const { return needs_thresholding_; }
  void setLastUpdatedStamp(Time stamp = Clock::now()) {
    last_updated_stamp_ = stamp;
  }
  Time getLastUpdatedStamp() const { return last_updated_stamp_; }
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

  Ndtree<Coefficients::Details, kDim> ndtree_{tree_height_ - 1};
  Coefficients::Scale root_scale_coefficient_{};

  bool needs_thresholding_ = false;
  bool needs_pruning_ = false;
  Time last_updated_stamp_ = Clock::now();

  Coefficients::Scale recursiveThreshold(NodeType& node,
                                         Coefficients::Scale scale_coefficient);
  void recursivePrune(NodeType& node);
};
}  // namespace wavemap

#include "wavemap/data_structure/volumetric/impl/hashed_wavelet_octree_block_inl.h"

#endif  // WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_HASHED_WAVELET_OCTREE_BLOCK_H_
