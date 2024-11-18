#ifndef WAVEMAP_CORE_MAP_HASHED_CHUNKED_WAVELET_OCTREE_BLOCK_H_
#define WAVEMAP_CORE_MAP_HASHED_CHUNKED_WAVELET_OCTREE_BLOCK_H_

#include "wavemap/core/common.h"
#include "wavemap/core/data_structure/chunked_ndtree/chunked_ndtree.h"
#include "wavemap/core/map/cell_types/haar_coefficients.h"
#include "wavemap/core/map/cell_types/haar_transform.h"
#include "wavemap/core/map/map_base.h"
#include "wavemap/core/utils/time/time.h"

namespace wavemap {
class HashedChunkedWaveletOctreeBlock {
 public:
  static constexpr int kDim = 3;
  static constexpr int kChunkHeight = 3;
  static constexpr int kMaxSupportedTreeHeight = 9;
  using BlockIndex = Index3D;
  using Coefficients = HaarCoefficients<FloatingPoint, kDim>;
  using Transform = HaarTransform<FloatingPoint, kDim>;
  using OctreeType = ChunkedOctree<Coefficients::Details, kChunkHeight>;

  explicit HashedChunkedWaveletOctreeBlock(IndexElement tree_height,
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
  OctreeType::ChunkType& getRootChunk() { return ndtree_.getRootChunk(); }
  const OctreeType::ChunkType& getRootChunk() const {
    return ndtree_.getRootChunk();
  }

  void setNeedsPruning(bool value = true) { needs_pruning_ = value; }
  bool getNeedsPruning() const { return needs_pruning_; }
  bool& getNeedsPruning() { return needs_pruning_; }

  void setNeedsThresholding(bool value = true) { needs_thresholding_ = value; }
  bool getNeedsThresholding() const { return needs_thresholding_; }

  void setLastUpdatedStamp(Timestamp stamp = Time::now()) {
    last_updated_stamp_ = stamp;
  }
  Timestamp getLastUpdatedStamp() const { return last_updated_stamp_; }
  FloatingPoint getTimeSinceLastUpdated() const;

  template <TraversalOrder traversal_order>
  auto getChunkIterator() {
    return ndtree_.getChunkIterator<traversal_order>();
  }
  template <TraversalOrder traversal_order>
  auto getChunkIterator() const {
    return ndtree_.getChunkIterator<traversal_order>();
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

  void recursiveThreshold(OctreeType::NodeRefType node,
                          Coefficients::Scale& node_scale_coefficient);
  void recursivePrune(
      HashedChunkedWaveletOctreeBlock::OctreeType::NodeRefType chunk);
};
}  // namespace wavemap

#include "wavemap/core/map/impl/hashed_chunked_wavelet_octree_block_inl.h"

#endif  // WAVEMAP_CORE_MAP_HASHED_CHUNKED_WAVELET_OCTREE_BLOCK_H_
