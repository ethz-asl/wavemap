#ifndef WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_HASHED_CHUNKED_WAVELET_OCTREE_H_
#define WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_HASHED_CHUNKED_WAVELET_OCTREE_H_

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>

#include "wavemap/common.h"
#include "wavemap/data_structure/chunked_ndtree/chunked_ndtree.h"
#include "wavemap/data_structure/volumetric/cell_types/haar_coefficients.h"
#include "wavemap/data_structure/volumetric/cell_types/haar_transform.h"
#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"
#include "wavemap/indexing/index_hashes.h"
#include "wavemap/utils/int_math.h"

namespace wavemap {
class HashedChunkedWaveletOctree : public VolumetricDataStructureBase {
 public:
  static constexpr IndexElement kChunkHeight = 3;
  static constexpr IndexElement kTreeHeight = 6;

  using Ptr = std::shared_ptr<HashedChunkedWaveletOctree>;
  using ConstPtr = std::shared_ptr<const HashedChunkedWaveletOctree>;
  using Coefficients = HaarCoefficients<FloatingPoint, kDim>;
  using Transform = HaarTransform<FloatingPoint, kDim>;
  using NodeChunkType =
      NdtreeNodeChunk<typename Coefficients::Details, kDim, kChunkHeight>;
  static constexpr bool kRequiresExplicitThresholding = true;

  using BlockIndex = Index3D;
  using CellIndex = OctreeIndex;

  class Block {
   public:
    using Clock = std::chrono::steady_clock;
    using Time = std::chrono::time_point<Clock>;

    explicit Block(HashedChunkedWaveletOctree* parent)
        : parent_(CHECK_NOTNULL(parent)) {}

    bool empty() const { return chunked_ndtree_.empty(); }
    size_t size() const { return chunked_ndtree_.size(); }
    void threshold();
    void prune();

    FloatingPoint getCellValue(const OctreeIndex& index) const;
    void setCellValue(const OctreeIndex& index, FloatingPoint new_value);
    void addToCellValue(const OctreeIndex& index, FloatingPoint update);

    void forEachLeaf(
        const BlockIndex& block_index,
        typename VolumetricDataStructureBase::IndexedLeafVisitorFunction
            visitor_fn) const;

    Coefficients::Scale& getRootScale() { return root_scale_coefficient_; }
    const Coefficients::Scale& getRootScale() const {
      return root_scale_coefficient_;
    }
    NodeChunkType& getRootChunk() { return chunked_ndtree_.getRootChunk(); }
    const NodeChunkType& getRootChunk() const {
      return chunked_ndtree_.getRootChunk();
    }

    void setNeedsPruning(bool value = true) { needs_pruning_ = value; }
    bool getNeedsPruning() const { return needs_pruning_; }
    void setNeedsThresholding(bool value = true) {
      needs_thresholding_ = value;
    }
    bool getNeedsThresholding() const { return needs_thresholding_; }
    void setLastUpdatedStamp(Time stamp = Clock::now()) {
      last_updated_stamp_ = stamp;
    }
    Time getLastUpdatedStamp() const { return last_updated_stamp_; }
    FloatingPoint getTimeSinceLastUpdated() const;

    template <TraversalOrder traversal_order>
    auto getChunkIterator() {
      return chunked_ndtree_.getIterator<traversal_order>();
    }
    template <TraversalOrder traversal_order>
    auto getChunkIterator() const {
      return chunked_ndtree_.getIterator<traversal_order>();
    }

    size_t getMemoryUsage() const { return chunked_ndtree_.getMemoryUsage(); }

   private:
    Coefficients::Scale root_scale_coefficient_{};
    ChunkedNdtree<Coefficients::Details, kDim, kChunkHeight> chunked_ndtree_{
        kTreeHeight - 1};

    HashedChunkedWaveletOctree* parent_;

    bool needs_thresholding_ = false;
    bool needs_pruning_ = false;
    Time last_updated_stamp_ = Clock::now();

    struct RecursiveThresholdReturnValue {
      Coefficients::Scale scale;
      bool is_nonzero_child;
    };
    RecursiveThresholdReturnValue recursiveThreshold(
        NodeChunkType& chunk, Coefficients::Scale scale_coefficient);
    void recursivePrune(NodeChunkType& chunk);
  };

  // Use the base class' constructor
  using VolumetricDataStructureBase::VolumetricDataStructureBase;

  bool empty() const override { return blocks_.empty(); }
  size_t size() const override;
  void threshold() override;
  void prune() override;
  void pruneDistant() override;
  void clear() override { blocks_.clear(); }

  size_t getMemoryUsage() const override;

  Index3D getMinIndex() const override;
  Index3D getMaxIndex() const override;
  IndexElement getTreeHeight() const { return kTreeHeight; }
  IndexElement getChunkHeight() const { return kChunkHeight; }
  Index3D getBlockSize() const { return Index3D::Constant(kCellsPerBlockSide); }

  FloatingPoint getCellValue(const Index3D& index) const override;
  FloatingPoint getCellValue(const OctreeIndex& index) const;
  void setCellValue(const Index3D& index, FloatingPoint new_value) override;
  void addToCellValue(const Index3D& index, FloatingPoint update) override;

  bool hasBlock(const Index3D& block_index) const;
  Block& getOrAllocateBlock(const Index3D& block_index);
  Block& getBlock(const Index3D& block_index);
  const Block& getBlock(const Index3D& block_index) const;
  auto& getBlocks() { return blocks_; }
  const auto& getBlocks() const { return blocks_; }

  void forEachLeaf(
      typename VolumetricDataStructureBase::IndexedLeafVisitorFunction
          visitor_fn) const override;

 private:
  struct StackElement {
    const OctreeIndex node_index;
    const NodeChunkType& chunk;
    const Coefficients::Scale scale_coefficient{};
  };

  static constexpr IndexElement kCellsPerBlockSide =
      int_math::exp2(kTreeHeight);
  static constexpr FloatingPoint kDoNotPruneIfUsedInLastNSec = 5.f;

  std::unordered_map<BlockIndex, Block, IndexHash<kDim>> blocks_;

  static BlockIndex computeBlockIndexFromIndex(const Index3D& index) {
    return int_math::div_exp2_floor(index, kTreeHeight);
  }
  static BlockIndex computeBlockIndexFromIndex(const OctreeIndex& node_index) {
    // TODO(victorr): Divide by height difference directly instead of round trip
    //                through height 0
    const Index3D index = convert::nodeIndexToMinCornerIndex(node_index);
    return int_math::div_exp2_floor(index, kTreeHeight);
  }
  static CellIndex computeCellIndexFromBlockIndexAndIndex(
      const BlockIndex& block_index, OctreeIndex index);
};
}  // namespace wavemap

#include "wavemap/data_structure/volumetric/impl/hashed_chunked_wavelet_octree_inl.h"

#endif  // WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_HASHED_CHUNKED_WAVELET_OCTREE_H_
