#ifndef WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_HASHED_WAVELET_OCTREE_H_
#define WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_HASHED_WAVELET_OCTREE_H_

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>

#include "wavemap/common.h"
#include "wavemap/data_structure/ndtree/ndtree.h"
#include "wavemap/data_structure/volumetric/cell_types/haar_coefficients.h"
#include "wavemap/data_structure/volumetric/cell_types/haar_transform.h"
#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"
#include "wavemap/indexing/index_hashes.h"
#include "wavemap/utils/int_math.h"

namespace wavemap {
class HashedWaveletOctree : public VolumetricDataStructureBase {
 public:
  using Ptr = std::shared_ptr<HashedWaveletOctree>;
  using ConstPtr = std::shared_ptr<const HashedWaveletOctree>;
  using Coefficients = HaarCoefficients<FloatingPoint, 3>;
  using Transform = HaarTransform<FloatingPoint, 3>;
  using NodeType = NdtreeNode<typename Coefficients::Details, 3>;
  static constexpr bool kRequiresExplicitThresholding = true;

  using BlockIndex = Index3D;
  using CellIndex = OctreeIndex;

  class Block {
   public:
    using Clock = std::chrono::steady_clock;
    using Time = std::chrono::time_point<Clock>;

    explicit Block(HashedWaveletOctree* parent)
        : parent_(CHECK_NOTNULL(parent)) {}

    bool empty() const { return ndtree_.empty(); }
    size_t size() const { return ndtree_.size(); }
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
    NodeType& getRootNode() { return ndtree_.getRootNode(); }
    const NodeType& getRootNode() const { return ndtree_.getRootNode(); }

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
    auto getNodeIterator() {
      return ndtree_.getIterator<traversal_order>();
    }
    template <TraversalOrder traversal_order>
    auto getNodeIterator() const {
      return ndtree_.getIterator<traversal_order>();
    }

    size_t getMemoryUsage() const { return ndtree_.getMemoryUsage(); }

   private:
    Coefficients::Scale root_scale_coefficient_{};
    Ndtree<Coefficients::Details, 3> ndtree_{kTreeHeight - 1};

    HashedWaveletOctree* parent_;

    bool needs_thresholding_ = false;
    bool needs_pruning_ = false;
    Time last_updated_stamp_ = Clock::now();

    Coefficients::Scale recursiveThreshold(
        NodeType& node, Coefficients::Scale scale_coefficient);
    void recursivePrune(NodeType& node);
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
    const NodeType& node;
    const Coefficients::Scale scale_coefficient{};
  };

  static constexpr IndexElement kTreeHeight = 9;
  static constexpr IndexElement kCellsPerBlockSide =
      int_math::exp2(kTreeHeight);
  static constexpr FloatingPoint kDoNotPruneIfUsedInLastNSec = 5.f;

  std::unordered_map<BlockIndex, Block, IndexHash<3>> blocks_;

  static BlockIndex computeBlockIndexFromIndex(const Index3D& index) {
    return int_math::div_exp2_floor(index, kTreeHeight);
  }
  static BlockIndex computeBlockIndexFromIndex(const OctreeIndex& node_index) {
    const Index3D index = convert::nodeIndexToMinCornerIndex(node_index);
    return int_math::div_exp2_floor(index, kTreeHeight);
  }
  static CellIndex computeCellIndexFromBlockIndexAndIndex(
      const BlockIndex& block_index, OctreeIndex index);
};
}  // namespace wavemap

#include "wavemap/data_structure/volumetric/impl/hashed_wavelet_octree_inl.h"

#endif  // WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_HASHED_WAVELET_OCTREE_H_
