#ifndef WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_OCTREE_H_
#define WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_OCTREE_H_

#include <memory>
#include <string>

#include "wavemap/data_structure/ndtree/ndtree.h"
#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"
#include "wavemap/indexing/index_conversions.h"
#include "wavemap/indexing/ndtree_index.h"

namespace wavemap {
class VolumetricOctree : public VolumetricDataStructureBase {
 public:
  using Ptr = std::shared_ptr<VolumetricOctree>;
  using ConstPtr = std::shared_ptr<const VolumetricOctree>;
  using NodeType = NdtreeNode<FloatingPoint, kDim>;

  // TODO(victorr): Make this configurable
  static constexpr NdtreeIndexElement kMaxHeight = 14;
  static constexpr bool kRequiresExplicitThresholding = true;

  // Use the base class' constructor
  using VolumetricDataStructureBase::VolumetricDataStructureBase;

  bool empty() const override { return ndtree_.empty(); }
  size_t size() const override { return ndtree_.size(); }
  void threshold() override;
  void prune() override;
  void clear() override { return ndtree_.clear(); }

  typename OctreeIndex::ChildArray getFirstChildIndices() const;

  Index3D getMinIndex() const override;
  Index3D getMaxIndex() const override;
  Index3D getMinPossibleIndex() const;
  Index3D getMaxPossibleIndex() const;

  FloatingPoint getCellValue(const Index3D& index) const override;
  void setCellValue(const Index3D& index, FloatingPoint new_value) override;
  void setCellValue(const OctreeIndex& index, FloatingPoint new_value);
  void addToCellValue(const Index3D& index, FloatingPoint update) override;
  void addToCellValue(const OctreeIndex& index, FloatingPoint update);

  void forEachLeaf(
      typename VolumetricDataStructureBase::IndexedLeafVisitorFunction
          visitor_fn) const override;

  NodeType& getRootNode() { return ndtree_.getRootNode(); }
  const NodeType& getRootNode() const { return ndtree_.getRootNode(); }
  template <TraversalOrder traversal_order>
  auto getNodeIterator() {
    return ndtree_.getIterator<traversal_order>();
  }
  template <TraversalOrder traversal_order>
  auto getNodeIterator() const {
    return ndtree_.getIterator<traversal_order>();
  }

  size_t getMemoryUsage() const override { return ndtree_.getMemoryUsage(); }

 private:
  struct StackElement {
    const OctreeIndex node_index;
    const NodeType& node;
    const FloatingPoint parent_value{};
  };

  Ndtree<FloatingPoint, kDim> ndtree_{kMaxHeight};

  static OctreeIndex getInternalRootNodeIndex() {
    return OctreeIndex{kMaxHeight, OctreeIndex::Position::Zero()};
  }
  const OctreeIndex root_node_index_offset_{kMaxHeight - 1,
                                            OctreeIndex::Position::Ones()};
  const Index3D root_index_offset_ =
      convert::nodeIndexToMinCornerIndex(root_node_index_offset_);

  OctreeIndex toInternal(const Index3D& index) const {
    return convert::indexAndHeightToNodeIndex<kDim>(index + root_index_offset_,
                                                    0);
  }
  OctreeIndex toInternal(const OctreeIndex& node_index) const {
    DCHECK_LE(node_index.height, root_node_index_offset_.height);
    const Index3D height_adjusted_offset =
        int_math::div_exp2_floor(root_index_offset_, node_index.height);
    return {node_index.height, node_index.position + height_adjusted_offset};
  }
  Index3D toExternalIndex(const Index3D& index) const {
    return index - root_index_offset_;
  }
  OctreeIndex toExternalNodeIndex(const OctreeIndex& node_index) const {
    DCHECK_LE(node_index.height, root_node_index_offset_.height);
    const Index3D height_adjusted_offset =
        int_math::div_exp2_floor(root_index_offset_, node_index.height);
    return {node_index.height, node_index.position - height_adjusted_offset};
  }

  const NodeType* getDeepestNodeAtIndex(const Index3D& index) const;

  friend class VolumetricOctreeTest_IndexConversions_Test;
};
}  // namespace wavemap

#include "wavemap/data_structure/volumetric/impl/volumetric_octree_inl.h"

#endif  // WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_OCTREE_H_
